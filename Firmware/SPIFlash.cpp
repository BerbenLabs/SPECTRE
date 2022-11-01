/* Device Driver To Work With MCU (ESP32) 
 * 
 * Subject: S70FL01GSAGMFI011 SPI Flash - Cypress
 * Project: SPECTRE [3.0]
 * 
 * Parts Of This Software Have Been Copied From:
 * https://github.com/kriswiner/SPIFlash
 * 
 * This Device Driver is Designed For SPI Communication Interface
 */

#include "SPIFLASH.h"

SPIClass SPIFlash(HSPI);
SPISettings settingsFlash(10000000, MSBFIRST, SPI_MODE0);

unsigned char flash_wait_for_write = 0; // Indicator For Function write_pause(), = 0 If No Write Operation In Progress
volatile uint8_t CS_PIN = 0;
cbDelete_t Func_cbDelete = NULL;

// ----------------------------------------------------------------------------------------------------------------------------------------//
// -------- FUNCTIONS ---------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------------------------------------------------------------------------------------------//
//=============================================================================
/*
 * Changes The Chip Select Pin To Select The First Flash Bank
 * Parameters:  -
 * Return:      -
 */
void selectBank1()
{
  CS_PIN = FLASH_CSN1;
}
//=============================================================================
//=============================================================================
/*
 * Changes The Chip Select Pin To Select The Second Flash Bank
 * Parameters:  -
 * Return:      -
 */
void selectBank2()
{
  CS_PIN = FLASH_CSN2;
}
//=============================================================================
//=============================================================================
/*
 * Initilisation Of Flash: SPI, CS Pins, Enabling 4-Byte-Address Mode
 * Parameters:  -
 * Return:      Boolean true Or false If Enabling 4-Byte-Address Mode Was Successful Or Not
 */
bool flash_init( cbDelete_t cb )
{
  static unsigned char Flash_Page_Buffer[FLASH_PAGE_SIZE];
  unsigned char Flash_ID[20]; // or Use With Function flash_read_id(Flash_ID)
  
  pinMode(FLASH_CSN1, OUTPUT);
  pinMode(FLASH_CSN2, OUTPUT);
  digitalWrite(FLASH_CSN1, HIGH);
  digitalWrite(FLASH_CSN2, HIGH);
  
  SPIFlash.begin();
  delay(100);

  // Initialise Callback Function For flash_chip_erase, e.g. For LED Indication
  Func_cbDelete = cb;
  
  // Set Flash To 4-Byte-Address Mode (Use Full Flash Size) By Setting Bit EXTADD Of BAR Register (0x17) 
  // Select Bank 1 And Switch To 4-Byte-Address Mode, Check If Successful
  selectBank1();
  if(!(setFlashBAR(0x80) & 0x80))
    return false;

  // Select Bank 2 And Switch To 4-Byte-Address Mode, Check If Successful
  selectBank2();
  if(!(setFlashBAR(0x80) & 0x80))
    return false;

  return true;
}
//=============================================================================
//=============================================================================
/*
 * Changes The Chip Select Pin To Select The Flash Bank Depending On Selected 
 * Page Number During Write/Read Operations
 * Parameters:  *pn = Pointer To Page Number To Read/Write From/To
 * Return:      Boolean true Or false If Error Or Success
 */
bool selectBank(int *pn)
{
  // Check If Page Number Is Valid 
  if(*pn <0 && *pn>= 2*FLASH_BANK_PAGE_MAX)
    return false;

  // Check If Page Number Is Higher Than Maximum Page Number Of A Single Bank
  if(*pn >= FLASH_BANK_PAGE_MAX)
  {
    // If Yes, Select Bank 2 And Start Counting From 0
    selectBank2();
    *pn -=  FLASH_BANK_PAGE_MAX; // 
  }    
  else  // If Not, Select Bank 1
    selectBank1();

  return true;
}
//=============================================================================
//=============================================================================
/*
 * Checks If Value In Transferred Array Address Equals 0xFF
 * Parameters:  unsigned char * page = Data Read Out From Flash Page | int length = Length Of Data In Byte
 * Return:      Boolean true Or false If Empty (= 0xFF) Or Not Empty (!= 0xFF)
 */
bool isPageEmpty(unsigned char * page, int length)
{
  for (int i = 0; i < length; i++)
  {
    if (page[i] != 0xFF)
    {
      return false;
    }
  }
  return true;
}
//=============================================================================
//=============================================================================
/*
 * Reads Status Register Of Flash 
 * Parameters:  -
 * Return:      unsigned char c = Status Register Data
 */
unsigned char flash_read_status(void)
{
  unsigned char c;

  // This Can't Do A write_pause
  SPIFlash.beginTransaction(settingsFlash);
  digitalWrite(CS_PIN, LOW);
  SPIFlash.transfer(CMD_READ_STATUS_REG);
  c = SPIFlash.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPIFlash.endTransaction();
  return (c);
}
//=============================================================================
//=============================================================================
/*
 * Wait For Any Current WRITE Operation In Progress To End (As Specified In 
 * WIP Bit In Status Register (STAT_WIP)) - resumes if STAT_WIP is cleared
 * Parameters:  -
 * Return:      -
 */
void write_pause(void)
{
  if (flash_wait_for_write) {
    while (flash_read_status() & STAT_WIP);
    flash_wait_for_write = 0;
  }
}
//=============================================================================
//=============================================================================
/*
 * Write Data To Flash Page By Page = Host To Memory Transfer
 * Parameters:  *wp = Data To Write | pn = Page Number
 * Return:      int bytes_written = Number Of Bytes Written
 */
int flash_page_program(unsigned char *wp, int pn)
{
  int address;
  int bytes_written = 0;

  // Check If Page Number Is Valid And Select The Right Memory Bank
  if(!selectBank(&pn))
    return bytes_written;
    
  write_pause();
  
  // Send WRITE Enable Command
  SPIFlash.beginTransaction(settingsFlash);
  digitalWrite(CS_PIN, LOW);
  SPIFlash.transfer(CMD_WRITE_ENABLE);
  digitalWrite(CS_PIN, HIGH);

  digitalWrite(CS_PIN, LOW);
  SPIFlash.transfer(CMD_PAGE_PROGRAM);
  // Send The 4-Byte-Address Of The Page
  address = pn << 8;
  SPIFlash.transfer((address >> 24) & 0xFF);
  SPIFlash.transfer((address >> 16) & 0xFF);
  SPIFlash.transfer((address >> 8) & 0xFF);
  SPIFlash.transfer(address & 0xFF);
  
  // Now Write 256 Bytes To The Page
  for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i++) {
    SPIFlash.transfer(*wp++);
    bytes_written++;
  }
  digitalWrite(CS_PIN, HIGH);
  SPIFlash.endTransaction();
  
  // Indicate That Next I/O Must Wait For This WRITE Process To Finish
  flash_wait_for_write = 1;

  return bytes_written;
}
//=============================================================================
//=============================================================================
/*
 * Normal Speed Page Readout Of Flash (Speed = 6 MBps (50 MHz)) = Memory To Host Transfer
 * Parameters:  *P = Address Of Array To Write Data To | pn = Page Number | n_pages = Number Of Pages To Read From
 * Return:      int bytes_read = Number Of Bytes Read
 */
int flash_read_pages(unsigned char *p, int pn, const int n_pages)
{
  int address;
  unsigned char *rp = p;
  int bytes_read = 0;

  // Check If Page Number Is Valid And Select The Right Memory Bank
  if(!selectBank(&pn))
    return bytes_read;
  
  write_pause();
  SPIFlash.beginTransaction(settingsFlash);
  // Send READ Enable Command
  digitalWrite(CS_PIN, LOW);
  SPIFlash.transfer(CMD_READ_DATA);
  
  // Send The 4-Byte-Address Of The Page
  address = pn << 8;
  SPIFlash.transfer((address >> 24) & 0xFF);
  SPIFlash.transfer((address >> 16) & 0xFF);
  SPIFlash.transfer((address >> 8) & 0xFF);
  SPIFlash.transfer(address & 0xFF);
  
  // Send Dummy Byte And Read The Page's Data Bytes
  for (uint32_t i = 0; i < n_pages * FLASH_PAGE_SIZE; i++) {
    *rp++ = SPIFlash.transfer(0);
    bytes_read++;
  }
  digitalWrite(CS_PIN, HIGH);
  SPIFlash.endTransaction();

  return bytes_read;
}
//=============================================================================
//=============================================================================
/*
 * Fast Readout Of Flash (Speed = 17 MBps (133 MHz)) = Memory To Host Transfer
 * Parameters:  *P = Address Of Array To Write Data To | pn = Page Number | n_pages = Number Of Pages To Read From
 * Return:      int bytes_read = Number Of Bytes Read
 */
int flash_fast_read_pages(unsigned char *p, int pn, const int n_pages)
{
  int address;
  unsigned char *rp = p;
  int bytes_read = 0;

  // Check If Page Number Is Valid And Select The Right Memory Bank
  if(!selectBank(&pn))
    return bytes_read;
    
  write_pause();
  SPIFlash.beginTransaction(settingsFlash);
  // Send FAST READ Enable Command
  digitalWrite(CS_PIN, LOW);
  SPIFlash.transfer(CMD_READ_HIGH_SPEED);
  
// Send The 4-Byte-Address Of The Page
  address = pn << 8;
  SPIFlash.transfer((address >> 24) & 0xFF);
  SPIFlash.transfer((address >> 16) & 0xFF);
  SPIFlash.transfer((address >> 8) & 0xFF);
  SPIFlash.transfer(address & 0xFF);
  
  // Send Dummy Byte And Read The Page's Data Bytes
  SPIFlash.transfer(0);
  for (int i = 0; i < n_pages * FLASH_PAGE_SIZE; i++) {
    *rp++ = SPIFlash.transfer(0);
    bytes_read++;
  }
  digitalWrite(CS_PIN, HIGH);
  SPIFlash.endTransaction();

  return bytes_read;
}
//=============================================================================
//=============================================================================
/*
 * Send Erase Command To Erase Flash And Wait Until Erasing Has Finished
 * Parameters:  Boolean true Or false
 * Return:      Boolean true Or false Ff Erase Error Cccurred (As Stated In E_ERR In Status REG)
 */
bool flash_chip_erase(boolean wait, int cb_delay)
{
  // Subsequently Select Both Memory Banks
  for(int i = 0; i < 2; i++)
  {
    if(i==0)
      selectBank1();
    else
      selectBank2();
    
    write_pause();
    // Send WRITE Enable command
    SPIFlash.beginTransaction(settingsFlash);
    digitalWrite(CS_PIN, LOW);
    SPIFlash.transfer(CMD_WRITE_ENABLE);
    digitalWrite(CS_PIN, HIGH);
    
    // Send Chip Erase Command
    digitalWrite(CS_PIN, LOW);
    SPIFlash.transfer(CMD_CHIP_ERASE);
    digitalWrite(CS_PIN, HIGH);
    SPIFlash.endTransaction();
    flash_wait_for_write = 1;

    // As Long As Flash Is Erasing (Wait == true) Use Callback Function To Blink LED In Main
    if (wait)
    { 
      if (flash_wait_for_write) 
      {
        while (flash_read_status() & STAT_WIP)
        {
          if(Func_cbDelete != NULL)
          {
            Func_cbDelete();
            delay(cb_delay);
          }
        }
        flash_wait_for_write = 0;
      }   
    }

    // Check Status Register Bit If Error Occured
    if(flash_read_status() & STAT_E_ERR)
      return false;

    delay(100);
  }

  return true;
}
//=============================================================================
//=============================================================================
/*
 * Read ID Of Flash
 * Parameters:  Pointer To unsigned char Array, Where ID Can Be Stored
 * Return:      -
 */
void flash_read_id(unsigned char *idt)
{
  write_pause();
  
  //Read ID
  SPIFlash.beginTransaction(settingsFlash);
  digitalWrite(CS_PIN, LOW);
  SPIFlash.transfer(CMD_READ_ID);
  for (int i = 0; i < 20; i++) {
    *idt++ = SPIFlash.transfer(0x00);
  }
  digitalWrite(CS_PIN, HIGH);
  SPIFlash.endTransaction();
}
//=============================================================================
//=============================================================================
/*
 *  Sets Flash To 4-Byte-Address Mode To Enable Use Of Full Flash Size By 
 *  Setting EXTADD Bit Of BAR Register And Reads Value Of BAR Register 
 *  To Check If Operation Was Successful (= Edits The Writeable Size Of 
 *  The Flash To Full Size (4-byte-mode))
 *  Parameters: REG_Data = Data To Write To BAR Register
 *  Return:     unsigned char REG = Read Out Value Of BAR Register 
 */
unsigned char setFlashBAR(byte REG_Data)
{
  unsigned char REG;

  SPIFlash.beginTransaction(settingsFlash);
  // Send BAR Register WRITE Command, Write Data To Register
  digitalWrite(CS_PIN, LOW);
  SPIFlash.transfer(BRWR);
  SPIFlash.transfer(REG_Data);
  digitalWrite(CS_PIN, HIGH);

  // Send BAR Register READ Command, Read Out BAR Register Value
  digitalWrite(CS_PIN, LOW);
  SPIFlash.transfer(BRRD);
  REG = SPIFlash.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPIFlash.endTransaction();
  
  return REG;
}
