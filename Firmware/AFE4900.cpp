/* Device Driver To Work With MCU (ESP32)  
 * 
 * Subject: AFE4900 - Texas Instruments
 * Project: SPECTRE [3.0]
 * 
 * For Details On The AFE4900, See:
 * http://www.ti.com/lit/ds/symlink/afe4900.pdf  |  http://www.ti.com/product/AFE4900
 * 
 * This Device Driver is Designed For SPI Communication Interface
 */

#include "AFE4900.h"
#include <SPI.h>

SPIClass SPIAFE(VSPI);
SPISettings settingsAFE(4000000, MSBFIRST, SPI_MODE0);     // Maximum Clock Speed: 4MHz   | Tested With EVM - MSBFIRST And SPIMODE 0 Are Correct
// Conversion: Output Data From ADC Is Un-Decimated 2's Complement Format (See AFE4900_EVAL.pdf, p. 44)
// ----------------------------------------------------------------------------------------------------------------------------------------//
// -------- REG CONFIG --------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------------------------------------------------------------------------------------------//
/* 
 *  21.02.2020 - LED Current Optimized, FIFO Partition = 0000, FIFO_PERIOD = 15 (16*4 Samples To Store Before FIFO Readout = 1 FLashpage), 
 *  ECG Gain = 2, LED Green = 0.39 mA | LED Red = 7.995 mA | LED IR = 7.99 mA
 */
uint32_t Reg_Cfg_AFE4900 [][2]= {
{0x00, 0x000060}, /*CONTROL0*/
{0x01, 0x000001}, /*LED2STC*/
{0x02, 0x000002}, /*LED2ENDC*/
{0x03, 0x000008}, /*LED1LEDSTC*/
{0x04, 0x00000A}, /*LED1LEDENDC*/
{0x05, 0x000005}, /*ALED2STC*/
{0x06, 0x000006}, /*ALED2ENDC*/
{0x07, 0x000009}, /*LED1STC*/
{0x08, 0x00000A}, /*LED1ENDC*/
{0x09, 0x000000}, /*LED2LEDSTC*/
{0x0A, 0x000002}, /*LED2LEDENDC*/
{0x0B, 0x00000D}, /*ALED1STC*/
{0x0C, 0x00000E}, /*ALED1ENDC*/
{0x0D, 0x000004}, /*LED2CONVST*/
{0x0E, 0x000007}, /*LED2CONVEND*/
{0x0F, 0x000009}, /*ALED2CONVST*/
{0x10, 0x00000C}, /*ALED2CONVEND*/
{0x11, 0x00000E}, /*LED1CONVST*/
{0x12, 0x000011}, /*LED1CONVEND*/
{0x13, 0x000013}, /*ALED1CONVST*/
{0x14, 0x000016}, /*ALED1CONVEND*/
{0x1D, 0x00001F}, /*PRPCOUNT*/
{0x1E, 0x000101}, /*CONTROL1*/
{0x1F, 0x000000}, /*TIAGAIN_2_3*/
{0x20, 0x000003}, /*TIAGAIN*/
{0x21, 0x000003}, /*TIA_AMB_GAIN*/
{0x22, 0x58A280}, /*LEDCNTRL1*/
{0x23, 0x104018}, /*CONTROL2*/
{0x24, 0x000000}, /*LEDCNTRL2*/
{0x28, 0x000000}, /*TOGGLE*/
{0x29, 0x000000}, /*CLKDIV1*/
{0x2A, 0x000000}, /*LED2VAL*/
{0x2B, 0x000000}, /*ALED2VAL*/
{0x2C, 0x000000}, /*LED1VAL*/
{0x2D, 0x000000}, /*ALED1VAL*/
{0x2E, 0x000000}, /*LED2-ALED2VAL*/
{0x2F, 0x000000}, /*LED1-ALED1VAL*/
{0x31, 0x000020}, /*CONTROL3*/
{0x34, 0x000000}, /*PROG_INT2_STC*/
{0x35, 0x000000}, /*PROG_INT2_ENDC*/
{0x36, 0x000004}, /*LED3LEDSTC*/
{0x37, 0x000006}, /*LED3LEDENDC*/
{0x39, 0x000000}, /*CLKDIV2*/
{0x3A, 0x100200}, /*OFFDAC*/
{0x3B, 0x000000}, /*THRDETLOW*/
{0x3C, 0x000000}, /*THRDETHIGH*/
{0x3D, 0x000000}, /*THRDET*/
{0x3E, 0x000000}, /*I_OFFDAC*/
{0x3F, 0x000000}, /*AVG_LED2_ALED2VAL*/
{0x40, 0x000000}, /*AVG_LED1_ALED1VAL*/
{0x42, 0x0003E0}, /*FIFO*/
{0x43, 0x00000C}, /*LED4LEDSTC*/
{0x44, 0x00000E}, /*LED4LEDENDC*/
{0x45, 0x000000}, /*TG_PD1STC*/
{0x46, 0x000000}, /*TG_PD1ENDC*/
{0x47, 0x000000}, /*TG_PD2STC*/
{0x48, 0x000000}, /*TG_PD2ENDC*/
{0x49, 0x000000}, /*TG_PD3STC*/
{0x4A, 0x000000}, /*TG_PD3ENDC*/
{0x4B, 0x000000}, /*CONTROL4*/
{0x4E, 0x000004}, /*DUAL_PD*/
{0x50, 0x040018}, /*CONTROL5*/
{0x51, 0x000000}, /*FIFO_OFFSET*/
{0x52, 0x00001D}, /*DATA_RDY_STC*/
{0x53, 0x00001D}, /*DATA_RDY_ENDC*/
{0x54, 0x000000}, /*MASK_PPG*/
{0x57, 0x000000}, /*PROG_INT1_STC*/
{0x58, 0x000000}, /*PROG_INT1_ENDC*/
{0x61, 0x080000}, /*ECG_CHOP*/
{0x62, 0x800000}, /*ECG_RLD*/
{0x63, 0x000000}, /*RCOMP*/
{0x64, 0x000000}, /*DYN_TIA_STC*/
{0x65, 0x000020}, /*DYN_TIA_ENDC*/
{0x66, 0x000000}, /*DYN_ADC_STC*/
{0x67, 0x000020}, /*DYN_ADC_ENDC*/
{0x68, 0x000000}, /*DYN_CLOCK_STC*/
{0x69, 0x000020}, /*DYN_CLOCK_ENDC*/
{0x6A, 0x000022}, /*DEEP_SLEEP_STC*/
{0x6B, 0x00001F}, /*DEEP_SLEEP_ENDC*/
{0x6C, 0x000000}, /*PD_SHORT*/
{0x6D, 0x000000}, /*REG_POINTER*/
{0x72, 0x000000}, /*LED_DRIVER_CONTROL*/
{0x73, 0x000000}, /*THR_DETECT_LOGIC*/
{0xFF, 0xFFFFFF}
};
/* 
 *  22.12.2019 - LED Current Optimized, FIFO Partition = 0000, FIFO_PERIOD = 7 (8*4 Samples To Store Before FIFO Readout), 
 *  ECG Gain = 2, LED Green (Both, Both Drivers) = Each 0.39 mA | LED Red = 7.995 mA | LED IR = 7.99 mA
 */
// 
// uint32_t Reg_Cfg_AFE4900 [][2]= {
//{0x00, 0x000060}, /*CONTROL0*/
//{0x01, 0x000001}, /*LED2STC*/
//{0x02, 0x000002}, /*LED2ENDC*/
//{0x03, 0x000008}, /*LED1LEDSTC*/
//{0x04, 0x00000A}, /*LED1LEDENDC*/
//{0x05, 0x000005}, /*ALED2STC*/
//{0x06, 0x000006}, /*ALED2ENDC*/
//{0x07, 0x000009}, /*LED1STC*/
//{0x08, 0x00000A}, /*LED1ENDC*/
//{0x09, 0x000000}, /*LED2LEDSTC*/
//{0x0A, 0x000002}, /*LED2LEDENDC*/
//{0x0B, 0x00000D}, /*ALED1STC*/
//{0x0C, 0x00000E}, /*ALED1ENDC*/
//{0x0D, 0x000004}, /*LED2CONVST*/
//{0x0E, 0x000007}, /*LED2CONVEND*/
//{0x0F, 0x000009}, /*ALED2CONVST*/
//{0x10, 0x00000C}, /*ALED2CONVEND*/
//{0x11, 0x00000E}, /*LED1CONVST*/
//{0x12, 0x000011}, /*LED1CONVEND*/
//{0x13, 0x000013}, /*ALED1CONVST*/
//{0x14, 0x000016}, /*ALED1CONVEND*/
//{0x1D, 0x00001F}, /*PRPCOUNT*/
//{0x1E, 0x000101}, /*CONTROL1*/
//{0x1F, 0x000000}, /*TIAGAIN_2_3*/
//{0x20, 0x000003}, /*TIAGAIN*/
//{0x21, 0x000003}, /*TIA_AMB_GAIN*/
//{0x22, 0x54A280}, /*LEDCNTRL1*/
//{0x23, 0x104018}, /*CONTROL2*/
//{0x24, 0x000000}, /*LEDCNTRL2*/
//{0x28, 0x103531}, /*TOGGLE*/
//{0x29, 0x000000}, /*CLKDIV1*/
//{0x2A, 0x02ACFD}, /*LED2VAL*/
//{0x2B, 0x0034AF}, /*ALED2VAL*/
//{0x2C, 0x1FFF19}, /*LED1VAL*/
//{0x2D, 0xFFFA04}, /*ALED1VAL*/
//{0x2E, 0x02784E}, /*LED2-ALED2VAL*/
//{0x2F, 0x200515}, /*LED1-ALED1VAL*/
//{0x31, 0x000020}, /*CONTROL3*/
//{0x34, 0x000000}, /*PROG_INT2_STC*/
//{0x35, 0x000000}, /*PROG_INT2_ENDC*/
//{0x36, 0x000004}, /*LED3LEDSTC*/
//{0x37, 0x000006}, /*LED3LEDENDC*/
//{0x39, 0x000000}, /*CLKDIV2*/
//{0x3A, 0x100200}, /*OFFDAC*/
//{0x3B, 0x000000}, /*THRDETLOW*/
//{0x3C, 0x000000}, /*THRDETHIGH*/
//{0x3D, 0x000000}, /*THRDET*/
//{0x3E, 0x000000}, /*I_OFFDAC*/
//{0x3F, 0x000000}, /*AVG_LED2_ALED2VAL*/
//{0x40, 0x000000}, /*AVG_LED1_ALED1VAL*/
//{0x42, 0x0001E0}, /*FIFO*/
//{0x43, 0x00000C}, /*LED4LEDSTC*/
//{0x44, 0x00000E}, /*LED4LEDENDC*/
//{0x45, 0x000000}, /*TG_PD1STC*/
//{0x46, 0x000000}, /*TG_PD1ENDC*/
//{0x47, 0x000000}, /*TG_PD2STC*/
//{0x48, 0x000000}, /*TG_PD2ENDC*/
//{0x49, 0x000000}, /*TG_PD3STC*/
//{0x4A, 0x000000}, /*TG_PD3ENDC*/
//{0x4B, 0x000000}, /*CONTROL4*/
//{0x4E, 0x000004}, /*DUAL_PD*/
//{0x50, 0x040018}, /*CONTROL5*/
//{0x51, 0x000000}, /*FIFO_OFFSET*/
//{0x52, 0x00001D}, /*DATA_RDY_STC*/
//{0x53, 0x00001D}, /*DATA_RDY_ENDC*/
//{0x54, 0x000000}, /*MASK_PPG*/
//{0x57, 0x000000}, /*PROG_INT1_STC*/
//{0x58, 0x000000}, /*PROG_INT1_ENDC*/
//{0x61, 0x080000}, /*ECG_CHOP*/
//{0x62, 0x800000}, /*ECG_RLD*/
//{0x63, 0x000000}, /*RCOMP*/
//{0x64, 0x000000}, /*DYN_TIA_STC*/
//{0x65, 0x000020}, /*DYN_TIA_ENDC*/
//{0x66, 0x000000}, /*DYN_ADC_STC*/
//{0x67, 0x000020}, /*DYN_ADC_ENDC*/
//{0x68, 0x000000}, /*DYN_CLOCK_STC*/
//{0x69, 0x000020}, /*DYN_CLOCK_ENDC*/
//{0x6A, 0x000022}, /*DEEP_SLEEP_STC*/
//{0x6B, 0x00001F}, /*DEEP_SLEEP_ENDC*/
//{0x6C, 0x000000}, /*PD_SHORT*/
//{0x6D, 0x00001F}, /*REG_POINTER*/
//{0x72, 0x000010}, /*LED_DRIVER_CONTROL*/
//{0x73, 0x000000}, /*THR_DETECT_LOGIC*/
//{0xFF, 0xFFFFFF}
//};

// ----------------------------------------------------------------------------------------------------------------------------------------//
// -------- FUNCTIONS ---------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------------------------------------------------------------------------------------------//

//============================================================================
/*
 * Initialisation Of AFE4900 - Configuration Of Registers, SPI Communication And MCU Pins
 * Parameters:  -
 * Return:      -
 */
  void afe_init(void* ISR)
  {   
     // Initialise I/O Pins:                     
      pinMode(AFE_CSN,      OUTPUT); 
      pinMode(I2C_SPI_SEL,  OUTPUT);
      pinMode(RESETZ,       OUTPUT);
      pinMode(AFE_ADC_RDY,  INPUT_PULLUP);
      pinMode(PROG_OUT1,    INPUT);
      
      // Take The Chip Select High To De-Select:
      digitalWrite(AFE_CSN, HIGH);
      
      attachInterrupt(digitalPinToInterrupt(AFE_ADC_RDY), (void (*)())ISR, HIGH);
    
      // Reset AFE4900 
      digitalWrite(RESETZ, HIGH);
      delay(100);
          
      digitalWrite(RESETZ, LOW);
      delayMicroseconds(35);
    
      digitalWrite(RESETZ, HIGH);
    
      delay(10);
      
      // Enable SPI Interface
      digitalWrite(I2C_SPI_SEL, HIGH);            
      digitalWrite(RESETZ, HIGH);
      SPIAFE.begin();

      // Load Register Settings - Setup AFE4900
      delay(10);
      for(int i=0;;i++)
      {
        if(Reg_Cfg_AFE4900[i][0] == 0xFF)
          break;

        AFE_Reg_Write(Reg_Cfg_AFE4900[i][0], Reg_Cfg_AFE4900[i][1]);
      }
  }
//=============================================================================
//=============================================================================
/*
 * Every Sample Period Read Data From FIFO Of AFE4900 
 * Parameters:  int32_t* RED, int32_t* IR, int32_t* GREEN, int32_t* ECG - Address Pointer To Store Read-Out Data
 * Return:      -
 */
void AFE_Read_FIFO(int32_t* RED, int32_t* IR, int32_t* GREEN, int32_t* ECG)
{
    int i;
    uint8_t SPI_TX_Buff;
    char SPI_temp_32b[4] = {0};

   // Take The Chip Select LOW To Select The Device:
    SPIAFE.beginTransaction(settingsAFE);
    digitalWrite(AFE_CSN, LOW);
    
   // Combine READ Command With Address Of FIFO (0xFF):
    SPI_TX_Buff = 0xFF | REG_READ;
    SPIAFE.transfer(SPI_TX_Buff); //Send Register Location And Command

    // Read Out FIFO (MSBFIRST) (Data = 2's Complement Format), Switch Bytes
    for ( i = 0; i < 3; i++) SPI_temp_32b[i] = SPIAFE.transfer(0xff);
    *RED = (((int32_t)SPI_temp_32b[0]) << 16) | ((int32_t)SPI_temp_32b[1] << 8)| (int32_t)SPI_temp_32b[2];

    for ( i = 0; i < 3; i++) SPI_temp_32b[i] = SPIAFE.transfer(0xff);
    *IR = (((int32_t)SPI_temp_32b[0]) << 16) | ((int32_t)SPI_temp_32b[1] << 8)| (int32_t)SPI_temp_32b[2];

    for ( i = 0; i < 3; i++) SPI_temp_32b[i] = SPIAFE.transfer(0xff);
    *GREEN = (((int32_t)SPI_temp_32b[0]) << 16) | ((int32_t)SPI_temp_32b[1] << 8)| (int32_t)SPI_temp_32b[2];

    for ( i = 0; i < 3; i++) SPI_temp_32b[i] = SPIAFE.transfer(0xff);
    *ECG = (((int32_t)SPI_temp_32b[0]) << 16) | ((int32_t)SPI_temp_32b[1] << 8)| (int32_t)SPI_temp_32b[2];
    
    // Take The Chip Select High To De-Select:
    digitalWrite(AFE_CSN, HIGH);
    SPIAFE.endTransaction();
}
//=============================================================================
//=============================================================================
/*
 * Read From Any Register Of AFE4900
 * Parameters:  uint8_t Reg_address = Address Of Register To Read From
 * Return:      int32_t SPI_temp_32b Data Read From Register
 */
int32_t AFE_Reg_Read(uint8_t Reg_address)
{
    int i;
    uint8_t SPI_TX_Buff;
    char SPI_temp_32b[4] = {0};                             

    int time = micros();
   // Take The Chip Select LOW To Select The Device:
    SPIAFE.beginTransaction(settingsAFE);
    digitalWrite(AFE_CSN, LOW);
    
   // Combine READ Command With Address Of Read-Register:
    SPI_TX_Buff = Reg_address | REG_READ;
    SPIAFE.transfer(SPI_TX_Buff); //Send Register Location + Command

   // Read 3 Byte Of Data From Register:
    for ( i = 0; i < 3; i++)
    {
       SPI_temp_32b[i] = SPIAFE.transfer(0xff);
    }
    
   // Take The Chip Select High To De-Select:
    digitalWrite(AFE_CSN, HIGH);
    SPIAFE.endTransaction();
 
    return (((int32_t)(signed char)SPI_temp_32b[0]) << 16) | ((int32_t)SPI_temp_32b[1] << 8)| (int32_t)SPI_temp_32b[2];
}
//=============================================================================
//=============================================================================
/*
 * Write To Any Register Of AFE4900
 * Parameters:  unsigned char WRITE_ADDRESS = Register Address To Write To | unsigned long data = Data To Write To Register Address
 * Return:      -
 */
void AFE_Reg_Write (unsigned char WRITE_ADDRESS, unsigned long data)
{
    // Combine WRITE Command With Address Of Write-Register:
    byte dataToSend = (WRITE_ADDRESS<<0) | REG_WRITE;
    uint8_t regData1, regData2, regData3;

    // Split Data Into 3 Separate Bytes
     regData1 = (uint8_t)(data>>16);
     regData2 = (uint8_t)(data>>8);
     regData3 = (uint8_t)(data);

     SPIAFE.beginTransaction(settingsAFE);
    // Take The Chip Select LOW To Select The Device:
     digitalWrite(AFE_CSN, LOW);
     
     SPIAFE.transfer(dataToSend);   //Send Register Location
     SPIAFE.transfer(regData1);     //Send First Byte Of Data
     SPIAFE.transfer(regData2);     //Send Second Byte Of Data
     SPIAFE.transfer(regData3);     //Send Thrid Byte Of Data
     
    // Take The Chip Select High To De-Select:
     digitalWrite(AFE_CSN, HIGH);
     SPIAFE.endTransaction();
}
//=============================================================================
