/* Device Driver To Work With MCU (ESP32) 
 * 
 * Subject: MAX30003 - Maxim Integrated
 * Project: SPECTRE [3.0]
 * 
 * For More Information About MAX30003:
 * https://datasheets.maximintegrated.com/en/ds/MAX30003WING.pdf
 * https://datasheets.maximintegrated.com/en/ds/MAX30003.pdf
 * https://github.com/Protocentral/protocentral_max30003
 */
 
/*
 * Parts of this software have been copied from: https://github.com/Protocentral/protocentral_max30003
 * 
 *   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 *   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *   For information on how to use, visit https://github.com/Protocentral/protocentral_max30003
 */

 #include "MAX30003.h"
 #include <SPI.h>

 SPIClass SPIMAX(HSPI);                                     // Initialise SPI Class
 SPISettings settingsMAX(4000000, MSBFIRST, SPI_MODE0);     // Maximum Clock Speed: 4MHz         

 // MAX30003 Register Settings
 uint32_t max_reg_cfg[]{
  (CNFG_GEN     << 24 | 0x081007),
  (CNFG_CAL     << 24 | 0x720000),
  (CNFG_EMUX    << 24 | 0x0B0000),
  (CNFG_ECG     << 24 | 0x805000),
  (CNFG_RTOR1   << 24 | 0x3FC600),
  0xFFFFFFFF
 };

// ----------------------------------------------------------------------------------------------------------------------------------------//
// -------- FUNCTIONS ---------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------------------------------------------------------------------------------------------//

//=============================================================================
// 
/*
 * Initialisation Of MAX30003 - Configure Register Settings According To Data In max_reg_cfg Array
 * Parameters:  -
 * Return:      -
 */
 void max_init(void)
 {
    int i = 0;
    uint8_t regAddr, regData1, regData2, regData3;
    uint32_t regData;

    // Initialise I/O Pins:
    pinMode(MAX_MISO,     INPUT);
    pinMode(MAX_MOSI,     OUTPUT);
    pinMode(MAX_SCLK,     OUTPUT);
    pinMode(MAX_CSN,      OUTPUT);
    pinMode(MAX_INTB,     INPUT);

    // Take The Chip Select High To De-Select:
    digitalWrite(MAX_CSN, HIGH);

    // Start The SPI Library:
    SPIMAX.begin(MAX_SCLK,MAX_MISO,MAX_MOSI,MAX_CSN);
    SPIMAX.setBitOrder(MSBFIRST); 
    SPIMAX.setDataMode(SPI_MODE0);
  
    // Reset MAX30003
    max30003_sw_reset();
    delay(100);

    // Load Register Settings - Setup MAX30003
    while(1){
      // Read Register Address And Data From max_reg_cfg, Then Transfer Via SPI, Until END Of Array (0xFFFFFF) Is Reached = Every Register Has Been Set
      regAddr = (uint8_t)(max_reg_cfg[i]>>24);
      regData = (uint32_t)(max_reg_cfg[i]);

      MAX30003_Reg_Write(regAddr, regData);
      delay(100);
      i++;

      if(max_reg_cfg[i] == 0xFFFFFFFF){
        break;
      }
    }
 }
//=============================================================================
//=============================================================================
/*
 * Read From Any Register Of MAX30003
 * Parameters:  uint8_t Reg_address = Address Of Register To Read From
 * Return:      Pointer To uint32_t SPI_temp_32 Array That Stores Data Read From Register
 */
uint32_t MAX30003_Reg_Read(uint8_t Reg_address)
{
    int i;
    uint8_t SPI_TX_Buff;
    char SPI_temp_32b[4] = {0};                                   // For Normal Register Readout
    // char SPI_temp_Burst[100];                                     // For Burst Register Readout

    SPIMAX.beginTransaction(settingsMAX);
    // Take The Chip Select LOW To Select The Device:
    digitalWrite(MAX_CSN, LOW);

    // Combine READ Command With Address Of Read-Register:
    SPI_TX_Buff = (Reg_address<<1 ) | RREG;
    SPIMAX.transfer(SPI_TX_Buff); //Send Register Location + Command

    // Read 3 Byte Of Data From Register:
    for ( i = 0; i < 3; i++)
    {
       SPI_temp_32b[i] = SPIMAX.transfer(0xff);
    }
    
    // Take The Chip Select High To De-Select:
    digitalWrite(MAX_CSN, HIGH);
    SPIMAX.endTransaction();
  
    return *(uint32_t*) SPI_temp_32b;
}
//=============================================================================
//=============================================================================
/*
 * Write To Any Register Of MAX30003
 * Parameters:  unsigned char WRITE_ADDRESS = Address Of Register To Write To | unsigned long data = Data To Write To Register
 * Return:      -
 */
void MAX30003_Reg_Write (unsigned char WRITE_ADDRESS, unsigned long data)
{
    // Combine WRITE Command With Address Of Write-Register:
     byte dataToSend = (WRITE_ADDRESS<<1) | WREG;
     uint8_t regData1, regData2, regData3;

    // Split Data Into 3 Separate Bytes
     regData1 = (uint8_t)(data>>16);
     regData2 = (uint8_t)(data>>8);
     regData3 = (uint8_t)(data);

     SPIMAX.beginTransaction(settingsMAX);
     // Take The Chip Select LOW To Select The Device:
     digitalWrite(MAX_CSN, LOW);
     
     delay(2);
     SPIMAX.transfer(dataToSend);   //Send Register Location
     SPIMAX.transfer(regData1);     //Send First Byte Of Data
     SPIMAX.transfer(regData2);     //Send Second Byte Of Data
     SPIMAX.transfer(regData3);     //Send Third Byte Of Data
     delay(2);
     
     // Take The Chip Select High To De-Select:
     digitalWrite(MAX_CSN, HIGH);
     SPIMAX.endTransaction();
}
//=============================================================================
//=============================================================================
/*
 * Software Reset Of MAX30003
 * Parameters:  -
 * Return:      -
 */
void max30003_sw_reset(void)
{
    MAX30003_Reg_Write(SW_RST,0x000000);     
    delay(100);
}
//=============================================================================
//=============================================================================
/* 
 * In Addition To Resetting And Synchronizing The Operations Of Any Active ECG And RtoR Circuitry, SYNCH Will Also
 * Reset And Clear The FIFO Memories And The DSP Filters (To Midscale), Allowing The User To Effectively Set The 
 * “Time Zero” For The FIFO Records
 * Parameters:  -
 * Return:      -
 */
void max30003_synch(void)
{
    MAX30003_Reg_Write(SYNCH,0x000000);
}
//=============================================================================
//=============================================================================
/*
 * Reading RtoR Interval From FIFO RTOR And Estimating Heart Rate HR
 * Reads Out RtoR Data From FIFO, Calucates HR From That
 * Parameters:  -
 * Return:      byte hr = HR Data Calculated From Value Of RTOR Register Readout From MAX30003
 */
byte MAXreadoutHR(void)
{
  // Read First Two Bytes Of RTOR Register From MAX30003
  uint32_t rtor = MAX30003_Reg_Read(RTOR) & 0xFFFF;
  
  // Switch LSB And MSB Of MAX-Readout RTOR Data (Because MSBFIRST) To Get The Correct Bit Order
  rtor = rtor >> 8 | (rtor & 0xFF) << 8;
    
  // Shift RTOR Value, Due To Partition Of Register (See Data Sheet)
  rtor = ((rtor >> 2) & 0x3FFF);
  
  // Calculate HR From RTOR Value:
  return(byte)(60 / ((float)rtor*0.0078125)); // Timing Resolution = 0.0078125 s - Because CLK = 32.786 Hz
}
//=============================================================================
