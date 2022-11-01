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
 
#ifndef MAX30003_H_
#define MAX30003_H_

#include <Arduino.h>
#include <SPI.h>

/*
  ____________________________________________________________________________________________________
  |       Declaration               |            Functionality                                       |
  ____________________________________________________________________________________________________
*/
// SPI register read/write commands - MAX30003
#define WREG      0x00              // WRITE Command
#define RREG      0x01              // READ Command

// Register Definition MAX30003
#define   NO_OP             0x00    // read-write register that has no internal effect on the device
#define   STATUS            0x01    // STATUS is a read-only register that provides a comprehensive overview of the current status of the device
#define   EN_INT            0x02    // read/write register that governs the operation of the INTB output | can also be used to mask persistent interrupt conditions in order to perform other interrupt-driven operations
#define   EN_INT2           0x03    // read/write register that governs the operation of the INTB2 output | can also be used to mask persistent interrupt conditions in order to perform other interrupt-driven operations
#define   MNGR_INT          0x04    // read/write register that manages the operation of the configurable interrupt bits in response to ECG FIFO conditions
#define   MNGR_DYN          0x05    // read/write register that manages the settings of any general/dynamic modes within the device (ECG fast recovery mode, thresholds)
#define   SW_RST            0x08    // software reset
#define   SYNCH             0x09    // Synchronize is a write-only register/command that begins new ECG operations and recording, beginning on the internal MSTR clock edge following the end of the SPI SYNCH transaction
#define   FIFO_RST          0x0A    // FIFO reset
#define   INFO              0x0F    // read-only register that provides information about the MAX30003
#define   CNFG_GEN          0x10    // read/write register which governs general settings, most significantly the master clock rate for all internal timing operations
#define   CNFG_CAL          0x12    // read/write register that configures the operation, settings, and function of the Internal Calibration Voltage Sources
#define   CNFG_EMUX         0x14    // read/write register which configures the operation, settings, and functionality of the Input Multiplexer associated with the ECG channel
#define   CNFG_ECG          0x15    // read/write register which configures the operation, settings, and functionality of the ECG channel
#define   CNFG_RTOR1        0x1D    // algorithmic voltage gain + threshold parameters of two-part read/write register that configures the operation, settings, and function of the RTOR heart rate detection block
#define   CNFG_RTOR2        0x1E    // algorithmic timing parameters of two-part read/write register that configures the operation, settings, and function of the RTOR heart rate detection block
#define   ECG_FIFO_BURST    0x20    // ECG Sample Voltage Data
#define   ECG_FIFO          0x21    // ECG Sample Voltage Data
#define   RTOR              0x25    // RTOR Interval Timing Data
#define   NO_OP             0x7F    // read-write register that has no internal effect on the device

// Pins Used For Connection Between MCU (ESP32) And MAX30003
#define   MAX_MISO      12          // SPI Slave Out Master In
#define   MAX_MOSI      13          // SPI Slave In Master Out
#define   MAX_SCLK      14          // SPI Clock Pin
#define   MAX_CSN       15          // Active-Low SPI Chip-Select Input Pin | 0 LOW = Enable | 1 HIGH = Disable
#define   MAX_INTB      26          // Interrupt Output. INTB Is An Active LOW Status Output. It Can Be Used To Interrupt An External Device. 
//#define   MAX_INTB2     27          // Interrupt 2 Output. INT2B Is An Active LOW Status Output. It Can Be Used To Interrupt An External Device. 

void max_init(void);
uint32_t MAX30003_Reg_Read(uint8_t Reg_address);
void MAX30003_Reg_Write (unsigned char WRITE_ADDRESS, unsigned long data);
void max30003_sw_reset(void);
void max30003_synch(void);
byte MAXreadoutHR(void);

#endif
