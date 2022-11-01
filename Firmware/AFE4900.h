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

#ifndef __AFE_4900_H_
#define __AFE_4900_H_

#include <Arduino.h>
#include <SPI.h>

//  ____________________________________________________________________________________________________
//  |       Declaration               |            Functionality                                       |
//  ____________________________________________________________________________________________________

#define REG_READ              0x00000001          // Register Address 0x00 (CONTROL0), bit 0 = SPI_REG_READ gesetzt = READ Register Command
#define REG_WRITE             0x00000000          // Register Address 0x00 (CONTROL0), bit 0 = SPI_REG_READ nicht gesetzt = WRITE Register Command

// Register Definition AFE4900
#define   CONTROL0            0x00                // AFE control settings
#define   LED2STC             0x01                // determines sample LED2 start
#define   LED2ENDC            0x02                // determines sample LED2 end
#define   LED1LEDSTC          0x03                // determines LED1 start
#define   LED1LEDENDC         0x04                // determines LED1 end
#define   ALED2STC            0x05                // determines sample Ambient2 (or determines sample LED3) start
#define   ALED2ENDC           0x06                // determines sample Ambient2 (or determines sample LED3) end
#define   LED1STC             0x07                // determines sample LED1 start
#define   LED1ENDC            0x08                // determines sample LED1 end
#define   LED2LEDSTC          0x09                // determines LED2 start
#define   LED2LEDENDC         0x0A                // determines LED2 end
#define   ALED1STC            0x0B                // determines sample Ambient1 start
#define   ALED1ENDC           0x0C                // determines sample Ambient1 end
#define   LED2CONVST          0x0D                // determines LED2 convert phase start
#define   LED2CONVEND         0x0E                // determines LED2 convert phase end
#define   ALED2CONVST         0x0F                // determines Ambient2 (or LED3) convert phase start
#define   ALED2CONVEND        0x10                // determines Ambient2 (or LED3) convert phase end
#define   LED1CONVST          0x11                // determines LED1 convert phase start
#define   LED1CONVEND         0x12                // determines LED1 convert phase end
#define   ALED1CONVST         0x13                // determines Ambient1 convert phase start
#define   ALED1CONVEND        0x14                // determines Ambient1 convert phase end
#define   PRPCOUNT            0x1D                // the count value for the counter that sets the PRF. The counter automatically counts until PRPCT and then return back to 0 to start the count for the next PRF cycle
#define   CONTROL1            0x1E                // enables the timing engine | determines the number of ADC averages
#define   TIAGAIN_2_3         0x1F                // TIA settings
#define   TIAGAIN             0x20                // TIA settings
#define   TIA_AMB_GAIN        0x21                // TIA settings
#define   LEDCNTRL1           0x22                // current control ILED of LED1, LED2, LED3
#define   CONTROL2            0x23                // AFE control setting
#define   LEDCNTRL2           0x24                // current control ILED of LED4
#define   TOGGLE              0x28                // toggles between 0 and 1 at every FIFO_RDY. The FIFO_TOGGLE register bit can also be brought out on the ADC_RDY pin.
#define   CLKDIV1             0x29                // tri-states the SDOUT buffer
#define   LED2VAL             0x2A                // these bits are the LED2 output code in 24-bit, two's complement format.
#define   ALED2VAL            0x2B                // these bits are the Ambient2 or LED3 output code in 24-bit, two's complement format.
#define   LED1VAL             0x2C                // these bits are the LED1 output code in 24-bit, two's complement format.
#define   ECGVAL              0x2D                // these bits are the Ambient1 output code in 24-bit, two's complement format. When the ECG signal chain is enabled (ENABLE_PTT = 1), ALED1VAL denotes the output code corresponding to the ECG signal.
#define   LED2-ALED2VAL       0x2E                // these bits are the (LED2–Ambient2) output code in 24-bit, two's complement format.
#define   LED1-ALED1VAL       0x2F                // these bits are the (LED1–Ambient1) output code in 24-bit, two's complement format.
#define   READ_CONTROL1       0x30                // bit 19: read out state of the CONTROL1 pin | 0 = LDO Enable | 1 = LDO Bypass
#define   CONTROL3            0x31                // bandwidth setting of the noise-reduction filter | disconnects the PD signals (INP, INM) from the TIA inputs. When enabled, the current input to the TIA is determined completely by the offset cancellation DAC current (I_OFFDAC)
#define   PROG_INT2_STC       0x34                // determines spare interrupt 2 start
#define   PROG_INT2_ENDC      0x35                // determines spare interrupt 2 end
#define   LED3LEDSTC          0x36                // determines LED3 start
#define   LED3LEDENDC         0x37                // determines LED3 end
#define   CLKDIV2             0x39                // clock division ratio for the clock divider in the path of the input clock input to the timing engine.
#define   OFFDAC              0x3A                // offset cancellation DAC settings
#define   THRDETLOW           0x3B                // 24-bit code in two's complement format used for setting the low threshold when checking the output code in threshold detect mode
#define   THRDETHIGH          0x3C                // 24-bit code in two's complement format used for setting the high threshold when checking the output code in threshold detect mode
#define   THRDET              0x3D                // threshold detect mode settings
#define   O_OFFDAC            0x3E                // offset cancellation DAC settings
#define   AVG_LED2_ALED2VAL   0x3F                // these bits are the 24-bit averaged output code for (LED2–Ambient2) when decimation mode is enabled. The averaging is done over the number of samples specified by the decimation factor.
#define   AVG_LED1_ALED1VAL   0x40                // these bits are the 24-bit averaged output code for (LED1–Ambient1) when decimation mode is enabled. The averaging is done over the number of samples specified by the decimation factor
#define   FIFO                0x42                // FIFO + interrupt multiplexing settings
#define   LED4LEDSTC          0x43                // determines LED4 start
#define   LED4LEDENDC         0x44                // determines LED4 end
#define   TG_PD1STC           0x45                // determines the TG_PD1 start count. Must be defined if DUAL_PD_ENABLE is set to 1
#define   TG_PD1ENDC          0x46                // determines the TG_PD1 end count. Must be defined if DUAL_PD_ENABLE is set to 1
#define   TG_PD2STC           0x47                // determines the TG_PD2 start count. Must be defined if DUAL_PD_ENABLE is set to 1
#define   TG_PD2ENDC          0x48                // determines the TG_PD2 end count. Must be defined if DUAL_PD_ENABLE is set to 1
#define   TG_PD3STC           0x49                // determines the TG_PD3 start count. Must be defined if PD3 is used
#define   TG_PD3ENDC          0x4A                // determines the TG_PD3 end count. Must be defined if PD3 is used
#define   CONTROL4            0x4B                // AFE control settings
#define   DUAL_PD             0x4E                // lead-off detect settings
#define   CONTROL5            0x50                // AFE control settings
#define   FIFO_OFFSET         0x51                // FIFO offset settings
#define   DATA_RDY_STC        0x52                // determines the start count for DATA_RDY, FIFO_RDY, and THR_DET_EN
#define   DATA_RDY_ENDC       0x53                // determines the end count for DATA_RDY, FIFO_RDY, and THR_DET_EN
#define   MASK_PPG            0x54                // determines the sampling rate (relative to the PRF) for the PPG signal acquisition
#define   PROG_INT1_STC       0x57                // determines spare interrupt 1 start
#define   PROG_INT1_ENDC      0x58                // determines spare interrupt 1 end
#define   ECG_CHOP            0x61                // bit 19: 0 = chopping of ECG signal chain is disabled | 1 = chopping of ECG signal chain enabled
#define   ECG_RLD             0x62                // enable RLD | enable lead-off detect
#define   RCOMP               0x63                // output of high-side comparator in dc lead-off detect mode
#define   DYN_TIA_STC         0x64                // determines the start of the active phase of the TIA coinciding with the start of the device active phase
#define   DYN_TIA_ENDC        0x65                // determines the end of the TIA active phase coinciding with the end of the device active phase
#define   DYN_ADC_STC         0x66                // determines the start of the ADC active phase coinciding with the start of the device active phase
#define   DYN_ADC_ENDC        0x67                // determines the end of the ADC active phase coinciding with the end of the device active phase
#define   DYN_CLOCK_STC       0x68                // determines the start of the active phase of the 4-MHz oscillator used for ADC conversion. Coincides with the start of the device active phase
#define   DYN_CLOCK_ENDC      0x69                // determines the end of the active phase of the 4-MHz oscillator used for ADC conversion. Coincides with the end of the device active phase
#define   DEEP_SLEEP_STC      0x6A                // determines the start of deep sleep phase
#define   DEEP_SLEEP_ENDC     0x6B                // determines the end of deep sleep phase
#define   PD_SHORT            0x6C                // short PD1 input pins to VCM (internal common mode voltage) when PD1 is disconnected from the active TIA.
#define   REG_POINTER         0x6D                // The instantaneous value of the (write pointer minus read pointer) minus 1 is stored in REG_POINTER_DIFF. When a FIFO_RDY interrupt is issued, the value stored in REG_POINTER_DIFF is expected to be equal to the value programmed in REG_WM_FIFO
#define   LED_DRIVER_CONTROL  0x72                // enables/disables driver controls for LEDs
#define   THR_DETECT_LOGIC    0x73                // determines whether the THR_DET_RDY interrupt is generated by the output code going in-range or out-of-range

// Pins Used For Connection Between MCU (ESP32) And AFE4900
#define   AFE_MISO      19                        // SPI Slave Out Master In
#define   AFE_MOSI      23                        // SPI Slave In Master Out
#define   AFE_SCLK      18                        // 1. SPI mode: SPI Clock Pin | 2. I²C Mode: I²C Clock Pin
#define   AFE_CSN       5                         // 1. SPI mode: Chip-Select Pin For SPI (Active Low) | 2. I²C Mode: Inverts The LSB Of The I2C Slave Address.
#define   I2C_SPI_SEL   22                        // 0 LOW = I²C Interface Enabled | 1 HIGH = SPI Interface Enabled 
#define   RESETZ        2                         // Either RESETZ Or PWDN. Functionality Is Based On The (Active Low) Duration Of RESETZ.
//...................................................RESET = 25 us - 50 us pulse | PWDN = > 200 us Pulled Down (Hardware PWDN Mode)
#define   AFE_ADC_RDY   25                        // Programmable Interrupt (Output Of AFE4900)
#define   PROG_OUT1     21                        // Programmable Interrupt (Output) - To Enable The PROG_OUT1 Pin As An Output Pin, Set The EN_PROG_OUT1 Bit To 1.

void afe_init(void*);
int32_t AFE_Reg_Read(uint8_t Reg_address);
void AFE_Reg_Write (unsigned char WRITE_ADDRESS, unsigned long data);
void AFE_Read_FIFO(int32_t* RED, int32_t* IR, int32_t* GREEN, int32_t* ECG);

#endif
