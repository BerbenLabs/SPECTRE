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

#ifndef __SPI_FLASH_H_
#define __SPI_FLASH_H_

#include <Arduino.h>
#include <SPI.h>

/*  ____________________________________________________________________________________________________
  |       Declaration               |            Functionality                                       |
  ____________________________________________________________________________________________________
*/

// S70FL01GSAGMFI011 SPI Flash Status Register Bit Definition

#define STAT_WIP    (1<<0)      // Write in Prograss (Status REG)   - Device Busy - Write Registers (WRR), Program, Erase Or Other Operation Is In Progress
#define STAT_WEL    (1<<1)      // Write Enable Latch (Status REG)  - Device Accepts Write Registers (WRR), Program Or Erase Commands
#define STAT_E_ERR  (1<<6)      // Erase Error Occurred
#define STAT_P_ERR  (1<<7)      // Programming Error Occured
#define STAT_SRWD   (1<<8)      // Status Register Write Disable - Locks State Of SRWD, BP, And Configuration Register Bits When WP# Is Low By Ignoring WRR Command

// S70FL01GSAGMFI011 SPI Flash Register Definition
#define CMD_WRITE_STATUS_REG    0x01
#define CMD_PAGE_PROGRAM        0x02
#define CMD_READ_DATA           0x03
#define CMD_WRITE_DISABLE       0x04                  //Not Tested
#define CMD_READ_STATUS_REG     0x05
#define CMD_WRITE_ENABLE        0x06
#define CMD_READ_HIGH_SPEED     0x0B                  //Not Tested
#define CMD_SECTOR_ERASE        0x20                  //Not Tested
#define CMD_BLOCK32K_ERASE      0x52                  //Not Tested
#define CMD_RESET_DEVICE        0xF0                  //<<-different from winbond
#define CMD_READ_ID             0x9F
#define CMD_RELEASE_POWER_DOWN  0xAB                  //Not Tested
#define CMD_POWER_DOWN          0xB9                  //Not Tested
#define CMD_CHIP_ERASE          0xC7
#define CMD_BLOCK64K_ERASE      0xD8                  //Not Tested

#define BRRD                    0x16
#define BRWR                    0x17

// Pins Used For Connection Between MCU (ESP32) And S70FL01GSAGMFI011 SPI Flash
#define FLASH_CSN1              16                    // SPI Chip-Select. FL512S #1.  
#define FLASH_CSN2              17                    // SPI Chip-Select. FL512S #2.
#define FLASH_MISO              12                    // SPI Slave Out Master In
#define FLASH_MOSI              13                    // SPI Slave In Master Out
#define FLASH_SCLK              14                    // 1. SPI mode: SPI Clock Pin | 2. I²C mode: I²C Clock Pin

#define FLASH_PAGE_SIZE         256                   // In Byte
#define FLASH_PAGE_MAX          0x80000               // 128 MByte
#define FLASH_BANK_PAGE_MAX     0x40000               //  64 MByte

// Callback Function To Check If Device Is Still Erasing
typedef void (*cbDelete_t) ();

bool flash_init( cbDelete_t cb );
bool isPageEmpty(unsigned char * page, int length);
void write_pause(void);
int flash_page_program(unsigned char *wp, int pn);
int flash_read_pages(unsigned char *p, int pn, const int n_pages);
int flash_fast_read_pages(unsigned char *p, int pn, const int n_pages);
bool flash_chip_erase(boolean wait, int cb_delay);
unsigned char setFlashBAR(byte REG_Data);
void flash_read_id(unsigned char *idt);

#endif
