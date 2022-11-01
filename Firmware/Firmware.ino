/* 
 * Subject: Firmware
 * Project: SPECTRE[3.0]
 *
 * This Firmware Runs The Measuring Unit SPECTRE [3.0], Which Communicates With The Control Unit STAVRO Over WiFi (TCP And MQTT)
 * It Can be Updated Via ArduinoOTA
 */

/* Hardware specifications and additional informations
    ---------------------------------------------------------------------------------------------------------------------------------------

  Firmware:
    AFE4900:                      ECG + PPG data capture and transfer to ESP32 via SPI
    MAX30003:                     Calculating Heart Rate from ECG data, transfer to ESP32 via SPI
    S70FL01GSAGMFI011 SPI Flash:  Saving Data (ECG + PPG) transferred from ESP32 via SPI
    ESP32 WROOM:                  Reading/Writing data from/to ICs via SPI, transferring/receiving data via WiFi (MQTT)

    ---------------------------------------------------------------------------------------------------------------------------------------
    ---------------------------------------------------------------------------------------------------------------------------------------
  Uses the ESP32 WiFi Library.
  Uses the SPI library.
  Uses the async-mqtt-client Library.   https://github.com/marvinroger/async-mqtt-client
  Uses the ArduinoOTA Library.          https://lastminuteengineers.com/esp32-ota-updates-arduino-ide/

  For details on the AFE4900, see:
  http://www.ti.com/lit/ds/symlink/afe4900.pdf  |  http://www.ti.com/product/AFE4900

  For details on the MAX30003, see:
  https://datasheets.maximintegrated.com/en/ds/MAX30003.pdf  |  https://www.maximintegrated.com/en/products/analog/data-converters/analog-front-end-ics/MAX30003.html
  https://github.com/Protocentral/protocentral_max30003

  For details on the S70FL01GSAGMFI011 SPI Flash, see:
  https://www.cypress.com/file/233721/download  |  https://www.cypress.com/documentation/datasheets/s70fl01gs-1-gbit-128-mbyte-30v-spi-flash-datasheet
  https://github.com/kriswiner/SPIFlash

  For details on the ESP32 WROOM, see:
  https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf  |  https://www.espressif.com/en/products/hardware/esp-wroom-32/overview
  https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf

  For details on MQTT, see:
  https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/ | https://randomnerdtutorials.com/what-is-mqtt-and-how-it-works/
  Functions copied and modified from: https://github.com/marvinroger/async-mqtt-client/blob/master/examples/FullyFeatured-ESP32/FullyFeatured-ESP32.ino
  QoS #:
  0 = At most once
  1 = At least once
  2 = Exactly once

    ---------------------------------------------------------------------------------------------------------------------------------------
    ---------------------------------------------------------------------------------------------------------------------------------------

  Hardware - Circuit ICs / ESP32:
  ______________________________________________________________________
  | Net_Label     |   IC Pin    | ESP32 WROOM Pin + Description | shares Pin with... |
  ______________________________________________________________________

  AFE4900 Breakout Board attached to pins 2, 5, 18, 19, 21 - 23, 33, 34:
    AFE_MISO:       SDOUT           IO19    VSPIQ
    AFE_MOSI:       I2C_DAT         IO23    VSPID
    AFE_SCLK:       I2C_CLK         IO18    VSPICLK
    AFE_CSN:        SEN             IO5     VSPICS0
    I2C_SPI_SEL:    I2C_SPI_SEL     IO22    GPIO
    RESETZ:         RESETZ          IO2     GPIO
    AFE_PROG_OUT1:  PROG_OUT1       IO21    GPIO
    AFE_ADC_RDY:    ADC_RDY         IO34    GPIO
    XTAL_32:        CLK             IO33    XTAL_32K_N              - MAX XTAL_32
                                            (32.768 kHz PWM output)

  MAX30003 attached to pins 12 - 15, 26, 27, 33:
    MAX_MISO:       SDO             IO12    HSPIQ                   - FLASH_MISO
    MAX_MOSI:       SDI             IO13    HSPID                   - FLASH_MOSI
    MAX_SCLK:       SCLK            IO14    HSPICLK                 - FLASH_SCLK
    MAX_CSN:        CSB             IO15    HSPICS0
    MAX_INTB:       INTB            IO26    GPIO
    MAX_INTB2:      INT2B           IO27    GPIO
    XTAL_32:        FCLK            IO33    XTAL_32K_N              - AFE XTAL_32
                                            (32.768 kHz PWM output)

  S70FL01GSAGMFI011 SPI Flash attached to pins 12 - 14, 16, 17:
    FLASH_MISO:     SO/IO2          IO12    VSPIQ                   - MAX_MISO
    FLASH_MOSI:     SI/IO1          IO13    VSPID                   - MAX_MOSI
    FLASH_SCLK:     SCK             IO14    VSPICLK                 - MAX_SCLK
    FLASH_CSN1:     CS1#            IO16    GPIO
    FLASH_CSN2:     CS2#            IO17    GPIO

      ---------------------------------------------------------------------------------------------------------------------------------------
      ---------------------------------------------------------------------------------------------------------------------------------------

  LED indications:
    LED_RED attached to Pin 27
    LED_GREEN attached to Pin 32
      (both powered by VCC (3.3 V) - switched against GND by means of switching ESP Output Pins to LOW)

    NO CONNECTION TO MQTT           = LED RED is on,  LED GREEN is off
    CONNECTION TO MQTT ESTABLISHED  = LED RED is off, LED GREEN is on
    ANY HARDWARE ERROR              = LED RED is on,  LED GREEN is on
    FLASH DELETING                  = LED RED blinking periodically with T = 2*CB_CYCLE_DEL_DELAY
    RECORDING AFE4900 DATA          = LED RED blinking periodically with T ~ 1 s
    SENDING AFE4900 DATA TO HOST    = LED RED blinking periodically with T ~ 400 ms
*/

// ----------------------------------------------------------------------------------------------------------------------------------------//
// Core Pinner
#define SPECTRE_HEARTBEAT_CORE 0
#define SPECTRE_MEASUREMENT_CORE 1
#define CONFIG_ASYNC_TCP_RUNNING_CORE SPECTRE_HEARTBEAT_CORE

// ----------------------------------------------------------------------------------------------------------------------------------------//
// WIFI
#define WIFI_SSID           "SPECTRE-AP"
#define WIFI_PASSWORD       "_stavro_007"
#define HOST_IP IPAddress(192, 168, 1, 40)

#define HOST_PORT 8088
#define MQTT_PORT 1883

// ----------------------------------------------------------------------------------------------------------------------------------------//
// OTA
#define OTA_PASSWORD "SPECTRE007"

// ----------------------------------------------------------------------------------------------------------------------------------------//
// ESP32 Pins
#define LED_RED 27
#define LED_GREEN 32
#define XTAL 33  // XTAL_32K_N (32.768 kHz crystal oscillator output)

// ----------------------------------------------------------------------------------------------------------------------------------------//
// MQTT specifications
// QoS (quality of service) for MQTT communication
// 0 = at most once | 1 = at least once | 2 = exactly once
#define MQTT_QoS_SUB 1  // QoS for subscriptions
#define MQTT_QoS_PUB 1  // QoS for publishing

#define TOPIC_REC "SPECTRE/RECORDING"    // Topic for commands on recording start/stop
#define TOPIC_O2 "SPECTRE/O2"            // Topic for setting O₂ markers - O₂ on/off
#define TOPIC_DEL "SPECTRE/DELETE"       // Topic for deleting
#define TOPIC_GETDATA "SPECTRE/GETDATA"  // Topic for deleting
#define TOPIC_STAT "SPECTRE/STATUS/"     // Topic for status
#define TOPIC_HR "SPECTRE/HR/"           // Topic for MAX30003 Heart Rate
#define TOPIC_TIME "SPECTRE/TIME/"       // Topic for time
#define TOPIC_RSSI "SPECTRE/RSSI/"       // Topic for WiFi.RSSI (WiFi connection strength)

#define FLAG_O2 (1 << 0)             // command: set FLAG_O2_DATA_MARKER
#define FLAG_RECORDING (1 << 1)      // command: start measurement
#define FLAG_DELETING (1 << 2)       // command: delete data from flash
#define FLAG_SEND_DATA (1 << 3)      // command: send data to host
#define FLAG_ABORT_MEASURE (1 << 4)  // status: measure aborted due to memory overflow
#define FLAG_READ_AFE_FIFO (1 << 5)  // interrupt: read data from FIFO of AFE4900

#define FLAG_O2_DATA_MARKER 0x80000000

#define CLEARBITMASK(input, mask) input &= ~mask
#define SETBITMASK(input, mask) input |= mask

// ----------------------------------------------------------------------------------------------------------------------------------------//
// LED toggle pulse duration
#define CB_CYCLE_DEL_DELAY 1500  // cycle delay for function callback called by flash_chip_erase() --> T = 3 Hz
#define REC_LED_TGL_FAC 31       // 256 bytes/16 bytes = 16 --> ~ 16 ms per page --> 500 ms/ 16 ms ~ 31 --> T ~ 1 Hz

// -------------------------------------------------
// Misc
#define ON HIGH
#define OFF LOW

#define FLASH_PAGE_BUFFER_SIZE 256  // Always Use A Multiple Of 4, Because Then Switch To 2nd Flash Bank While Readout Not Clean = Missing Values !!!

// ----------------------------------------------------------------------------------------------------------------------------------------//

#include <WiFi.h>
#include <SPI.h>
#include "AsyncMqttClient.h"

#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#include "AFE4900.h"
#include "MAX30003.h"
#include "SPIFLASH.h"

#include "esp_task_wdt.h"
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

// ----------------------------------------------------------------------------------------------------------------------------------------//
// -------- DECLARATIONS ------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------------------------------------------------------------------------------------------//

/*____________________________________________________________________________________________________
  |       Declaration               |            Functionality                                       |
  ____________________________________________________________________________________________________
*/
TaskHandle_t HeartBeatTask;  // TaskHandle_t For WiFi Communication Task
TaskHandle_t Measuretask;    // TaskHandle_t For Data Acquisition Task
TimerHandle_t MQTT_ReconnectTimer;
SemaphoreHandle_t xSemaphore;  // Semaphore Handle For Starting Measurement

char UniqueClientID[21];
char UniqueClientID_OTA[20];

WiFiClient WifiClient;
AsyncMqttClient MQTT_Client;

String MAC = "";
String IP_device = "";

volatile char CMD_FLAG = 0x00;  // CMD_FLAG To Determine And React To Pi Requests |

// PWM On Pin XTAL, Configurations
const int xtal_freq = 31988;  // Tested 07.01.2019: 31988 = 125 kHz | PWM Frequency For MAX Readout - Sampletime: 1000 (sometimes 995) with f = 31548 | 962 with f= 32768 | 993 with f = 31768 | 1003 with f = 31468
const int ledChannel = 0;
const int resolution = 8;   // PWM Bit Resolution | 8 Bit = 2^8 = 256
const int dutycycle = 127;  // PWM Duty Cycle Determination |  0 % = 0 | 25% = 64| 50% = 127 | 75% = 191 | 100% = 255 - If 8 Bit Resolution
// ----------------------------------------------------------------------------------------------------------------------------------------//
// Variables For Operating MAX30003
byte HR;

// ----------------------------------------------------------------------------------------------------------------------------------------//
// Variables For Operating SPI Flash
int Flash_Page_Number = 0;
unsigned char Flash_Page_Buffer[FLASH_PAGE_SIZE * FLASH_PAGE_BUFFER_SIZE];

// ----------------------------------------------------------------------------------------------------------------------------------------//
// ---------------- FUNCTIONS -------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------------------------------------------------------------------------------------------//
inline void SET_LED(int PIN, bool ON_OFF) {
  // Determines If LED Is ON Or OFF According To Function Input Parameters
  pinMode(PIN, ON_OFF ? OUTPUT : INPUT);
  digitalWrite(PIN, LOW);
}
// ----------------------------------------------------------------------------------------------------------------------------------------//
void cb_flash_delete() {
  // Callback Function That Toggles LED_RED While Flash Is Still Erasing
  static bool led_tgl = false;

  SET_LED(LED_RED, led_tgl = !led_tgl);
}
// ----------------------------------------------------------------------------------------------------------------------------------------//
void IRAM_ATTR ISR() {
  // ISR That Sets CMD_FLAG Bit If AFE4900 Send FIFO Ready Interrupt On AFE_ADC_RDY Pin
  SETBITMASK(CMD_FLAG, FLAG_READ_AFE_FIFO);
}
// ----------------------------------------------------------------------------------------------------------------------------------------//
// -------- SETUP -------------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------------------------------------------------------------------------------------------//

void setup() {
  Serial.begin(115200);

  pinMode(XTAL, OUTPUT);

  // Turn On LED_RED To Indicate Missing MQTT Connection On Setup
  SET_LED(LED_RED, ON);

  // Setup Timing PWM For ICs AFE4900 And MAX30003
  ledcSetup(ledChannel, xtal_freq, resolution);
  ledcAttachPin(XTAL, ledChannel);
  ledcWrite(ledChannel, dutycycle);

  // ----------------------------------------------------------------------------------------------------------------------------------------//
  // Initialise ICs
  afe_init((void*)ISR);
  delay(100);

  if (!flash_init(cb_flash_delete)) {
    // LED_RED Already ON, Indicate Hardware Error
    SET_LED(LED_GREEN, ON);
    while (1)
      ;
  }
  delay(100);

  max_init();
  delay(100);

  // ----------------------------------------------------------------------------------------------------------------------------------------//
  // Get CPU Task Handle From Both ESP32 Cores, Disable Watchdog And Check If Successful
  TaskHandle_t idle_0 = xTaskGetIdleTaskHandleForCPU(0);
  if (idle_0 == NULL || esp_task_wdt_delete(idle_0) != ESP_OK) {
    // LED_RED Already ON, Indicate Hardware Error
    SET_LED(LED_GREEN, ON);
    while (1)
      ;
  }

  // ----------------------------------------------------------------------------------------------------------------------------------------//
  // Creates A Semaphore To Start Recording, Sempahore Is Available From The Start
  vSemaphoreCreateBinary(xSemaphore);
  // If Semaphore Could Be Created, Obtain The Semaphore To Wait With Measuring Until Start Command From User
  if (xSemaphore == NULL) {
    // LED_RED Already ON, Indicate Hardware Error
    SET_LED(LED_GREEN, ON);
    while (1)
      ;
  }
  xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);
  // ----------------------------------------------------------------------------------------------------------------------------------------//
  // Task For: MAX30003 Readout, HR And Device State Info Transmission To Pi
  xTaskCreatePinnedToCore(
    StartHeartbeatTask,       // Task Function
    "HeartBeatTask",          // Name Of Task
    10000,                    // Stack Size Of Task
    NULL,                     // Parameter Of The Task
    1,                        // Priority Of The Task
    &HeartBeatTask,           // Task Handle To Keep Track Of Created Task
    SPECTRE_HEARTBEAT_CORE);  // Pin Task To ESP32 Core 0 [Kernel and WiFi/TCP Tasks Also Operate On ESP32 Core 0]

  // Task For: AFE4900 Readout, Writing Data To Flash
  xTaskCreatePinnedToCore(
    StartMeasureTask,           // Task Function
    "Measuretask",              // Name Of Task
    10000,                      // Stack Size Of Task
    NULL,                       // Parameter Of The Task
    -1,                         // Priority Of The Task
    &Measuretask,               // Task Handle To Keep Track Of Created Task
    SPECTRE_MEASUREMENT_CORE);  // Pin Task To ESP32 Core 1 [void loop() Also Operates On ESP32 Core 1]

  // ----------------------------------------------------------------------------------------------------------------------------------------//
  // Initialise MQTT And WiFi Connection
  MQTT_ReconnectTimer = xTimerCreate("MQTT_Timer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(MQTT_Connect));

  WiFi.onEvent(WiFiEvent);

  uint8_t MAC[6];
  esp_read_mac(MAC, ESP_MAC_WIFI_STA);

  //Create ClientID By The Name Of SPECTRE[MACaddress], And SPECTRE-MACaddress For OTA
  sprintf(UniqueClientID, "SPECTRE[%02X%02X%02X%02X%02X%02X]", MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]);
  sprintf(UniqueClientID_OTA, "SPECTRE-%02X%02X%02X%02X%02X%02X", MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]);

  MQTT_Client.setClientId(UniqueClientID);
  MQTT_Client.setKeepAlive(10800);  // Keep Alive For Three Hours

  MQTT_Client.onConnect(onMqttConnect);
  MQTT_Client.onDisconnect(onMqttDisconnect);

  MQTT_Client.onMessage(onMqttMessage);
  MQTT_Client.setServer(HOST_IP, MQTT_PORT);

  WiFi_Connect();

  // ----------------------------------------------------------------------------------------------------------------------------------------//
  // For OTA Programming
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: If Updating SPIFFS This Would Be The Place To Unmount SPIFFS Using SPIFFS.end()
    })
    .onEnd([]() {
      
      Serial.println("OTA End");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      
      Serial.println("OTA In Porgress");
    })
    .onError([](ota_error_t error) {
      Serial.println("OTA ERROR");
    });

  //Define OTA Device Identifier As SPECTRE-MACaddress
  ArduinoOTA.setHostname(UniqueClientID_OTA);

  //Define Password To Block Unauhorised Access
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.begin();
}

// ----------------------------------------------------------------------------------------------------------------------------------------//
// -------- LOOP --------------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------------------------------------------------------------------------------------------//
void loop() {
  bool led_tgl;

  ArduinoOTA.handle();

  // -------------------------------------------------------------------
  /*
   * Deletes Content Of Flash Memory Storage - Deletes Every Page Of The Memory
   * Throws Error If Error Occured During Erase (Checks Respective Flash Status Register Bit)
   * LED Is Toggled Every 1.5 s
   */
  if (CMD_FLAG & FLAG_DELETING) {
    // Do If Erasing Command Was Sent By Host
    if (flash_chip_erase(true, CB_CYCLE_DEL_DELAY)) {
      // If No Error Occured While Erasing Flash
      // Clear Flags: "Deleting" And "Measure Aborted Due To Memory Overflow"
      CLEARBITMASK(CMD_FLAG, FLAG_DELETING | FLAG_ABORT_MEASURE);
      SET_LED(LED_RED, OFF);
    } else {
      // Hardware Error Indication
      SET_LED(LED_RED, ON);
      SET_LED(LED_GREEN, ON);
      while (1)
        ;
    }
  }
  // -------------------------------------------------------------------
  /*
   * Reads Out 256 Flash Pages In Fast Read Mode, Checks Those Pages For Last Entry (0xFF),
   * Transmits Number Of Pages That Contain Data (= Not Empty) To Host Via TCP,
   * Toggles LED_RED After Each Data Transmission Burst
   */
  else if (CMD_FLAG & FLAG_SEND_DATA) {
    // Do If Data Transfer Command Was Send By Host
    // Check WiFi Connection, Terminate Operation If Connections Is Lost
    if (!WifiClient.connect(HOST_IP, HOST_PORT)) {
      SET_LED(LED_GREEN, OFF);
      return;
    }
    SET_LED(LED_GREEN, ON);

    // Read Data From Flash Memory, Write To Flash_Page_Buffer Until Flash_Page_Buffer = 0xFF
    for (int page_number_r = 0; page_number_r < FLASH_PAGE_MAX; page_number_r += FLASH_PAGE_BUFFER_SIZE) {
      int bytes_read = flash_fast_read_pages(Flash_Page_Buffer, page_number_r, FLASH_PAGE_BUFFER_SIZE);
      int MaxValidPageNum = 0;
      for (MaxValidPageNum = 0; MaxValidPageNum < FLASH_PAGE_BUFFER_SIZE; MaxValidPageNum++) {
        if (isPageEmpty(&Flash_Page_Buffer[FLASH_PAGE_SIZE * MaxValidPageNum], FLASH_PAGE_SIZE)) {
          break;
        }
      }

      if (MaxValidPageNum <= 0)
        break;

      // Send Data Via TCP To Host
      WifiClient.write(Flash_Page_Buffer, FLASH_PAGE_SIZE * MaxValidPageNum);

      SET_LED(LED_RED, led_tgl = !led_tgl);
    }

    CLEARBITMASK(CMD_FLAG, FLAG_SEND_DATA);
    SET_LED(LED_RED, OFF);
  }
}

// ----------------------------------------------------------------------------------------------------------------------------------------//
// -------- TASKS -------------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------------------------------------------------------------------------------------------//

//=============================================================================
// Starts Data Acquisition Depending On Availability Of Semaphore
// Semaphore Is Made Available If User Chooses To Start Measuring Via WiFi Command
// During Measurement LED_RED Blinks With Frequency Of 1 Hz To Indicate Heartbeat Of Device
// Data Is Readout From AFE4900 FIFO (2x 128 Byte) And Written To Flash Every Second Interrupt
void StartMeasureTask(void* pvParameters) {
  int bytes_written = 0;
  bool led_tgl;
  bool tgl_lowhigh;  // Interrupt Counter

  for (;;) {
    // Wait Until The Semaphore Can Be Obtained, Then Start Task
    xSemaphoreTake(xSemaphore, (TickType_t)portMAX_DELAY);

    Flash_Page_Number = 0;
    led_tgl = false;
    unsigned long StartTime = millis();

    // Clear ISR Flag In Case It Was Set By An Interrupt Before Measure Was Started
    CLEARBITMASK(CMD_FLAG, FLAG_READ_AFE_FIFO);
    // Set Interrupt Counter To:
    tgl_lowhigh = false;

    while (CMD_FLAG & FLAG_RECORDING) {
      if (CMD_FLAG & FLAG_READ_AFE_FIFO) {
        // If Interrupt Pin Of AFE4900 Is Triggered, i.e. FIFO Is Ready For Readout
        // Clear ISR Flag To Indicate It's Usage And Prepare It For Next Cycle
        CLEARBITMASK(CMD_FLAG, FLAG_READ_AFE_FIFO);

        //for (int i = tgl_lowhigh ? 32 : 0; i < ((FLASH_PAGE_SIZE / 4) >> (tgl_lowhigh ? 0 : 1) ); i += 4)
        for (int i = 0; i < (FLASH_PAGE_SIZE / 4); i += 4) {
          /* Start With tgl_lowhigh = false, If False Set i =0 (Start First Readout), If True Set i = 32 (Start Second Readout)
           * Let Loop Run Until i Is Less Than 32 ((FLASH_PAGE_SIZE / 4) >> 0) If tgl_lowhigh = false (Start i = 0) - Estimates To 8 Cycles = 128 Byte
           * Or Until i Is Less Than 64 ((FLASH_PAGE_SIZE / 4) >> 1) If tgl_lowhigh = true (Start i = 32)
           *
           *('>> 1' Means Decimation By 2 (Bitshift))
           */
          AFE_Read_FIFO(
            &(((int32_t*)Flash_Page_Buffer)[i + 0]),
            &(((int32_t*)Flash_Page_Buffer)[i + 1]),
            &(((int32_t*)Flash_Page_Buffer)[i + 2]),
            &(((int32_t*)Flash_Page_Buffer)[i + 3]));

          // Check If Host Set O₂ Marker Command To On, If So Set O₂ Marker Bit To 1 In ECG Data, Otherwise To 0
          ((int32_t*)Flash_Page_Buffer)[i + 3] |= (CMD_FLAG & FLAG_O2) ? FLAG_O2_DATA_MARKER : 0;
        }

        // Toggle tgl_lowhigh, From false To true And Back. If false Write To Flash (After 2x 128 Byte = 256 Byte (Page Size Of Flash))
        // Write Data To Flash Memory, Initially Starting On Flash_Page_Number = 0
        bytes_written = flash_page_program(Flash_Page_Buffer, Flash_Page_Number);

        // If Writing Data Exceeds Maximum Flash Size, Set An Error Flag (FLAG_ABORT_MEASURE) To Indicate And Stop Recording
        if (++Flash_Page_Number >= FLASH_PAGE_MAX) {
          CLEARBITMASK(CMD_FLAG, FLAG_RECORDING);
          SETBITMASK(CMD_FLAG, FLAG_ABORT_MEASURE);
        }

        // Heartbeat - Toggle LED_RED Every 16 ms*REC_LED_TGL_FAC ~ 500 ms (If REC_LED_TGL_FAC = 31) (16 Values Per Page (16*16 bytes= 256 bytes), One Page Duration ~ 16 ms))
        if (!(Flash_Page_Number % REC_LED_TGL_FAC))
          SET_LED(LED_RED, led_tgl = !led_tgl);
      }
    }

    // Publish Time Topic On MQTT To See Elapsed Recording Time
    MQTT_Client.publish(((String)TOPIC_TIME + MAC + "/" + IP_device).c_str(), MQTT_QoS_PUB, true, String((unsigned long)(millis() - StartTime)).c_str());

    SET_LED(LED_RED, OFF);
  }
}

//=============================================================================
// Reads Out MAC Aaddress And IP Address From ESP32 And HR Data From MAX30003
// Publishes Device Status And HR With MAC And IP On MQTT Topics Every Second
void StartHeartbeatTask(void* pvParameters) {
  TickType_t yLastWakeTime;
  const TickType_t yFrequency = 1000;  // Determines Task Frequency - Do Task Every x Milliseconds

  // The Task Suspends Itself And Will Not Run Until Another Task Calls vTaskResume( HeartBeatTask )
  vTaskSuspend(NULL);

  MAC = String(WiFi.macAddress());
  IPAddress CurrentIP = WiFi.localIP();
  for (int i = 0; i < 4; i++) {
    IP_device += i ? "." + String(CurrentIP[i]) : String(CurrentIP[i]);
  }

  yLastWakeTime = xTaskGetTickCount();

  while (1) {
    HR = MAXreadoutHR();

    // Publish WiFi Connection Strength (RSSI), Heart Rate And Status Flag On MQTT Topics
    MQTT_Client.publish(((String)TOPIC_RSSI + MAC + "/" + IP_device).c_str(), MQTT_QoS_PUB, true, String((int)WiFi.RSSI()).c_str());
    MQTT_Client.publish(((String)TOPIC_STAT + MAC + "/" + IP_device).c_str(), MQTT_QoS_PUB, true, String((int)CMD_FLAG).c_str());
    MQTT_Client.publish(((String)TOPIC_HR + MAC + "/" + IP_device).c_str(), MQTT_QoS_PUB, true, String(HR).c_str());
    // Delay Task Until 1 Second Has Passed
    vTaskDelayUntil(&yLastWakeTime, yFrequency);
  }
}

//=========================================================================================================================================//
// -------- FUNCTIONS ---------------------------------------------------------------------------------------------------------------------//
//=========================================================================================================================================//

// ----------------------------------------------------------------------------------------------------------------------------------------//
// WIFI AND MQTT FUNCTIONS
// ----------------------------------------------------------------------------------------------------------------------------------------//

//=============================================================================
// Connects Device To WiFi
void WiFi_Connect() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
//=============================================================================
// Connects Device To MQTT Broker
void MQTT_Connect() {
  MQTT_Client.connect();
}
//=============================================================================
// Determines What Happens If WiFi Is Connected Or Disconnected
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      // If Connection Could Be Established And IP Was Assigned
      MQTT_Connect();
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      // If Device Is Disconnected From WiFi
      // Ensure We Don't Reconnect To MQTT While Reconnecting To Wi-Fi
      xTimerStop(MQTT_ReconnectTimer, 0);
      break;
  }
}

//=============================================================================
// Publishes Messages To MQTT Broker
void onMqttConnect(bool sessionPresent) {
  MQTT_Client.subscribe(TOPIC_REC, MQTT_QoS_SUB);

  MQTT_Client.subscribe(TOPIC_O2, MQTT_QoS_SUB);

  MQTT_Client.subscribe(TOPIC_GETDATA, MQTT_QoS_SUB);

  MQTT_Client.subscribe(TOPIC_DEL, MQTT_QoS_SUB);

  // Turn Off LED_RED And Turn On LED_GREEN To Indicate Established MQTT Connection
  SET_LED(LED_RED, OFF);
  SET_LED(LED_GREEN, ON);

  // Resume HeartBeatTask - Let Task Run
  vTaskResume(HeartBeatTask);
}

//=============================================================================
// Determines What Happens If MQTT Is Disconnected
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  // Turn On LED_RED And Turn Off LED_GREEN To Indicate Lost MQTT Connection
  SET_LED(LED_RED, ON);
  SET_LED(LED_GREEN, OFF);

  if (WiFi.isConnected()) {
    // If WiFi Is Still Connected Attempt MQTT Reconnect After Timer Has Ended
    xTimerStart(MQTT_ReconnectTimer, 0);
  }

  // Suspend HeartBeatTask Until Resumed Again
  vTaskSuspend(HeartBeatTask);
}

//=============================================================================
// Determines What Happens If Data Is Received On Subscribed Topics
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if (!(CMD_FLAG & FLAG_DELETING) && String(topic) == TOPIC_REC) {
    // Topic That Controls Start And Stop Of Measuring
    // Only Start Measuring If Device Is Not Currently Deleting
    switch (payload[0]) {
      case '0':
        {
          // Abort Data Acquisition Request
          CLEARBITMASK(CMD_FLAG, FLAG_RECORDING);
          break;
        }
      case '1':
        {
          // Start Recording Request
          xSemaphoreGive(xSemaphore);
          SETBITMASK(CMD_FLAG, FLAG_RECORDING);
          break;
        }
    }
  } else if (String(topic) == TOPIC_O2)  // Optinally Adding: !(CMD_FLAG & FLAG_DELETING) && (CMD_FLAG & FLAG_RECORDING) && - To Only Set Marker If In Measuring Mode (= Not Currently Deleting)
  {
    // Topic TThat Controls O₂ Marker Setting
    switch (payload[0]) {
      case '0':  // O₂ Is OFF - Default State
        {
          CLEARBITMASK(CMD_FLAG, FLAG_O2);
          break;
        }
      case '1':  // O₂ Is ON
        {
          SETBITMASK(CMD_FLAG, FLAG_O2);
          break;
        }
    }
  } else if (!(CMD_FLAG & FLAG_DELETING) && !(CMD_FLAG & FLAG_RECORDING) && String(topic) == TOPIC_GETDATA) {
    // Only Transmit Data If Device Is Not Currently Recording Or Deleting
    SETBITMASK(CMD_FLAG, FLAG_SEND_DATA);
  } else if (!(CMD_FLAG & FLAG_DELETING) && String(topic) == TOPIC_DEL) {
    // If Already Deleting, Do Not Delete Again
    SETBITMASK(CMD_FLAG, FLAG_DELETING);
  }
}