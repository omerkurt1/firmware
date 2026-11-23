#pragma once

#define HAS_SCREEN 1
#define HAS_GPS 1
#define HAS_WIRE 1
#define HAS_TELEMETRY 1

#define LED_PIN LED_GREEN
#define LED_STATE_ON HIGH
#define BUTTON_PIN USER_BTN
#define BUTTON_ACTIVE_LOW
#define BUTTON_NEED_PULLUP
#define ALT_BUTTON_PIN SW2_BTN
#define DOWN_BUTTON_PIN SW3_BTN

#define GPS_SERIAL_PORT SerialLP1
#define GPS_TX_PIN PA2
#define GPS_RX_PIN PA3
#define GPS_BAUDRATE 9600

#define WB_CC1101_CS PC12
#define WB_CC1101_GDO0 PC13
#define WB_SX1262_CS PA9

// Enable the external radio after confirming the fitted module's TCXO voltage.
#ifdef GPS_WB_SX1262_TCXO_MV
#define HAS_RADIO 1
#define USE_SX1262
#define LORA_SCK PA5
#define LORA_MISO PA6
#define LORA_MOSI PA7
#define LORA_CS WB_SX1262_CS
#define SX126X_CS LORA_CS
#define SX126X_DIO1 PC6
#define SX126X_RESET PA10
#define SX126X_BUSY PC10
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_USE_REGULATOR_LDO
#define SX126X_MAX_POWER 20
#define SX126X_DIO3_TCXO_VOLTAGE (GPS_WB_SX1262_TCXO_MV / 1000.0f)
#endif
