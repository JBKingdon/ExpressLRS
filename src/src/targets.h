#pragma once

/// General Features ///
#define FEATURE_OPENTX_SYNC //uncomment to use OpenTX packet sync feature (requires OpenTX 2.4 onwards) - this reduces latency.
#define LED_MAX_BRIGHTNESS 50 //0..255 for max led brightness
/////////////////////////

#ifdef PLATFORM_STM32
#define WORD_ALIGNED_ATTR //nothing//
#else
#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#endif

#ifdef PLATFORM_STM32
#define ICACHE_RAM_ATTR //nothing//
#else
#undef ICACHE_RAM_ATTR //fix to allow both esp32 and esp8266 to use ICACHE_RAM_ATTR for mapping to IRAM
#define ICACHE_RAM_ATTR IRAM_ATTR
#endif

#ifdef TARGET_TTGO_LORA_V1_AS_TX
#define GPIO_PIN_NSS 18
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0 26
#define GPIO_PIN_DIO1 -1
#define GPIO_PIN_MOSI 27
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 5
#define GPIO_PIN_RST 14
#define GPIO_PIN_OLED_SDA 4
#define GPIO_PIN_OLED_SCK 15
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

#ifdef TARGET_TTGO_LORA_V1_AS_RX
#endif

#ifdef TARGET_TTGO_LORA_V2_AS_TX
#define GPIO_PIN_NSS 18
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0 26
#define GPIO_PIN_DIO1 -1
#define GPIO_PIN_MOSI 27
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 5
#define GPIO_PIN_RST 14
#define GPIO_PIN_OLED_SDA 21
#define GPIO_PIN_OLED_SCK 22
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

#ifdef TARGET_TTGO_LORA_V2_AS_RX
 // not supported
#endif

#ifdef TARGET_EXPRESSLRS_PCB_TX_V3
#define GPIO_PIN_NSS 5
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0 26
#define GPIO_PIN_DIO1 25
#define GPIO_PIN_MOSI 23
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 18
#define GPIO_PIN_RST 14
#define GPIO_PIN_RX_ENABLE 13
#define GPIO_PIN_TX_ENABLE 12
#define GPIO_PIN_RCSIGNAL_RX 2
#define GPIO_PIN_RCSIGNAL_TX 2 // so we don't have to solder the extra resistor, we switch rx/tx using gpio mux
#endif

#ifdef TARGET_EXPRESSLRS_PCB_TX_V3_LEGACY
#define GPIO_PIN_BUTTON 36
#define RC_SIGNAL_PULLDOWN 4
#endif

#ifdef TARGET_EXPRESSLRS_PCB_RX_V3
#define GPIO_PIN_NSS 15
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0 4
#define GPIO_PIN_DIO1 5
#define GPIO_PIN_MOSI 13
#define GPIO_PIN_MISO 12
#define GPIO_PIN_SCK 14
#define GPIO_PIN_RST 2
#define GPIO_PIN_RCSIGNAL_RX -1 //only uses default uart pins so leave as -1 
#define GPIO_PIN_RCSIGNAL_TX -1
#define GPIO_PIN_LED 16
#define GPIO_PIN_LED 16
#define GPIO_PIN_BUTTON 2
#define timerOffset -1
#endif

#ifdef TARGET_R9M_RX
/*
Credit to Jacob Walser (jaxxzer) for the pinout!!!
https://github.com/jaxxzer
*/
#define GPIO_PIN_NSS            PB12
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0           PA15
#define GPIO_PIN_DIO1           PA1 // NOT CORRECT!!! PIN STILL NEEDS TO BE FOUND BUT IS CURRENTLY UNUSED
#define GPIO_PIN_MOSI           PB15
#define GPIO_PIN_MISO           PB14
#define GPIO_PIN_SCK            PB13
#define GPIO_PIN_RST            PC14
#define GPIO_PIN_SDA            PB7
#define GPIO_PIN_SCL            PB6
#define GPIO_PIN_RCSIGNAL_RX    PA10
#define GPIO_PIN_RCSIGNAL_TX    PA9
#define GPIO_PIN_LED            PC1 // Red
#define GPIO_PIN_LED_RED        PC1 // Red
#define GPIO_PIN_LED_GREEN      PB3 // Green 
#define GPIO_PIN_BUTTON         PC13  // pullup e.g. LOW when pressed
#define timerOffset             2

// External pads
// #define R9m_Ch1    PA8
// #define R9m_Ch2    PA11
// #define R9m_Ch3    PA9
// #define R9m_Ch4    PA10
// #define R9m_sbus   PA2
// #define R9m_sport  PA5
// #define R9m_isport PB11

//method to set HSE and clock speed correctly//
// #if defined(HSE_VALUE)
// /* Redefine the HSE value; it's equal to 8 MHz on the STM32F4-DISCOVERY Kit */
//#undef HSE_VALUE
//#define HSE_VALUE ((uint32_t)16000000).
//#define HSE_VALUE    25000000U
// #endif /* HSE_VALUE */
//#define SYSCLK_FREQ_72MHz
#endif

#ifdef TARGET_R9M_TX

#define GPIO_PIN_RFamp_APC1           PA6  //APC2 is connected through a I2C dac and is handled elsewhere
#define GPIO_PIN_RFswitch_CONTROL     PB3  //HIGH = RX, LOW = TX

#define GPIO_PIN_NSS            PB12
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0           PA15
#define GPIO_PIN_MOSI           PB15
#define GPIO_PIN_MISO           PB14
#define GPIO_PIN_SCK            PB13
#define GPIO_PIN_RST            PC14
#define GPIO_PIN_RX_ENABLE      GPIO_PIN_RFswitch_CONTROL
#define GPIO_PIN_TX_ENABLE      GPIO_PIN_RFamp_APC1
#define GPIO_PIN_SDA            PB7
#define GPIO_PIN_SCL            PB6
#define GPIO_PIN_RCSIGNAL_RX    PB11 // not yet confirmed
#define GPIO_PIN_RCSIGNAL_TX    PB10 // not yet confirmed
#define GPIO_PIN_LED_RED        PA11 // Red LED
#define GPIO_PIN_LED_GREEN      PA12 // Green LED
#define GPIO_PIN_BUTTON         PA8 // pullup e.g. LOW when pressed
#define GPIO_PIN_BUZZER         PB1
#define GPIO_PIN_DIP1           PA12 // dip switch 1
#define GPIO_PIN_DIP2           PA11 // dip switch 2

#define GPIO_PIN_DEBUG_RX    PA10 // confirmed
#define GPIO_PIN_DEBUG_TX    PA9 // confirmed

#define GPIO_PIN_BUZZER      PB1 // confirmed

#define BUFFER_OE               PA5  //CONFIRMED
#define GPIO_PIN_DIO1           PA1  //Not Needed, HEARTBEAT pin
#endif

#ifdef TARGET_R9M_LITE_TX

#define GPIO_PIN_RFswitch_CONTROL     PC13  // need to confirm  //HIGH = RX, LOW = TX

#define GPIO_PIN_NSS            PB12
#define GPIO_PIN_DIO0           PC15
#define GPIO_PIN_DIO1           -1    //unused for sx1280 
#define GPIO_PIN_BUSY           -1    //unused for sx1280 
#define GPIO_PIN_MOSI           PB15
#define GPIO_PIN_MISO           PB14
#define GPIO_PIN_SCK            PB13
#define GPIO_PIN_RST            PC14
#define GPIO_PIN_RX_ENABLE      PC13 //PB3 // need to confirm 
#define GPIO_PIN_SDA            PB7
#define GPIO_PIN_SCL            PB6
#define GPIO_PIN_RCSIGNAL_RX    PB11 // not yet confirmed
#define GPIO_PIN_RCSIGNAL_TX    PB10 // not yet confirmed
#define GPIO_PIN_LED_RED        PA1 // Red LED // not yet confirmed
#define GPIO_PIN_LED_GREEN      PA4 // Green LED // not yet confirmed

#define GPIO_PIN_DEBUG_RX    PA3 // confirmed
#define GPIO_PIN_DEBUG_TX    PA2 // confirmed

#define BUFFER_OE               PA5  //CONFIRMED

#endif

#ifdef TARGET_RX_ESP8266_SX1280_V1
#define GPIO_PIN_NSS         15
#define GPIO_PIN_BUSY         5
#define GPIO_PIN_DIO0        -1 // does not exist on sx1280
#define GPIO_PIN_DIO1         4
#define GPIO_PIN_DIO2         9
#define GPIO_PIN_MOSI        13
#define GPIO_PIN_MISO        12
#define GPIO_PIN_SCK         14
#define GPIO_PIN_RST          2
#define GPIO_PIN_RCSIGNAL_RX -1 //only uses default uart pins so leave as -1 
#define GPIO_PIN_RCSIGNAL_TX -1
#define GPIO_PIN_LED         16
#define GPIO_PIN_BUTTON       0
#define ANTENNA_SWITCH GPIO_PIN_BUTTON   // enables diversity when defined
#define ANTENNA_SWITCH_CMP   10
#define timerOffset          -1
#define MAX_PRE_PA_POWER     13
#endif

#ifdef TARGET_TX_ESP32_SX1280_V1
#define GPIO_PIN_NSS 5
#define GPIO_PIN_BUSY 21
#define GPIO_PIN_DIO0 -1 // does not exist on sx1280
#define GPIO_PIN_DIO1 4
#define GPIO_PIN_MOSI 23
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 18
#define GPIO_PIN_RST 14
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

#ifdef TARGET_TX_ESP32_E28_SX1280_V1
#define GPIO_PIN_NSS 5
#define GPIO_PIN_BUSY 21
#define GPIO_PIN_DIO0 -1
#define GPIO_PIN_DIO1 4
#define GPIO_PIN_MOSI 23
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 18
#define GPIO_PIN_RST 14
#define GPIO_PIN_RX_ENABLE 27
#define GPIO_PIN_TX_ENABLE 26
#define GPIO_PIN_OLED_SDA -1
#define GPIO_PIN_OLED_SCK -1
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

// For the 2nd DAG handset prototype

#ifdef TARGET_TX_DAG_V1
#define GPIO_PIN_NSS         21
#define GPIO_PIN_BUSY        14
#define GPIO_PIN_DIO0        -1
#define GPIO_PIN_DIO1        27
#define GPIO_PIN_MOSI        23
#define GPIO_PIN_MISO        19
#define GPIO_PIN_SCK         18
#define GPIO_PIN_RST         12
#define GPIO_PIN_RX_ENABLE   22
#define GPIO_PIN_TX_ENABLE   13
// #define GPIO_PIN_RX_ENABLE   -1     // e28-12 doesn't have enables
// #define GPIO_PIN_TX_ENABLE   -1
#define GPIO_PIN_RCSIGNAL_RX -1 // keep the crsf lib happy
#define GPIO_PIN_RCSIGNAL_TX -1
#define GPIO_PIN_LED         2

#define GPIO_PIN_COPRO_RX   16
#define GPIO_PIN_COPRO_TX   17

#define GPIO_PIN_DEBUG       -1
#define GPIO_PIN_BUTTON       0

#define ADC_ROLL             36 // Warning - these are currently ignored
#define ADC_PITCH            39 // and values are hard coded in initADC()
#define ADC_THROTTLE         34 // .
#define ADC_YAW              35 // .

// For the non-amplified E28-12
// #define MAX_PRE_PA_POWER     13
// #define MIN_PRE_PA_POWER    -18

// For the non-amplified E28-20
#define MAX_PRE_PA_POWER     -2
#define MIN_PRE_PA_POWER    -18

#endif // TARGET_TX_DAG_V1

// For the 3rd DAG handset prototype

#ifdef TARGET_TX_DAG_V3

#define USE_TFT
#define USE_ADC_COPRO

// if using ttgo t-display, lcd is hardwired to
// 19, 18, 5, 16, 23, 14


#define GPIO_PIN_SCK        26
#define GPIO_PIN_MISO       25
#define GPIO_PIN_ADCCP_SS   27
#define GPIO_PIN_MOSI       33


#define GPIO_PIN_NSS         21
#define GPIO_PIN_BUSY        14
#define GPIO_PIN_DIO0        -1
#define GPIO_PIN_DIO1        27
// #define GPIO_PIN_MOSI        23
// #define GPIO_PIN_MISO        19
// #define GPIO_PIN_SCK         18
#define GPIO_PIN_RST         12
#define GPIO_PIN_RX_ENABLE   22
#define GPIO_PIN_TX_ENABLE   13
// #define GPIO_PIN_RX_ENABLE   -1     // e28-12 doesn't have enables
// #define GPIO_PIN_TX_ENABLE   -1
#define GPIO_PIN_RCSIGNAL_RX -1 // keep the crsf lib happy
#define GPIO_PIN_RCSIGNAL_TX -1
#define GPIO_PIN_LED         2

#define GPIO_PIN_COPRO_RX   15
#define GPIO_PIN_COPRO_TX   17

#define GPIO_PIN_DEBUG       -1
#define GPIO_PIN_BUTTON       0

// For the non-amplified E28-12
// #define MAX_PRE_PA_POWER     13
// #define MIN_PRE_PA_POWER    -18

// For the non-amplified E28-20
#define MAX_PRE_PA_POWER     -2
#define MIN_PRE_PA_POWER    -18

#endif // TARGET_TX_DAG_V1


#ifdef TARGET_TX_DAG_V1_COPRO

// keep CRSF.cpp happy
#define GPIO_PIN_RCSIGNAL_RX -1
#define GPIO_PIN_RCSIGNAL_TX -1

#endif

// first dag prototype

// #ifdef TARGET_TX_DAG_V1
// #define GPIO_PIN_NSS         32 
// #define GPIO_PIN_BUSY        22
// #define GPIO_PIN_DIO0        -1
// #define GPIO_PIN_DIO1        17
// #define GPIO_PIN_MOSI        25
// #define GPIO_PIN_MISO        26
// #define GPIO_PIN_SCK         33
// #define GPIO_PIN_RST         21
// // #define GPIO_PIN_RX_ENABLE   -1     // e28-12 doesn't have enables
// // #define GPIO_PIN_TX_ENABLE   -1
// // #define GPIO_PIN_OLED_SDA 21
// // #define GPIO_PIN_OLED_SCK 22
// #define GPIO_PIN_RCSIGNAL_RX -1
// #define GPIO_PIN_RCSIGNAL_TX -1
// #define GPIO_PIN_LED         2

// #define GPIO_PIN_DEBUG       -1
// #define GPIO_PIN_BUTTON       0

// #define ADC_ROLL             39 // Warning - these are currently ignored
// #define ADC_PITCH            38 // and values are hard coded in initADC()
// #define ADC_THROTTLE         37 // .
// #define ADC_YAW              36 // .

// // moved to copro
// // #define ADC_BATTERY          13

// // #define GPIO_AUX1            27  // switches moved to copro
// // #define GPIO_AUX2             2
// // #define GPIO_AUX3            12
// // #define GPIO_AUX4            15

// // #define GPIO_BUTTON1         35  // buttons were ttgo specific
// // #define GPIO_BUTTON2         0

// // For the non-amplified E28-12
// // #define MAX_PRE_PA_POWER     13
// // #define MIN_PRE_PA_POWER    -18

// // For the non-amplified E28-20
// #define MAX_PRE_PA_POWER     -2
// #define MIN_PRE_PA_POWER    -18

// #endif // TARGET_TX_DAG_V1

// #ifdef TARGET_TX_DAG_V1_COPRO

// // keep CRSF.cpp happy
// #define GPIO_PIN_RCSIGNAL_RX -1
// #define GPIO_PIN_RCSIGNAL_TX -1

// #endif



// older version
// #ifdef TARGET_TX_PICO_E28_SX1280_V1
// #define GPIO_PIN_NSS         10
// #define GPIO_PIN_BUSY        38
// #define GPIO_PIN_DIO0        -1
// #define GPIO_PIN_DIO1        37
// #define GPIO_PIN_MOSI        18
// #define GPIO_PIN_MISO        23
// #define GPIO_PIN_SCK          5
// #define GPIO_PIN_RST         32
// #define GPIO_PIN_RX_ENABLE    9
// #define GPIO_PIN_TX_ENABLE   33
// // #define GPIO_PIN_TX_ENABLE   25     // unconnected, harmless, use to disable the PA for testing
// // #define GPIO_PIN_OLED_SDA 21
// // #define GPIO_PIN_OLED_SCK 22
// #define GPIO_PIN_RCSIGNAL_RX 15
// #define GPIO_PIN_RCSIGNAL_TX 15
// #define GPIO_PIN_BLUE_LED     2 // led1
// #define GPIO_PIN_GREEN_LED   13 // led2

// #define GPIO_PIN_LED         GPIO_PIN_BLUE_LED  // convenince for compatability

// #define GPIO_PIN_DEBUG       21
// #define GPIO_PIN_BUTTON       0
// #endif