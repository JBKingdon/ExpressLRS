#include <Arduino.h>
#include <cmath>
#include "FIFO.h"
#include <EEPROM.h>
#include "utils.h"
#include "common.h"
#include "LowPassFilter.h"
#include "1AUDfilter.h"
// #include "1AUDfilterInt.h"

// #include "esp32-hal-uart.h"

// #define DEBUG_SUPPRESS

// #define USE_DMA_ADC

#ifdef USE_DMA_ADC
#include "dmaAdc.h"
#endif


#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
#include "SX127xDriver.h"
SX127xDriver Radio;
#elif defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)
#include "SX1280Driver.h"
SX1280Driver Radio;
// static int radioPower = MAX_PRE_PA_POWER;
static int radioPower = -10; // arbitrary but safe default
#endif

#include "CRSF.h"
#include "FHSS.h"
// #include "LED.h"
// #include "debug.h"
#include "targets.h"
#include "POWERMGNT.h"
#include "msp.h"
#include "msptypes.h"
#include <OTA.h>
//#include "elrs_eeprom.h"

#ifdef PLATFORM_ESP8266
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

#ifdef PLATFORM_ESP32
#include "ESP32_hwTimer.h"
#include <driver/adc.h>
#ifdef USE_TFT
#include <TFT_eSPI.h>
#endif
#include "OneEuroFilter.h"
// #include "driver/uart.h"

#ifdef USE_IO_COPRO
#include "UI_CoPro.h"
#endif // USE_IO_COPRO
#endif // PLATFORM_ESP32

#ifdef TARGET_R9M_TX
#include "DAC.h"
#include "STM32_hwTimer.h"
#include "button.h"
button button;
R9DAC R9DAC;
#endif

#ifdef TARGET_R9M_LITE_TX
#include "STM32_hwTimer.h"
#endif

//// CONSTANTS ////
#define RX_CONNECTION_LOST_TIMEOUT 3000 // After 1500ms of no TLM response consider that slave has lost connection
#define PACKET_RATE_INTERVAL 500
#define RF_MODE_CYCLE_INTERVAL 1000
#define MSP_PACKET_SEND_INTERVAL 200
#define SYNC_PACKET_SEND_INTERVAL_RX_LOST 1000 // how often to send the SYNC_PACKET packet (ms) when there is no response from RX
#define SYNC_PACKET_SEND_INTERVAL_RX_CONN 5000 // how often to send the SYNC_PACKET packet (ms) when there we have a connection


// struct for saving defaults in eeprom
// TODO move to a .h
typedef struct {

  uint8_t airRateIndex;
  int8_t  prePApower;
  uint8_t tlmInterval;
  uint8_t crc;

} defaultSettings_t;


String DebugOutput;

/// define some libs to use ///
hwTimer hwTimer;
CRSF crsf;
POWERMGNT POWERMGNT;
MSP msp;

void ICACHE_RAM_ATTR TimerCallbackISR();
volatile uint8_t NonceTX;

//// MSP Data Handling ///////
uint32_t MSPPacketLastSent = 0;  // time in ms when the last switch data packet was sent
uint32_t MSPPacketSendCount = 0; // number of times to send MSP packet
mspPacket_t MSPPacket;

////////////SYNC PACKET/////////
uint32_t SyncPacketLastSent = 0;

uint32_t LastTLMpacketRecvMillis = 0;
bool isRXconnected = false;
int packetCounteRX_TX = 0;
uint32_t PacketRateLastChecked = 0;
float PacketRate = 0.0;
uint8_t linkQuality = 0;

/// Variables for Sync Behaviour ////
uint32_t RFmodeLastCycled = 0;
///////////////////////////////////////

volatile bool UpdateParamReq = false;
volatile bool UpdateRFparamReq = false;

volatile bool RadioIsIdle = false;

bool Channels5to8Changed = false;

bool ChangeAirRateRequested = false;
bool ChangeAirRateSentUpdate = false;

bool WaitRXresponse = false;

bool saveNeeded = false;
bool sendStateNeeded = false;

// Used to indicate that a link rate change needs to be done
int8_t nextRFLinkRate = -1;
#define NONE_PENDING 127
int8_t nextRadioPower = NONE_PENDING; // can't use -1 as that's a valid power

///// Not used in this version /////////////////////////////////////////////////////////////////////////////////////////////////////////
// uint8_t TelemetryWaitBuffer[7] = {0};

// uint32_t LinkSpeedIncreaseDelayFactor = 500; // wait for the connection to be 'good' for this long before increasing the speed.
// uint32_t LinkSpeedDecreaseDelayFactor = 200; // this long wait this long for connection to be below threshold before dropping speed

// uint32_t LinkSpeedDecreaseFirstMetCondition = 0;
// uint32_t LinkSpeedIncreaseFirstMetCondition = 0;

// uint8_t LinkSpeedReduceSNR = 20;   //if the SNR (times 10) is lower than this we drop the link speed one level
// uint8_t LinkSpeedIncreaseSNR = 60; //if the SNR (times 10) is higher than this we increase the link speed
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// MSP packet handling function defs
void ProcessMSPPacket(mspPacket_t *packet);
void OnRFModePacket(mspPacket_t *packet);
void OnTxPowerPacket(mspPacket_t *packet);
void OnTLMRatePacket(mspPacket_t *packet);

// uint8_t baseMac[6];

// ===================================
// variables for direct attached gimbals

#ifdef USE_TFT
TFT_eSPI tft = TFT_eSPI();
static bool lcdNeedsRedraw = true;
#endif

// unsigned long tLeaveISR;

unsigned long tRF, tADC;
static volatile uint16_t cpuLoad;
LPF LPF_cpuLoad(3,1);

uint32_t nADCgroups = 0;

// variables for checking ADC performance
// static unsigned long minADC=99999, maxADC=0, totalADC=0;

// time when arm switch is enabled/disabled
// static unsigned long armStartTime = 0;
// static unsigned long armEndTime = 0;


// and the battery sensor
// LPF LPF_battery(3);

#ifdef USE_1E
// testing the 1e filter
double frequency = 800 ; // Hz - is set from setRFLinkRate, so this value isn't important
double mincutoff = 0.35 ; // min=.4, b=.06 looks pretty good on bench test. Maybe try slightly lower min & higher beta?
double beta = 0.06 ;     
double dcutoff = 1.0 ;   // orig desc: this one should be ok

OneEuroFilter f_roll(frequency, mincutoff, beta, dcutoff);
OneEuroFilter f_pitch(frequency, mincutoff, beta, dcutoff);
OneEuroFilter f_throttle(frequency, mincutoff, beta, dcutoff);
OneEuroFilter f_yaw(frequency, mincutoff, beta, dcutoff);
#endif // USE_1E


// The 1AUD

float minCutoff =  20;
float maxCutoff = 200;
float beta = 0.01f;
// slewLimit is now steps per second and gets auto converted when sample rate changes
float slewLimit = 100000.0f;

#ifdef USE_DMA_ADC
float sampleRate = float(TOTAL_SAMPLE_RATE) / GROUP_SAMPLE_LENGTH;
#else
// interval is in us
// rate is (1000000 / interval) * oversamples and oversamples is interval/200, so rate is
// 1000000 / 200  = 5000
float sampleRate = 5000;
#endif // not USE_DMA_ADC

float dCutoff = 50.0f;

OneAUDfilter aud_roll(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
OneAUDfilter aud_pitch(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
OneAUDfilter aud_yaw(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
OneAUDfilter aud_throttle(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);

// test the int impl of 1aud

// OneAUDfilterInt audI_roll(minCutoff, maxCutoff, int(1.0f/beta), sampleRate, dCutoff, slewLimit);
// OneAUDfilterInt audI_pitch(minCutoff, maxCutoff, int(1.0f/beta), sampleRate, dCutoff, slewLimit);
// OneAUDfilterInt audI_yaw(minCutoff, maxCutoff, int(1.0f/beta), sampleRate, dCutoff, slewLimit);
// OneAUDfilterInt audI_throttle(minCutoff, maxCutoff, int(1.0f/beta), sampleRate, dCutoff, slewLimit);

TaskHandle_t readCoProTask, readADCsTaskHandle, readGimbalsDMATaskHandle;

// static QueueHandle_t copro_uart_queue;

// -----------------------------------
#ifdef USE_DMA_ADC
void writePatternTableEntry(adc_hal_digi_pattern_table_t ptEntry, const uint32_t pattern_index)
{
  uint32_t tab;
  uint8_t index = pattern_index / 4;
  uint8_t offset = (pattern_index % 4) * 8;

  tab = SYSCON.saradc_sar1_patt_tab[index];   // Read old register value
  tab &= (~(0xFF000000 >> offset));           // clear old data
  tab |= ((uint32_t)ptEntry.val << 24) >> offset; // Fill in the new data
  SYSCON.saradc_sar1_patt_tab[index] = tab;   // Write back
}

void initI2S(void)
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = TOTAL_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // stops the sample rate being doubled vs LEFT_RIGHT
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,
        .dma_buf_count = 16,
        .dma_buf_len = GROUP_SAMPLE_LENGTH * 2,
        .use_apll = 1,
        .tx_desc_auto_clear = 1,
        .fixed_mclk = 1
    };

    //install and start i2s driver
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

    // i2s_stop(I2S_NUM_0);
    //init ADC pad
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);

    // make sure all the anlog channels are setup
    // currently using 0, 3, 6, 7
    pinMode(ADC1_CHANNEL_3_GPIO_NUM, ANALOG);
    pinMode(ADC1_CHANNEL_6_GPIO_NUM, ANALOG);
    pinMode(ADC1_CHANNEL_7_GPIO_NUM, ANALOG);

    // Now we need to set the pattern table
    // TODO do a 4x loop unroll and bin?

    // adc_hal_digi_pattern_table_t pt_entry[4];

    // pt_entry[0].channel = ADC1_CHANNEL_0;
    // pt_entry[1].channel = ADC1_CHANNEL_3;
    // pt_entry[2].channel = ADC1_CHANNEL_6;
    // pt_entry[3].channel = ADC1_CHANNEL_7;

    // // set the common bits and write to the hardware
    // for(int i=0; i<4; i++) {
    //   pt_entry[i].atten = ADC_ATTEN_DB_6;
    //   pt_entry[i].bit_width = ADC_WIDTH_BIT_12;
    //   writePatternTableEntry(pt_entry[i], i);
    // }

    // SYSCON.saradc_ctrl.sar1_patt_len = 3; // NB value to set is nPatterns-1

    adc_hal_digi_pattern_table_t pt_entry[4];
    uint8_t channels[4] = {ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_6, ADC1_CHANNEL_7};

    for(int i=0; i<4; i++) {
      pt_entry[i].channel = channels[i];
      pt_entry[i].atten = ADC_ATTEN_DB_11;
      pt_entry[i].bit_width = ADC_WIDTH_BIT_12;
      writePatternTableEntry(pt_entry[i], i);
    }

    SYSCON.saradc_ctrl.sar1_patt_len = 3; // NB value to set is nPatterns-1

    // set the 'inv' flag so that the results match those from the normal adc read api
    SYSCON.saradc_ctrl2.sar1_inv = 1;
}
#endif // USE_DMA_ADC

void initADC()
{
  #ifdef USE_DMA_ADC
  initI2S();
  #else
  adc1_config_width(ADC_WIDTH_BIT_12);

  // TODO convert from the target PIN numbers to ADC channels
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_6);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_6);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_6);
  #endif // USE_DMA_ADC

  // battery sensor
  // adc2_config_channel_atten(ADC2_CHANNEL_4, ADC_ATTEN_6db); // moved to copro

  // init the battery LPF
  // delay(10);
  // int batRaw;
  // for(int i=0; i<200; i++) {
  //   adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_BIT_12, &batRaw);
  //   LPF_battery.update(batRaw);
  // }

}

// crsf uses a reduced range, and BF expects to see it.
const static uint32_t MAX_OUT = 1811;
const static uint32_t MID_OUT =  992;
const static uint32_t MIN_OUT =  172;

uint32_t scaleYawData(uint16_t raw_adc_yaw)
{
  // non DMA
  // not reversed
  const static uint16_t ADC_YAW_MIN = 811;
  const static uint16_t ADC_YAW_CTR = 1951;
  const static uint16_t ADC_YAW_MAX = 3177;

  // reversed
  // const static uint16_t ADC_YAW_MIN = 920;
  // const static uint16_t ADC_YAW_CTR = 2143;
  // const static uint16_t ADC_YAW_MAX = 3280;


  // yaw is not reversed
  uint32_t adc_yaw;
  if (raw_adc_yaw <= ADC_YAW_MIN) {
    adc_yaw = MIN_OUT;
  } else if (raw_adc_yaw >= ADC_YAW_MAX) {
    adc_yaw = MAX_OUT;
  } else if (raw_adc_yaw > ADC_YAW_CTR) {
    // high half scaling
    adc_yaw = MID_OUT + (raw_adc_yaw - ADC_YAW_CTR) * (MID_OUT - MIN_OUT) / (ADC_YAW_MAX - ADC_YAW_CTR);
  } else {
    // low half scaling
    adc_yaw = MID_OUT - ((ADC_YAW_CTR - raw_adc_yaw) * (MAX_OUT - MID_OUT) / (ADC_YAW_CTR - ADC_YAW_MIN));
  }

  return adc_yaw;
}

uint32_t scaleThrottleData(uint16_t raw_adc_throttle)
{
  // non DMA
  // reversed, no centre needed for throttle
  const static uint16_t ADC_THROTTLE_MIN = 784;
  const static uint16_t ADC_THROTTLE_MAX = 3202;

  // not reversed
  // const static uint16_t ADC_THROTTLE_MIN = 895;
  // const static uint16_t ADC_THROTTLE_MAX = 3305;

  uint32_t adc_throttle;
  if (raw_adc_throttle <= ADC_THROTTLE_MIN) {
    adc_throttle = MAX_OUT;
  } else if (raw_adc_throttle >= ADC_THROTTLE_MAX) {
    adc_throttle = MIN_OUT;
  } else {
    adc_throttle = MAX_OUT - ((raw_adc_throttle - ADC_THROTTLE_MIN) * (MAX_OUT - MIN_OUT) / (ADC_THROTTLE_MAX - ADC_THROTTLE_MIN));
  }
  // Serial.printf("throttle raw %u, scaled %u\n", raw_adc_throttle, adc_throttle);
  return adc_throttle;
}

uint32_t scalePitchData(uint16_t raw_adc_pitch)
{
  // non-dma
  // not reversed
  const static uint16_t ADC_PITCH_MIN = 1098;
  const static uint16_t ADC_PITCH_CTR = 2024;
  const static uint16_t ADC_PITCH_MAX = 3040;

  // reversed
  // const static uint16_t ADC_PITCH_MIN = 1055;
  // const static uint16_t ADC_PITCH_CTR = 2063;
  // const static uint16_t ADC_PITCH_MAX = 3007;

  uint32_t adc_pitch;
  if (raw_adc_pitch <= ADC_PITCH_MIN) {
    adc_pitch = MIN_OUT;
  } else if (raw_adc_pitch >= ADC_PITCH_MAX) {
    adc_pitch = MAX_OUT;
  } else if (raw_adc_pitch > ADC_PITCH_CTR) {
    // high half scaling
    adc_pitch = MID_OUT + (raw_adc_pitch - ADC_PITCH_CTR) * (MAX_OUT - MID_OUT) / (ADC_PITCH_MAX - ADC_PITCH_CTR);
  } else {
    // low half scaling
    adc_pitch = MID_OUT - ((ADC_PITCH_CTR - raw_adc_pitch) * (MID_OUT - MIN_OUT) / (ADC_PITCH_CTR - ADC_PITCH_MIN));
  }

  return adc_pitch;
}


uint32_t scaleRollData(uint16_t raw_adc_roll)
{
  // Values as read from the ADC - will change if width or atten are changed

  // non-dma and new dma
  // roll is reversed
  const static uint16_t ADC_ROLL_MIN = 620;
  const static uint16_t ADC_ROLL_CTR = 1920;
  const static uint16_t ADC_ROLL_MAX = 3008;

  // old dma
  // roll is not reversed
  // const static uint16_t ADC_ROLL_MIN = 1090;
  // const static uint16_t ADC_ROLL_CTR = 2175;
  // const static uint16_t ADC_ROLL_MAX = 3470;


  // transition smoothly between the high and low scales across the centre
  // const static float rollHighScale = float(MAX_OUT - MID_OUT) / float(ADC_ROLL_MAX - ADC_ROLL_CTR);
  // const static float rollLowScale = float(MID_OUT - MIN_OUT) / float(ADC_ROLL_CTR - ADC_ROLL_MIN);
  // const static float rollScaleRange = rollHighScale - rollLowScale;
  // define a range over which the scale should transition from one to the other.
  // const static uint32_t SCALE_TRANSITION_PERCENT = 5;
  // const static uint32_t SCALE_RANGE = (ADC_ROLL_MAX - ADC_ROLL_MIN) * SCALE_TRANSITION_PERCENT / 100;
  // const static uint32_t SCALE_RANGE_START = ADC_ROLL_CTR - SCALE_RANGE;
  // const static uint32_t SCALE_RANGE_END = ADC_ROLL_CTR + SCALE_RANGE;

  // roll is reversed
  uint32_t adc_roll;
  
  if (raw_adc_roll <= ADC_ROLL_MIN) {
    adc_roll = MAX_OUT;
  } else if (raw_adc_roll >= ADC_ROLL_MAX) {
    adc_roll = MIN_OUT;
  } else if (raw_adc_roll > ADC_ROLL_CTR) {
    // high half scaling
    // float activeRollScale = rollHighScale;
    // // if we're in the transition zone crossfade the scaling factor between the low and high scales
    // if (raw_adc_roll < SCALE_RANGE_END) {
    //   activeRollScale = rollLowScale + rollScaleRange * (raw_adc_roll - SCALE_RANGE_START) / SCALE_RANGE;
    // }
    adc_roll = MID_OUT - (raw_adc_roll - ADC_ROLL_CTR) * (MID_OUT - MIN_OUT) / (ADC_ROLL_MAX - ADC_ROLL_CTR);
  } else {
    // low half scaling
    // float activeRollScale = rollLowScale;
    // if (raw_adc_roll > SCALE_RANGE_START) {
    //   activeRollScale = rollLowScale + rollScaleRange * (raw_adc_roll - SCALE_RANGE_START) / SCALE_RANGE;
    // }
    adc_roll = MID_OUT + (ADC_ROLL_CTR - raw_adc_roll) * (MAX_OUT - MID_OUT) / (ADC_ROLL_CTR - ADC_ROLL_MIN);
  }

  return adc_roll;
}

#ifdef USE_DMA_ADC

/**
 * Read the gimbals using DMA
 * 
*/
void readGimbalsViaDMA_task(void *pvParameters)
{
  uint16_t buffer[GROUP_SAMPLE_LENGTH];

  for(;;) {

    // read a block of raw adc values
    size_t bytes_read = 0, total_read = 0;
    const size_t bytesNeeded = GROUP_SAMPLE_LENGTH * 2;

    // i2s_adc_enable(I2S_NUM_0); // this breaks everything. Probably needs to have the pattern tables setup again after the disable

    while(total_read < bytesNeeded) {
        // XXX TODO currently always gets everything in 1 call, but needs updating to work properly if it ever needs >1 attempt
        // add offsets for buffer and subtract total read from needed
        i2s_read(I2S_NUM_0, (void*) buffer, bytesNeeded, &bytes_read, portMAX_DELAY); // this will wait until enough data is ready
        // Serial.print("bytes read ");Serial.println(bytes_read);
        total_read += bytes_read;
    }

    // i2s_adc_disable(I2S_NUM_0);

    // for perf debug
    nADCgroups++;

    // uint32_t rawRoll = 0, nRoll = 0;
    // uint32_t rawYaw = 0, nYaw = 0;

    uint32_t rawRoll=0, rawPitch=0, rawThrottle=0, rawYaw=0;
    uint32_t nRoll, nPitch, nThrottle, nYaw;
    nRoll = nPitch = nThrottle = nYaw = 0;

    // for each raw value
    for(int i=0; i<GROUP_SAMPLE_LENGTH; i++)
    {
      // get the channel and data
      uint32_t ch = buffer[i] >> 12;    // channel is in the top 4 bits
      uint32_t value = buffer[i] & 0x0FFFu;
      // Serial.printf("%x %u %u\n", buffer[i], ch, value);

      // update the corresponding filter for the channel
      switch(ch) {
        case 0:
          // if the adc barfs 0s they end up here, so filter them out
          if (value != 0) {
            // aud_throttle.update(value);
            rawThrottle += value;
            nThrottle++;
            // Serial.println(buffer[i]);
          }
          break;
        case 3:
          // aud_yaw.update(value);
          rawYaw += value;
          nYaw++;
          break;
        case 6:
          // aud_pitch.update(value);
          rawPitch += value;
          nPitch++;
          break;
        case 7:
          // aud_roll.update(value);
          Serial.println(value);
          rawRoll += value;
          nRoll++;
          break;
        default:
          Serial.printf("unknown channel %u\n", ch);
          break;
      }

    }

    // simple averaging
    if (nRoll) rawRoll /= nRoll;
    if (nPitch) rawPitch /= nPitch;
    if (nThrottle) rawThrottle /= nThrottle;
    if (nYaw) rawYaw /= nYaw;

    // feed the raw values into the filters
    aud_roll.update(rawRoll);
    aud_pitch.update(rawPitch);
    aud_throttle.update(rawThrottle);
    aud_yaw.update(rawYaw);

    // Serial.print(rawRoll);
    // Serial.print(' ');
    // Serial.print(rawPitch);
    // Serial.print(' ');
    // Serial.print(rawThrottle);
    // Serial.print(' ');
    // Serial.println(rawYaw);

    // debug for oversampled and filtered
    // Serial.print(rawRoll); Serial.print(' '); Serial.println(aud_roll.getCurrentAsInt());

    // Serial.print(micros()); Serial.print(' '); Serial.print(aud_roll.getCurrentAsInt());

    // Serial.printf("R %u P %u T %u Y %u\n", unscaled_roll, unscaled_pitch, unscaled_throttle, unscaled_yaw);
    // Serial.print(rawYaw); Serial.print(' ');
    // Serial.print(unscaled_yaw); Serial.print(' ');
    // Serial.print(rawRoll); Serial.print(' ');
    // Serial.println(unscaled_roll); 
    // Serial.print(" ");
    // Serial.print(unscaled_pitch); Serial.print(" ");
    // Serial.print(unscaled_throttle); Serial.print(" ");
    // Serial.println(unscaled_yaw);


  }
 
}

#else // USE_DMA_ADC

/**
 * reading the gimbals
 * 
 * POC version, simplistic (and slow) reading strategy.
*/
void ICACHE_RAM_ATTR refreshGimbalData()
{
  const unsigned int OVERSAMPLE = (ExpressLRS_currAirRate_Modparams->interval / 200); // TODO #def

  uint32_t ovs = 0;

  // 1AUD filters
  for(int i=0; i<OVERSAMPLE; i++) {
    const int rawRoll = adc1_get_raw(ADC1_CHANNEL_7);
    const int rawPitch = adc1_get_raw(ADC1_CHANNEL_6);
    const int rawThrottle = adc1_get_raw(ADC1_CHANNEL_0);
    const int rawYaw = adc1_get_raw(ADC1_CHANNEL_3);

    ovs += rawRoll;

    aud_roll.update(rawRoll);
    aud_pitch.update(rawPitch);
    aud_throttle.update(rawThrottle);
    aud_yaw.update(rawYaw);

    // audI_roll.update(rawRoll);
    // audI_pitch.update(rawPitch);
    // audI_throttle.update(rawThrottle);
    // audI_yaw.update(rawYaw);

    // debug for raw samples
    // Serial.print(rawRoll); Serial.print(' ');
    // Serial.print(rawPitch); Serial.print(' ');
    // Serial.print(rawThrottle); Serial.print(' ');
    // Serial.println(rawYaw);
    
  }

  // debug ovs vs filter
  // Serial.print(ovs/OVERSAMPLE); Serial.print(' '); 
  // Serial.print(aud_roll.getCurrent()); Serial.print(' ');
  // Serial.println(audI_roll.getCurrent());

  // debug 4 filters
  // Serial.print(aud_roll.getCurrentAsInt()); Serial.print(' ');
  // Serial.print(aud_pitch.getCurrentAsInt()); Serial.print(' ');
  // Serial.print(aud_throttle.getCurrentAsInt()); Serial.print(' ');
  // Serial.println(aud_yaw.getCurrentAsInt());


  #ifndef USE_IO_COPRO
  // switches
  int a1 = digitalRead(GPIO_AUX1);
  int a2 = digitalRead(GPIO_AUX2);
  int a3 = digitalRead(GPIO_AUX3);
  int a4 = digitalRead(GPIO_AUX4);

  // check if the arm switch has been enabled and record the time
  if (a1 && crsf.currentSwitches[0] == 0) {
    // switch on
    armStartTime = millis();
    armEndTime = 0;
  } else if (a1 == 0 && crsf.currentSwitches[0]) {
    // switch off
    armEndTime = millis();
  }

  crsf.currentSwitches[0] = a1*2;
  crsf.currentSwitches[1] = a2*2;
  crsf.currentSwitches[2] = a3*2;
  crsf.currentSwitches[3] = a4*2;
  #endif // USE_IO_COPRO

}

#endif // USE_DMA_ADC


/**
 * Store the current configurable radio settings as defaults in the eeprom
 */
void saveSettingsAsDefaults()
{
  defaultSettings_t savedDefaults;
  savedDefaults.airRateIndex = ExpressLRS_currAirRate_Modparams->index;
  savedDefaults.prePApower = Radio.currPWR;
  savedDefaults.tlmInterval = ExpressLRS_currAirRate_Modparams->TLMinterval;

  savedDefaults.crc = CalcCRC((uint8_t*)&savedDefaults, sizeof(defaultSettings_t)-1);

  hwTimer.stop();

  EEPROM.put(0, savedDefaults);
  EEPROM.commit();

  hwTimer.resume();

  Serial.println("current settings saved as defaults");
}

/** Send the current TX settings to the copro for display
 * 
 */
void sendTxStateToCoPro()
{
  txModePkt_u pktU;
  txModePkt_t *pkt = &pktU.txModePkt;

  pkt->eyecatcher = PKT_EYECATCHER;
  pkt->packetType = PKT_TXMODE;
  pkt->payload.airRate = 1000000/ExpressLRS_currAirRate_Modparams->interval;
  pkt->payload.power = Radio.getPowerMw();
  pkt->payload.tlmInterval = TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);

  pkt->crc = CalcCRC(pktU.dataBytes, sizeof(txModePkt_t)-1);

  Serial1.write(pktU.dataBytes, sizeof(txModePkt_t));
}


void ICACHE_RAM_ATTR ProcessTLMpacket()
{
  uint8_t calculatedCRC = CalcCRC(Radio.RXdataBuffer, 7) + CRCCaesarCipher;
  uint8_t inCRC = Radio.RXdataBuffer[7];
  uint8_t type = Radio.RXdataBuffer[0] & TLM_PACKET;
  uint8_t packetAddr = (Radio.RXdataBuffer[0] & 0b11111100) >> 2;
  uint8_t TLMheader = Radio.RXdataBuffer[1];

  //Serial.println("TLMpacket0");

  if (packetAddr != DeviceAddr)
  {
#ifndef DEBUG_SUPPRESS
    Serial.println("TLM device address error");
#endif
    return;
  }

  if ((inCRC != calculatedCRC))
  {
#ifndef DEBUG_SUPPRESS
    Serial.println("TLM crc error");
#endif
    return;
  }

  packetCounteRX_TX++;

  if (type != TLM_PACKET)
  {
#ifndef DEBUG_SUPPRESS
    Serial.println("TLM type error");
    Serial.println(type);
#endif
    return;
  }

  // turn on the led if this is a state change
  if (!isRXconnected) {
    digitalWrite(GPIO_PIN_LED, 1);
  }

  isRXconnected = true;
  LastTLMpacketRecvMillis = millis();

  if (TLMheader == CRSF_FRAMETYPE_LINK_STATISTICS)
  {
    #ifdef USE_ELRS_CRSF_EXTENSIONS
    #error not implemented
    #else
    crsf.LinkStatistics.uplink_RSSI_1 = Radio.RXdataBuffer[2];
    crsf.LinkStatistics.uplink_RSSI_2 = 0;
    crsf.LinkStatistics.uplink_SNR = Radio.RXdataBuffer[4];
    crsf.LinkStatistics.uplink_Link_quality = Radio.RXdataBuffer[5];

    crsf.LinkStatistics.downlink_SNR = int(Radio.LastPacketSNR * 10);
    crsf.LinkStatistics.downlink_RSSI = 120 + Radio.LastPacketRSSI;
    crsf.LinkStatistics.downlink_Link_quality = linkQuality;
    //crsf.LinkStatistics.downlink_Link_quality = Radio.currPWR;
    crsf.LinkStatistics.rf_Mode = 4 - ExpressLRS_currAirRate_Modparams->index;

    crsf.TLMbattSensor.voltage = (Radio.RXdataBuffer[3] << 8) + Radio.RXdataBuffer[6];
    #endif
    // XXX enable if not DAG
    // crsf.sendLinkStatisticsToTX();
  }
}

// void ICACHE_RAM_ATTR CheckChannels5to8Change()
// { //check if channels 5 to 8 have new data (switch channels)
//   for (int i = 4; i < 8; i++)
//   {
//     if (crsf.ChannelDataInPrev[i] != crsf.ChannelDataIn[i])
//     {
//       Channels5to8Changed = true;
//     }
//   }
// }

void ICACHE_RAM_ATTR GenerateSyncPacketData()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + SYNC_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = FHSSgetCurrIndex();
  Radio.TXdataBuffer[2] = NonceTX;
  Radio.TXdataBuffer[3] = ((ExpressLRS_currAirRate_Modparams->index & 0b111) << 5) + ((ExpressLRS_currAirRate_Modparams->TLMinterval & 0b111) << 2);
  Radio.TXdataBuffer[4] = UID[3];
  Radio.TXdataBuffer[5] = UID[4];
  Radio.TXdataBuffer[6] = UID[5];
}

// void ICACHE_RAM_ATTR Generate4ChannelData_10bit()
// {
//   uint8_t PacketHeaderAddr;
//   PacketHeaderAddr = (DeviceAddr << 2) + RC_DATA_PACKET;
//   Radio.TXdataBuffer[0] = PacketHeaderAddr;
//   Serial.println("TODO generate OTA packet for DAG");
//   Radio.TXdataBuffer[1] = ((CRSF_to_UINT10(crsf.ChannelDataIn[0]) & 0b1111111100) >> 2);
//   // Radio.TXdataBuffer[2] = ((CRSF_to_UINT10(crsf.ChannelDataIn[1]) & 0b1111111100) >> 2);
//   // Radio.TXdataBuffer[3] = ((CRSF_to_UINT10(crsf.ChannelDataIn[2]) & 0b1111111100) >> 2);
//   // Radio.TXdataBuffer[4] = ((CRSF_to_UINT10(crsf.ChannelDataIn[3]) & 0b1111111100) >> 2);
//   // Radio.TXdataBuffer[5] = ((CRSF_to_UINT10(crsf.ChannelDataIn[0]) & 0b0000000011) << 6) +
//   //                         ((CRSF_to_UINT10(crsf.ChannelDataIn[1]) & 0b0000000011) << 4) +
//   //                         ((CRSF_to_UINT10(crsf.ChannelDataIn[2]) & 0b0000000011) << 2) +
//   //                         ((CRSF_to_UINT10(crsf.ChannelDataIn[3]) & 0b0000000011) << 0);
// }

// void ICACHE_RAM_ATTR Generate4ChannelData_11bit()
// {
//   uint8_t PacketHeaderAddr;
//   PacketHeaderAddr = (DeviceAddr << 2) + RC_DATA_PACKET;
//   Radio.TXdataBuffer[0] = PacketHeaderAddr;
//   Radio.TXdataBuffer[1] = ((crsf.ChannelDataIn[0]) >> 3);
//   Radio.TXdataBuffer[2] = ((crsf.ChannelDataIn[1]) >> 3);
//   Radio.TXdataBuffer[3] = ((crsf.ChannelDataIn[2]) >> 3);
//   Radio.TXdataBuffer[4] = ((crsf.ChannelDataIn[3]) >> 3);
//   Radio.TXdataBuffer[5] = ((crsf.ChannelDataIn[0] & 0b00000111) << 5) +
//                           ((crsf.ChannelDataIn[1] & 0b111) << 2) +
//                           ((crsf.ChannelDataIn[2] & 0b110) >> 1);
//   Radio.TXdataBuffer[6] = ((crsf.ChannelDataIn[2] & 0b001) << 7) +
//                           ((crsf.ChannelDataIn[3] & 0b111) << 4); // 4 bits left over for something else?
// #ifdef One_Bit_Switches
//   Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[4]) << 3;
//   Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[5]) << 2;
//   Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[6]) << 1;
//   Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[7]) << 0;
// #endif
// }

void ICACHE_RAM_ATTR GenerateMSPData()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + MSP_DATA_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = MSPPacket.function;
  Radio.TXdataBuffer[2] = MSPPacket.payloadSize;
  Radio.TXdataBuffer[3] = 0;
  Radio.TXdataBuffer[4] = 0;
  Radio.TXdataBuffer[5] = 0;
  Radio.TXdataBuffer[6] = 0;
  if (MSPPacket.payloadSize <= 4)
  {
    MSPPacket.payloadReadIterator = 0;
    for (int i = 0; i < MSPPacket.payloadSize; i++)
    {
      Radio.TXdataBuffer[3 + i] = MSPPacket.readByte();
    }
  }
  else
  {
    // Serial.println("Unable to send MSP command. Packet too long.");
  }
}

void ICACHE_RAM_ATTR SetRFLinkRate(uint8_t index) // Set speed of RF link (hz)
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen);
  hwTimer.updateInterval(ModParams->interval);

  ExpressLRS_currAirRate_Modparams = ModParams;
  ExpressLRS_currAirRate_RFperfParams = RFperf;

  // crsf.RequestedRCpacketInterval = ModParams->interval;
  isRXconnected = false;

  if (UpdateRFparamReq)
    UpdateRFparamReq = false;

  #ifdef PLATFORM_ESP32
  // updateLEDs(isRXconnected, ExpressLRS_currAirRate_Modparams->TLMinterval);
  
  #ifdef USE_1E
  // update the 1e filters
  double freq = 1000000.0 / ModParams->interval;
  // Serial.print("setting 1e freq to "); Serial.println(freq);
  f_roll.setFrequency(freq);
  f_pitch.setFrequency(freq);
  f_yaw.setFrequency(freq);
  f_throttle.setFrequency(freq);
  #endif // USE_1E

  #endif
}

uint8_t ICACHE_RAM_ATTR decTLMrate()
{
  // Serial.println("dec TLM");
  uint8_t currTLMinterval = (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;

  if (currTLMinterval < (uint8_t)TLM_RATIO_1_2)
  {
    ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)(currTLMinterval + 1);
    // Serial.println(currTLMinterval);
  }
  return (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;
}

uint8_t ICACHE_RAM_ATTR incTLMrate()
{
  // Serial.println("inc TLM");
  uint8_t currTLMinterval = (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;

  if (currTLMinterval > (uint8_t)TLM_RATIO_NO_TLM)
  {
    ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)(currTLMinterval - 1);
  }
  return (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;
}

void ICACHE_RAM_ATTR decRFLinkRate()
{
  // Serial.println("dec RFrate");
  if (ExpressLRS_currAirRate_Modparams->index < (RATE_MAX - 1)) {
    SetRFLinkRate(ExpressLRS_currAirRate_Modparams->index + 1);
  }
}

void ICACHE_RAM_ATTR incRFLinkRate()
{
  // Serial.println("inc RFrate");
  if (ExpressLRS_currAirRate_Modparams->index > 0) {
    SetRFLinkRate(ExpressLRS_currAirRate_Modparams->index - 1);
  }
}

void ICACHE_RAM_ATTR HandleFHSS()
{
  uint8_t modresult = (NonceTX) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

  if (modresult == 0) // if it time to hop, do so.
  {
    Radio.SetFrequency(FHSSgetNextFreq());
  }
}

void ICACHE_RAM_ATTR HandleTLM()
{
  if (ExpressLRS_currAirRate_Modparams->TLMinterval > 0)
  {
    uint8_t modresult = (NonceTX) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    if (modresult != 0)
    {
      // not time yet, so return
      return;
    }
    // wait for tlm response because it's time
    Radio.RXnb();
    WaitRXresponse = true;
  }
}

void ICACHE_RAM_ATTR SendRCdataToRF()
{
#ifdef FEATURE_OPENTX_SYNC
  // crsf.JustSentRFpacket(); // tells the crsf that we want to send data now - this allows opentx packet syncing
#endif

  /////// This Part Handles the Telemetry Response ///////
  if ((uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval > 0)
  {
    uint8_t modresult = (NonceTX) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    if (modresult == 0)
    { // wait for tlm response
      if (WaitRXresponse == true)
      {
        WaitRXresponse = false;
        return;
      }
      else
      {
        NonceTX++;
      }
    }
  }

  uint32_t SyncInterval;

  if (isRXconnected)
  {
    SyncInterval = ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalConnected;
  }
  else
  {
    SyncInterval = ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalDisconnected;
  }

  //if (((millis() > (SyncPacketLastSent + SyncInterval)) && (Radio.currFreq == GetInitialFreq()))) //only send sync when its time and only on channel 0;
  //if ((millis() > ((SyncPacketLastSent + SYNC_PACKET_SEND_INTERVAL_RX_CONN)) && (Radio.currFreq == GetInitialFreq())) || ((isRXconnected == false) && (Radio.currFreq == GetInitialFreq())))
  if ((millis() > (SyncPacketLastSent + SyncInterval)) && (Radio.currFreq == GetInitialFreq()) && ((NonceTX) % ExpressLRS_currAirRate_Modparams->FHSShopInterval == 1)) // sync just after we changed freqs (helps with hwTimer.init() being in sync from the get go)
  {
    GenerateSyncPacketData();
    SyncPacketLastSent = millis();
    ChangeAirRateSentUpdate = true;
    //Serial.println("sync");
    //Serial.println(Radio.currFreq);
  }
  else
  {
    if (MSPPacketSendCount && (millis() > (MSP_PACKET_SEND_INTERVAL + MSPPacketLastSent))) // do the not-null first so that the rest can be short circuited
    {
      GenerateMSPData();
      MSPPacketLastSent = millis();
      MSPPacketSendCount--;
    }
    else
    {
      // copy the current stick positions into the crsf data structure
      crsf.ChannelDataIn[0] = scaleRollData(aud_roll.getCurrentAsInt());
      crsf.ChannelDataIn[1] = scalePitchData(aud_pitch.getCurrentAsInt());
      crsf.ChannelDataIn[2] = scaleThrottleData(aud_throttle.getCurrentAsInt());
      crsf.ChannelDataIn[3] = scaleYawData(aud_yaw.getCurrentAsInt());

      // crsf.ChannelDataIn[0] = scaleRollData(audI_roll.getCurrent());
      // crsf.ChannelDataIn[1] = scalePitchData(audI_pitch.getCurrent());
      // crsf.ChannelDataIn[2] = scaleThrottleData(audI_throttle.getCurrent());
      // crsf.ChannelDataIn[3] = scaleYawData(audI_yaw.getCurrent());

      // debug
      // Serial.print("0 ");
      // Serial.println(crsf.ChannelDataIn[0]);

#if defined HYBRID_SWITCHES_8
      GenerateChannelDataHybridSwitch8(Radio.TXdataBuffer, &crsf, DeviceAddr);
#elif defined SEQ_SWITCHES
      GenerateChannelDataSeqSwitch(Radio.TXdataBuffer, &crsf, DeviceAddr);
#else
      Generate4ChannelData_11bit();
#endif
    }
  }

  ///// Next, Calculate the CRC and put it into the buffer /////
  uint8_t crc = CalcCRC(Radio.TXdataBuffer, 7) + CRCCaesarCipher;
  Radio.TXdataBuffer[7] = crc;
  Radio.TXnb(Radio.TXdataBuffer, 8);



  if (ChangeAirRateRequested)
  {
    ChangeAirRateSentUpdate = true;
  }
}

// void ICACHE_RAM_ATTR ParamUpdateReq()
// {
//   UpdateParamReq = true;

//   if (crsf.ParameterUpdateData[0] == 1)
//   {
//     UpdateRFparamReq = true;
//   }
// }

// void HandleUpdateParameter()
// {

//   if (UpdateParamReq == false)
//   {
//     return;
//   }

//   switch (crsf.ParameterUpdateData[0])
//   {
//   case 0: // send all params
//     Serial.println("send all lua params");
//     break;

//   case 1:
//   Serial.println("Link rate");
//     if (crsf.ParameterUpdateData[1] == 0)
//     {
//       decRFLinkRate();
//     }
//     else if (crsf.ParameterUpdateData[1] == 1)
//     {
//       incRFLinkRate();
//     }
//     Serial.println(ExpressLRS_currAirRate_Modparams->enum_rate);
//     break;

//   case 2:

//     if (crsf.ParameterUpdateData[1] == 0)
//     {
//       decTLMrate();
//     }
//     else if (crsf.ParameterUpdateData[1] == 1)
//     {
//       incTLMrate();
//     }

//     break;

//   case 3:

//     if (crsf.ParameterUpdateData[1] == 0)
//     {
//       Serial.println("Decrease RF power");
//       POWERMGNT.decPower();
//     }
//     else if (crsf.ParameterUpdateData[1] == 1)
//     {
//       Serial.println("Increase RF power");
//       POWERMGNT.incPower();
//     }

//     break;

//   case 4:

//     break;

//   case 0xFF:
//     if (crsf.ParameterUpdateData[1] == 1)
//     {
//       Serial.println("Binding Requested!");
//       crsf.sendLUAresponse((uint8_t)0xFF, (uint8_t)0x01, (uint8_t)0x00, (uint8_t)0x00);

//       //crsf.sendLUAresponse((uint8_t)0xFF, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00); // send this to confirm binding is done
//     }
//     break;

//   default:
//     break;
//   }

//   UpdateParamReq = false;
//   //Serial.println("Power");
//   //Serial.println(POWERMGNT.currPower());
//   crsf.sendLUAresponse((ExpressLRS_currAirRate_Modparams->enum_rate + 3), ExpressLRS_currAirRate_Modparams->TLMinterval + 1, POWERMGNT.currPower() + 2, 4);
// }

void ICACHE_RAM_ATTR RXdoneISR()
{
  // Ideally this should just copy the data from the radio buffer and then
  // notify a telem task to do the processing.
  ProcessTLMpacket();
}

void ICACHE_RAM_ATTR TXdoneISR()
{
  NonceTX++; // must be done before callback
  HandleFHSS();
  HandleTLM();

  // is there a pending rate change?
  if (nextRFLinkRate >= 0) {
    SetRFLinkRate((uint8_t)nextRFLinkRate);
    isRXconnected = false;
    LastTLMpacketRecvMillis = 0; // kick the tx into reconnecting straight away
    nextRFLinkRate = -1; // so that the button can be used again
    #ifdef USE_TFT
    lcdNeedsRedraw = true; // trigger a screen update to show the new rate
    #endif
    sendStateNeeded = true; // update will be done from loop
  }

  // is there a pending power change?
  if (nextRadioPower != NONE_PENDING) {
    Radio.SetOutputPower(nextRadioPower);
    // Serial.print("power set to ");Serial.println(nextRadioPower);
    nextRadioPower = NONE_PENDING; // so that the button can be used again
    #ifdef USE_TFT
    lcdNeedsRedraw = true; // trigger a screen update to show the new rate
    #endif
    sendStateNeeded = true; // update will be done from loop
  }

  RadioIsIdle = true;
}

#ifdef USE_TFT
void redrawDisplay()
{
  tft.fillScreen(TFT_BLUE);

  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.setTextFont(2);

  tft.drawString("ExpressLRS", TFT_WIDTH/2, 0);

  tft.setTextDatum(TL_DATUM);

  float pmw = pow10(((float)Radio.currPWR)/10.0f);

  // Serial.print(Radio.currPWR); Serial.print(" "); Serial.println(pmw);

  tft.setCursor(0, 35, 2);
  tft.print("P ");
  tft.setTextFont(4);
  tft.printf("%3.2f mW\n", pmw);

  uint32_t freq = 1000000 / ExpressLRS_currAirRate_Modparams->interval;
  // sprintf(buffer, "R: %4d Hz", freq);
  // tft.println(buffer);
  tft.setTextFont(2);
  tft.print("F ");
  tft.setTextFont(4);
  tft.printf("%4d Hz\n", freq);
  
  // TODO add display for telem interval

}

/** Check that the throttle is low and switches are off
 *  Used to prevent radio transmission at startup in an unsafe state
 */
void startupSafetyCheck()
{
  const int SAFE_THROTTLE_LIMIT = 200;
  bool throttleSafe;
  bool anySwitchSet;
  do {
    // need to call multiple times or the LPFs take ages to settle
    for(int i=0; i<100; i++) {
      refreshGimbalData();
    }
    // throttle is channel 2
    throttleSafe = (crsf.ChannelDataIn[2] < SAFE_THROTTLE_LIMIT);

    anySwitchSet = false;
    for(int s=0; s<4; s++) {
      if (crsf.currentSwitches[s] != 0) {
        anySwitchSet = true;
        break;
      }
    }

    if (!throttleSafe || anySwitchSet) {
      #ifdef USE_TFT
      tft.fillScreen(TFT_RED);
      tft.setTextDatum(TC_DATUM);
      tft.setTextColor(TFT_WHITE);
      tft.setTextFont(4);

      if (anySwitchSet) {
        tft.drawString("SWITCHES!", TFT_WIDTH/2, 20);
      }

      if (!throttleSafe){
        tft.drawString("THROTTLE!", TFT_WIDTH/2, 100);
      }
      #endif // USE_TFT

      // give the user time to do something about it
      delay(100);
    }

  } while (!throttleSafe || anySwitchSet);

  // tft.fillScreen(TFT_BLUE);
}

#endif // USE_TFT

/**
 * Called from loop() when there is data available on the serial port
 */
void readFromCoPro(bool eyeCatcherAlreadySeen = false)
{
  // read bytes from serial port until it is empty, building up commands and executing them
  while(Serial1.available()) {
    int c = 0;
    if (!eyeCatcherAlreadySeen) {
      c = Serial1.read();
    } else {
      // make sure we only skip the read the first time around the loop
      eyeCatcherAlreadySeen = false;
      c = PKT_EYECATCHER;
    }
    if (c == PKT_EYECATCHER) {
      
      // read the command
      c = Serial1.read();
      switch(c) {
        default:
          Serial.printf("unsupported command %02x\n", c);
          break;
        case -1: // nothing was available
          Serial.println("readFromCoPro incomplete packet");
          return; // give up on this packet
          break;
        case PKT_SAVE_CONFIG:
          // ignore if the radio is armed
          if (crsf.currentSwitches[0] != 0) {
            Serial.println("ignoring save, armed");
          } else {
            // defer the change to loop()
            saveNeeded = true;
          }
          break;
        case PKT_SWITCH_STATE:
        {
          switchState_u switches;
          switches.dataBytes[0] = PKT_EYECATCHER; // seems daft, but we need it for the crc
          switches.dataBytes[1] = c; // also needed for crc
          // get the switch data byte
          c = Serial1.read();
          if (c == -1) {
            // ran out of data - maybe worth trying harder?
            Serial.println("no switch data");
            return;
          } else {
            switches.dataBytes[2] = c;
          }
          // get the crc
          uint8_t pktCRC = Serial1.read();
          if (pktCRC == -1) {
            // ran out of data - maybe worth trying harder?
            Serial.println("no crc on switch packet");
            return;
          }
          // calculate the expected crc and compare with the sent one
          uint8_t expectedCRC = CalcCRC(switches.dataBytes, sizeof(switches)-1);
          if (pktCRC != expectedCRC) {
            Serial.printf("crc mismatch, expected %02X, actual %02X\n", expectedCRC, switches.switchState.crc);
            return;
          }
          // we have switches - put the data somewhere useful!
          // Serial.printf("Yay switches: %d %d %d %d\n", switches.switchState.aux1, switches.switchState.aux2, switches.switchState.aux3, switches.switchState.aux4);

          // digitalWrite(GPIO_PIN_LED, switches.switchState.aux1?1:0); // for Testing

          crsf.currentSwitches[0] = switches.switchState.aux1;
          crsf.currentSwitches[1] = switches.switchState.aux2;
          crsf.currentSwitches[2] = switches.switchState.aux3;
          crsf.currentSwitches[3] = switches.switchState.aux4;

          break;
        }
        case PKT_PARAM_CHANGE:
        {
          paramChangePkt_u pktU;
          pktU.paramChangePkt.eyecatcher = PKT_EYECATCHER;
          pktU.paramChangePkt.packetType = PKT_PARAM_CHANGE;
          // get the data byte
          c = Serial1.read();
          if (c == -1) {
            // ran out of data - maybe worth trying harder?
            Serial.println("no body");
            return;
          } else {
            pktU.dataBytes[2] = c;
          }
          // get the crc
          uint8_t pktCRC = Serial1.read();
          if (pktCRC == -1) {
            // ran out of data - maybe worth trying harder?
            Serial.println("no crc on param packet");
            return;
          }
          // calculate the expected crc and compare with the sent one
          uint8_t expectedCRC = CalcCRC(pktU.dataBytes, sizeof(paramChangePkt_u)-1);
          if (pktCRC != expectedCRC) {
            Serial.printf("crc mismatch, expected %02X, actual %02X\n", expectedCRC, pktCRC);
            return;
          }
          // ignore param changes in the radio is armed
          if (crsf.currentSwitches[0] != 0) {
            Serial.println("ignoring param change, armed");
            break;
          }
          paramChange_t *pC = &pktU.paramChangePkt.payload;
          uint param = pC->param;
          uint dir = pC->direction;
          switch (param) {
            case PARAM_POWER:
            {
              int newPower = Radio.currPWR;
              if (dir == PARAM_INC) {
                newPower++;
              } else {
                newPower--;
              }
              nextRadioPower = newPower;
              // Radio.SetOutputPower(newPower);
              // sendTxStateToCoPro();
              break;
            }
            case PARAM_RATE:
            {
              // defer the changes to the txDoneISR
              if (dir == PARAM_INC) {
                if (ExpressLRS_currAirRate_Modparams->index > 0) {
                  nextRFLinkRate = ExpressLRS_currAirRate_Modparams->index - 1;
                }
              } else {
                if (ExpressLRS_currAirRate_Modparams->index < (RATE_MAX - 1)) {
                  nextRFLinkRate = ExpressLRS_currAirRate_Modparams->index + 1;
                }
              }
              break;
            }
            case PARAM_TLM_INT:
            {
              if (dir == PARAM_INC) {
                incTLMrate();
              } else {
                decTLMrate();
              }
              sendTxStateToCoPro(); // get the new data on the display
              break;
            }

            default:
              Serial.printf("unimpl param %d\n", param);
          }
          break;
        }
      }
    } // if eyecatcher
  } // while data available
}

#ifndef USE_DMA_ADC

/** task for reading the ADCs and filtering the gimbal data
 * Will be triggered from the timer callback
 */
void readADCsTask(void *pvParameters)
{
  for(;;) {
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    unsigned long start = micros();
    refreshGimbalData();
    tADC = micros() - start;

    cpuLoad = LPF_cpuLoad.update((tRF + tADC) * 100 / ExpressLRS_currAirRate_Modparams->interval);
  }
}

#endif // not USE_DMA_ADC

/** task for reading from the UI co-processor
 * 
 * Depends on the following addition to HardwareSerial.cpp:
 *
 * Wait for up to timeout ms for a byte of data
 * NB if timed out, returns 0 which is indistinguishable from a valid byte
  int HardwareSerial::blockingRead(uint32_t timeout)
  {
      return uartBlockingRead(_uart, timeout);
  }
 * */
void readFromCoProTask(void *pvParameters)
{
  #define RD_BUF_SIZE 100
  // uart_event_t event;
  // uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
  for(;;) {
    // Serial.println("top of uart reading task loop");
    // uart_write_bytes(UART_NUM_0, "top loop\n", 9);

    // NB if timeout, will return 0 which is indistinguishable from a valid byte.
    // Fortunately we can dump bytes until we find the (non-zero) eye catcher anyway.
    // NB THIS NEEDS A HACKED VERSION OF THE SERIAL LIBRARY
    int c = Serial1.blockingRead(5000);

    // Serial.printf("blockingRead returned %x\n", c);
    if (c == PKT_EYECATCHER) {
      const bool alreadyGotEyecatcher = true;
      readFromCoPro(alreadyGotEyecatcher);
    }

    // int bytesRead = uart_read_bytes(UART_NUM_1, dtmp, RD_BUF_SIZE, 5000);
    // stuff
    // Serial.printf("read %d bytes\n", bytesRead);
    // char pbuf[80];
    // sprintf(pbuf, "got %d bytes\n", bytesRead);
    // uart_write_bytes(UART_NUM_0, pbuf, strlen(pbuf));


    //Waiting for UART event.
    // if(xQueueReceive(copro_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) 
    // {
    //   bzero(dtmp, RD_BUF_SIZE);
    //   switch(event.type) {
    //     //Event of UART receving data
    //     /*We'd better handler data event fast, there would be much more data events than
    //     other types of events. If we take too much time on data event, the queue might
    //     be full.*/
    //     case UART_DATA:
    //       uart_read_bytes(UART_NUM_2, dtmp, event.size, portMAX_DELAY);
    //       // stuff
    //       Serial.printf("read %d bytes\n", event.size);
    //       break;
    //     default:
    //       Serial.printf("unhandled event %d\n", event.type);
    //       break;
    //   }
    // }
  }

}


void setup()
{
#ifdef PLATFORM_ESP32
  // Serial.begin(115200);
  // Serial.begin(460800);
  Serial.begin(921600);
  #ifdef USE_UART2
    #error uart2 is being used by the copro
    // Serial2.begin(400000);
  #endif

  pinMode(GPIO_PIN_LED, OUTPUT);
  digitalWrite(GPIO_PIN_LED, 1);

#endif

#if defined(TARGET_R9M_TX) || defined(TARGET_R9M_LITE_TX)

    pinMode(GPIO_PIN_LED_GREEN, OUTPUT);
    pinMode(GPIO_PIN_LED_RED, OUTPUT);
    digitalWrite(GPIO_PIN_LED_GREEN, HIGH);

#ifdef USE_ESP8266_BACKPACK
    HardwareSerial(USART1);
    Serial.begin(460800);
#else
    HardwareSerial(USART2);
    Serial.begin(400000);
#endif

#if defined(TARGET_R9M_TX)
    // Annoying startup beeps
#ifndef JUST_BEEP_ONCE
  pinMode(GPIO_PIN_BUZZER, OUTPUT);
  const int beepFreq[] = {659, 659, 659, 523, 659, 783, 392};
  const int beepDurations[] = {150, 300, 300, 100, 300, 550, 575};

  for (int i = 0; i < 7; i++)
  {
    tone(GPIO_PIN_BUZZER, beepFreq[i], beepDurations[i]);
    delay(beepDurations[i]);
    noTone(GPIO_PIN_BUZZER);
  }
  #else
  tone(GPIO_PIN_BUZZER, 400, 200);
  delay(200);
  tone(GPIO_PIN_BUZZER, 480, 200);
  #endif
  button.init(GPIO_PIN_BUTTON, true); // r9 tx appears to be active high
  R9DAC.init();
#endif

#endif

#ifdef PLATFORM_ESP32
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector needed for debug, shouldn't need to be actually used in practise.
  // strip.Begin();
  // Get base mac address
  // esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  // Print base mac address
  // This should be copied to common.h and is used to generate a unique hop sequence, DeviceAddr, and CRC.
  // UID[0..2] are OUI (organisationally unique identifier) and are not ESP32 unique.  Do not use!
  // Serial.println("");
  // Serial.println("Copy the below line into common.h.");
  // Serial.print("uint8_t UID[6] = {");
  // Serial.print(baseMac[0]);
  // Serial.print(", ");
  // Serial.print(baseMac[1]);
  // Serial.print(", ");
  // Serial.print(baseMac[2]);
  // Serial.print(", ");
  // Serial.print(baseMac[3]);
  // Serial.print(", ");
  // Serial.print(baseMac[4]);
  // Serial.print(", ");
  // Serial.print(baseMac[5]);
  // Serial.println("};");
  // Serial.println("");

  // init the display
  #ifdef USE_TFT
  tft.init();
  tft.setRotation(2);
  #endif

  #ifdef USE_IO_COPRO
  // setup serial port
  Serial1.begin(460800, SERIAL_8N1, GPIO_PIN_COPRO_RX, GPIO_PIN_COPRO_TX);
  // uart_config_t uart_config = {
  //   .baud_rate = 460800,
  //   .data_bits = UART_DATA_8_BITS,
  //   .parity = UART_PARITY_DISABLE,
  //   .stop_bits = UART_STOP_BITS_1,
  //   .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  //   .rx_flow_ctrl_thresh = 00, // hopefully doesn't matter
  //   .use_ref_tick = false, 
  // };
  //Install UART driver, and get the queue.
  // int res = uart_driver_install(UART_NUM_1, 1024, 1024, 0, NULL, 0);
  // Serial.printf("driver_install: %d\n", res);
  // res = uart_param_config(UART_NUM_1, &uart_config);
  // Serial.printf("param_config: %d\n", res);

  // Set UART log level
  // esp_log_level_set(TAG, ESP_LOG_INFO);
  // Set UART pins (using UART0 default pins ie no changes.)
  // is this needed?
  // uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, 23, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Set uart pattern detect function.
  // uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
  // Reset the pattern queue length to record at most 20 pattern positions.
  // uart_pattern_queue_reset(EX_UART_NUM, 20);

  // uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0);
  // uart_param_config(UART_NUM_0, &uart_config);

  // uart_write_bytes(UART_NUM_0, "no 5 is alive!\n", 15);

  // // start a task to listen for updates from the co-pro
  xTaskCreatePinnedToCore(readFromCoProTask, "readCoProTask", 3000, NULL, 10, &readCoProTask, 1);

  #else // USE_IO_COPRO

  // switches
  pinMode(GPIO_AUX1, INPUT_PULLUP);
  pinMode(GPIO_AUX2, INPUT_PULLUP);
  pinMode(GPIO_AUX3, INPUT_PULLUP);
  pinMode(GPIO_AUX4, INPUT_PULLUP);

  // ttgo buttons
  pinMode(GPIO_BUTTON1, INPUT);
  pinMode(GPIO_BUTTON2, INPUT);

  #endif // USE_IO_COPRO

#endif

  FHSSrandomiseFHSSsequence();

#if defined Regulatory_Domain_AU_915 || defined Regulatory_Domain_EU_868 || defined Regulatory_Domain_FCC_915
#ifdef Regulatory_Domain_EU_868
  Serial.println("Setting 868MHz Mode");
#else
  Serial.println("Setting 915MHz Mode");
#endif
#elif defined Regulatory_Domain_AU_433 || defined Regulatory_Domain_EU_433
  Serial.println("Setting 433MHz Mode");
#endif

  Radio.RXdoneCallback = &RXdoneISR;
  Radio.TXdoneCallback = &TXdoneISR;

#ifndef One_Bit_Switches
  crsf.RCdataCallback1 = &CheckChannels5to8Change;
#endif
  // crsf.connected = &hwTimer.resume; // it will auto init when it detects UART connection
  // crsf.disconnected = &hwTimer.stop;
  // crsf.RecvParameterUpdate = &ParamUpdateReq;
  hwTimer.callbackTock = &TimerCallbackISR;

  // Serial.println("ExpressLRS TX Module Booted...");


  POWERMGNT.init();
  Radio.currFreq = GetInitialFreq(); //set frequency first or an error will occur!!!

  delay(100); // small delay to give the copro and radio moduletime to startup

  bool result = EEPROM.begin(64);
  Serial.print("begin returned "); Serial.println(result);

  // TODO read & validate the default settings
  defaultSettings_t savedDefaults;
  EEPROM.get(0, savedDefaults);

  Serial.printf("defaults: %u %d %u (crc: %u)\n", savedDefaults.airRateIndex, savedDefaults.prePApower, savedDefaults.tlmInterval, savedDefaults.crc);

  uint8_t linkRateIndex = RATE_DEFAULT;
  int16_t tlmInterval = -1;

  uint8_t expectedCRC = CalcCRC((uint8_t *)&savedDefaults, sizeof(defaultSettings_t)-1); // don't include the crc in the new crc calc
  if (expectedCRC == savedDefaults.crc) {
    Serial.println("saved defaults are good");
    // apply the defaults to the radio settings
    radioPower = savedDefaults.prePApower;
    linkRateIndex = savedDefaults.airRateIndex;
    tlmInterval = savedDefaults.tlmInterval;
  } else {
    Serial.println("saved defaults bad crc");
  }


  Radio.Begin();
  //Radio.SetSyncWord(UID[3]);
  // POWERMGNT.setDefaultPower();
  Radio.SetOutputPower(radioPower);

  SetRFLinkRate(linkRateIndex);
  
  if (tlmInterval != -1) {
    ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)tlmInterval;
  }

  // TODO config
  // crsf.Begin();

  // TODO rethink for co-pro
  // startupSafetyCheck();

  // setup the ADC channels:
  initADC();

  // start a task to read the gimbals
  #ifdef USE_DMA_ADC
  xTaskCreatePinnedToCore(readGimbalsViaDMA_task, "DMA_ADCsTask", 3000, NULL, 20, &readGimbalsDMATaskHandle, 1);
  #else
  xTaskCreatePinnedToCore(readADCsTask, "readADCsTask", 3000, NULL, 20, &readADCsTaskHandle, 1);
  #endif

  hwTimer.init();
  // hwTimer.stop(); //comment to automatically start the timer and leave it running

  sendTxStateToCoPro();

  // Serial.println("setup finished\n");
}

void loop()
{
  // static uint32_t lastDebugOutput = 0;
  static uint32_t lastTelemSend = 0;

  // TODO add a timeout so we can cancel the save if something goes wrong
  if (saveNeeded && RadioIsIdle) {
    saveSettingsAsDefaults();
    saveNeeded = false;
  }

  if (sendStateNeeded) {
    sendTxStateToCoPro();
    sendStateNeeded = false;
  }

  unsigned long now = millis();

  // if (now > (lastDebugOutput + 1000)) {
  //   Serial.print("cpu ");
  //   Serial.println(cpuLoad);
  //   Serial.printf("R %u P %u T %u Y %u\n", crsf.ChannelDataIn[0], crsf.ChannelDataIn[1], crsf.ChannelDataIn[2], crsf.ChannelDataIn[3]);
  //   // Serial.printf("nGroups %u, sps %lu\n", nADCgroups, nADCgroups * GROUP_SAMPLE_LENGTH * 1000 / (now-lastDebugOutput));
  //   // nADCgroups = 0;
  //   lastDebugOutput = now;
  // }

  // TODO this is for testing, we'll probably send telem to copro everytime we get it from the rx
  if (now > (lastTelemSend + 500)) {
    lastTelemSend = now;
    // Serial.println("sending test telem");
    // send a packet to the copro for testing
    rxTelemPkt_u pktU;
    rxTelemPkt_t *pkt = &pktU.rxTelemPkt;

    pkt->eyecatcher = PKT_EYECATCHER;
    pkt->packetType = PKT_TELEM;
    pkt->payload.rssi_dBm = crsf.LinkStatistics.uplink_RSSI_1;
    pkt->payload.snr = crsf.LinkStatistics.uplink_SNR;
    pkt->payload.lq = crsf.LinkStatistics.uplink_Link_quality;
    pkt->payload.cpu = cpuLoad;

    pkt->crc = CalcCRC(pktU.dataBytes, sizeof(rxTelemPkt_t)-1);

    Serial1.write(pktU.dataBytes, sizeof(rxTelemPkt_t));

  }


  // static int button1state = 1;
  // static int button2state = 1;

  // static unsigned long b1LastChange = 0;
  // static unsigned long b2LastChange = 0;

  // buttons read 0 when pressed
  // // button 1 for packet rate
  // int b1 = digitalRead(GPIO_BUTTON1);
  // if (b1 != button1state && millis() > (b1LastChange + 100)) {
  //   // record the new state
  //   button1state = b1;
  //   b1LastChange = millis();
  //   if (b1 == 0 && nextRFLinkRate < 0) {
  //     // don't do the change here or the conflict with the radio can crash the tx
  //     // set the variable which will trigger the change at the next opportunity
  //     nextRFLinkRate = (ExpressLRS_currAirRate_Modparams->index + 1) % RATE_MAX;
  //   }
  // }

  // // button 2 for power
  // int b2 = digitalRead(GPIO_BUTTON2);
  // if (b2 != button2state && millis() > (b2LastChange + 100)) {
  //   // record the new state
  //   button2state = b2;
  //   b2LastChange = millis();
  //   if (b2 == 0) {
  //     nextRadioPower = radioPower + 1;
  //     if (nextRadioPower > MAX_PRE_PA_POWER) {
  //       nextRadioPower = MIN_PRE_PA_POWER;
  //     }
  //     radioPower = nextRadioPower;
  //   }
  // }


  // while(UpdateParamReq){
  //   HandleUpdateParameter();
  // }

#ifdef FEATURE_OPENTX_SYNC
  // Serial.println(crsf.OpenTXsyncOffset);
#endif

  #ifdef USE_TFT
  if (millis() > (lastDebugOutput + 500)) {  // TODO not really debug anymore, change name
    lastDebugOutput = millis();

    if (lcdNeedsRedraw) {
      redrawDisplay();
      lcdNeedsRedraw = false;
    }

    // Serial.print("rssi "); Serial.print((int8_t)crsf.LinkStatistics.uplink_RSSI_1);
    // Serial.print(" SNR "); Serial.print(crsf.LinkStatistics.uplink_SNR);
    // Serial.print(" LQ "); Serial.println(crsf.LinkStatistics.uplink_Link_quality);
    // Serial.print(" telem received "); Serial.print(telemReceived);
    // Serial.print(" telem lost "); Serial.println(telemExpected - telemReceived);

    tft.setCursor(0, 100, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);

    tft.print("RSSI ");
    tft.setTextFont(4);
    if (isRXconnected) {
      tft.printf("%3d \n", (int8_t)crsf.LinkStatistics.uplink_RSSI_1);
    } else {
      tft.println(" --    ");
    }

    tft.setTextFont(2);
    tft.print("SNR  ");
    tft.setTextFont(4);
    if (isRXconnected) {
      tft.printf("%3d \n", crsf.LinkStatistics.uplink_SNR);
    } else {
      tft.println(" --    ");
    }

    tft.setTextFont(2);
    tft.print("LQ   ");
    tft.setTextFont(4);
    if (isRXconnected) {
      tft.printf("%3d  \n", crsf.LinkStatistics.uplink_Link_quality);
     } else {
      tft.println(" --    ");
    }

    // if armEndTime isn't set then show a running timer, otherwise show the elapsed last armed time
    char *pstr;
    char buffer[16];
    if (armStartTime == 0) {
      // tft.println("    00:00  ");
      pstr = (char*)"00:00"; // promise not to overwrite the constant
    } else {
      pstr = buffer;
      unsigned long eTime = (armEndTime == 0) ? millis() : armEndTime;
      unsigned int armTime = (eTime - armStartTime) / 1000;
      unsigned int minutes = armTime / 60;
      unsigned int seconds = armTime % 60;
      sprintf(pstr, "%02d:%02d", minutes, seconds);
    }

    tft.setTextDatum(TC_DATUM);
    tft.drawString(pstr, TFT_WIDTH/2, 185);

    tft.setTextDatum(TL_DATUM);
    tft.setCursor(0,220,4);

    #define BAT_OVERSAMPLE 4
    int batRaw = 0;
    for(int i=0; i<BAT_OVERSAMPLE; i++) {
      int sample;
      adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_BIT_12, &sample);
      batRaw += sample;
    }
    batRaw = LPF_battery.update(batRaw/BAT_OVERSAMPLE);

    // scale must match width and atten
    const float scale = 7.885 / 4095; // (resistor divider factor * max atten voltage * fudgefactor) / max_reading
    float vBat = (float) batRaw * scale;

    tft.printf("%2.2fV", vBat);

    tft.setTextFont(1);
    sprintf(buffer, "cpu: %2d%%", cpuLoad);
    tft.setTextDatum(BR_DATUM);
    tft.drawString(buffer, TFT_WIDTH, TFT_HEIGHT);

    tft.setTextDatum(TL_DATUM);

    // int a1 = digitalRead(GPIO_AUX1);
    // int a2 = digitalRead(GPIO_AUX2);

    // int b1 = digitalRead(GPIO_BUTTON1);
    // int b2 = digitalRead(GPIO_BUTTON2);

    // tft.printf("A1 %d A2 %d B1 %d B2 %d", a1, a2, b1, b2);

  }
  #endif // USE_TFT


  if (now > (RX_CONNECTION_LOST_TIMEOUT + LastTLMpacketRecvMillis))
  {
    if (isRXconnected) {
      digitalWrite(GPIO_PIN_LED, 0);
    }
    isRXconnected = false;
    #if defined(TARGET_R9M_TX) || defined(TARGET_R9M_LITE_TX)
    digitalWrite(GPIO_PIN_LED_RED, LOW);
    #endif
  }
  else
  {
    if (!isRXconnected) {
      digitalWrite(GPIO_PIN_LED, 1);
    }
    isRXconnected = true;
    #if defined(TARGET_R9M_TX) || defined(TARGET_R9M_LITE_TX)
    digitalWrite(GPIO_PIN_LED_RED, HIGH);
    #endif
  }

  // float targetFrameRate = (ExpressLRS_currAirRate_Modparams->rate * (1.0 / TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval)));
  // PacketRateLastChecked = millis();
  // PacketRate = (float)packetCounteRX_TX / (float)(PACKET_RATE_INTERVAL);
  // linkQuality = int((((float)PacketRate / (float)targetFrameRate) * 100000.0));

  if (linkQuality > 99)
  {
    linkQuality = 99;
  }
  packetCounteRX_TX = 0;

#if defined(TARGET_R9M_TX) || defined(TARGET_R9M_LITE_TX)
  crsf.STM32handleUARTin();
  #ifdef FEATURE_OPENTX_SYNC
  crsf.sendSyncPacketToTX();
  #endif
  crsf.UARTwdt();
  #ifdef TARGET_R9M_TX
  button.handle();
  #endif
#endif

// #ifdef PLATFORM_ESP32
//   if (Serial2.available())
//   {
//     uint8_t c = Serial2.read();
// #else
//   if (Serial.available())
//   {
//     uint8_t c = Serial.read();
// #endif

//     if (msp.processReceivedByte(c))
//     {
//       // Finished processing a complete packet
//       ProcessMSPPacket(msp.getReceivedPacket());
//       msp.markPacketReceived();
//     }
//   }

  delay(10); // stop this method from eating all available cpu
}

void ICACHE_RAM_ATTR TimerCallbackISR()
{
  // static unsigned long lastDebug = 0;
  unsigned long start = micros();

  if (!UpdateRFparamReq && !saveNeeded)
  {
    RadioIsIdle = false;
    SendRCdataToRF(); // comment out for testing without radio
  }
  else
  {
    NonceTX++;
  }

  tRF = micros() - start;

  // unsigned long adc_start = micros();
  // refreshGimbalData();
  // unsigned long adc_end = micros();

  // unsigned long tRF = adc_start - start;
  // unsigned long tADC = adc_end - adc_start;

  // totalADC += tADC;
  // if (tADC > maxADC) {
  //   maxADC = tADC;
  // } else if (tADC < minADC) {
  //   minADC = tADC;
  // }

  // not really cpu load, more like timing window ustilisation
  // cpuLoad =  LPF_cpuLoad.update((tADC+tRF)*100/ExpressLRS_currAirRate_Modparams->interval);

  // unsigned long now = adc_end/1000;
  // if (now > (lastDebug + 5000)) {
  //   lastDebug = now;
  //   Serial.print("tRF ");    Serial.print(tRF);
  //   Serial.print(", tADC "); Serial.print(tADC);
  //   Serial.print(", total ");Serial.print(tADC+tRF);
  //   Serial.print(" cpu ");   Serial.println(cpuLoad);
  // }

  #ifndef USE_DMA_ADC
  // notify the ADC task to read the gimbal data
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  vTaskNotifyGiveFromISR(readADCsTaskHandle, &xHigherPriorityTaskWoken );

  // tLeaveISR = micros();
  portYIELD_FROM_ISR();
  #endif // USE_DMA_ADC
}


void OnRFModePacket(mspPacket_t *packet)
{
  // Parse the RF mode
  uint8_t rfMode = packet->readByte();
  CHECK_PACKET_PARSING();

  switch (rfMode)
  {
  case RATE_200HZ:
    SetRFLinkRate(RATE_200HZ);
    break;
  case RATE_100HZ:
    SetRFLinkRate(RATE_100HZ);
    break;
  case RATE_50HZ:
    SetRFLinkRate(RATE_50HZ);
    break;
  default:
    // Unsupported rate requested
    break;
  }
}

#ifdef NOPE
void OnTxPowerPacket(mspPacket_t *packet)
{
  // Parse the TX power
  uint8_t txPower = packet->readByte();
  CHECK_PACKET_PARSING();
  Serial.println("TX setpower");

  switch (txPower)
  {
  case PWR_10mW:
    POWERMGNT.setPower(PWR_10mW);
    break;
  case PWR_25mW:
    POWERMGNT.setPower(PWR_25mW);
    break;
  case PWR_50mW:
    POWERMGNT.setPower(PWR_50mW);
    break;
  case PWR_100mW:
    POWERMGNT.setPower(PWR_100mW);
    break;
  case PWR_250mW:
    POWERMGNT.setPower(PWR_250mW);
    break;
  case PWR_500mW:
    POWERMGNT.setPower(PWR_500mW);
    break;
  case PWR_1000mW:
    POWERMGNT.setPower(PWR_1000mW);
    break;
  case PWR_2000mW:
    POWERMGNT.setPower(PWR_2000mW);
    break;
  default:
    // Unsupported power requested
    break;
  }
}
#endif

void OnTLMRatePacket(mspPacket_t *packet)
{
  // Parse the TLM rate
  // uint8_t tlmRate = packet->readByte();
  // CHECK_PACKET_PARSING();

  // TODO: Implement dynamic TLM rates
  // switch (tlmRate) {
  // case TLM_RATIO_NO_TLM:
  //   break;
  // case TLM_RATIO_1_128:
  //   break;
  // default:
  //   // Unsupported rate requested
  //   break;
  // }
}

void ProcessMSPPacket(mspPacket_t *packet)
{
  // Inspect packet for ELRS specific opcodes
  if (packet->function == MSP_ELRS_FUNC)
  {
    uint8_t opcode = packet->readByte();

    CHECK_PACKET_PARSING();

    switch (opcode)
    {
    case MSP_ELRS_RF_MODE:
      OnRFModePacket(packet);
      break;
    case MSP_ELRS_TX_PWR:
      // OnTxPowerPacket(packet);
      break;
    case MSP_ELRS_TLM_RATE:
      OnTLMRatePacket(packet);
      break;
    default:
      break;
    }
  }
  else if (packet->function == MSP_SET_VTX_CONFIG)
  {
    MSPPacket = *packet;
    MSPPacketSendCount = 6;
  }
}
