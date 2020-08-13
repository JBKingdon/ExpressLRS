#include <Arduino.h>
#include "FIFO.h"
#include "utils.h"
#include "common.h"
#include <cmath>
#include "LowPassFilter.h"

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
#include "SX127xDriver.h"
SX127xDriver Radio;
#elif defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)
#include "SX1280Driver.h"
SX1280Driver Radio;
static int radioPower = MAX_PRE_PA_POWER;
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
#include <TFT_eSPI.h>
#include "OneEuroFilter.h"
#endif

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

uint8_t baseMac[6];

// ===================================
// variables for direct attached gimbals

TFT_eSPI tft = TFT_eSPI();

static bool lcdNeedsRedraw = true;

static volatile uint16_t cpuLoad;
LPF LPF_cpuLoad(3,1);

// variables for checking ADC performance
static unsigned long minADC=99999, maxADC=0, totalADC=0;

// time when arm switch is enabled/disabled
static unsigned long armStartTime = 0;
static unsigned long armEndTime = 0;

// channel LPFs
LPF LPF_pitch(4,1);
LPF LPF_roll(4,1);
LPF LPF_throttle(4,1);
LPF LPF_yaw(4,1);

// and the battery sensor
LPF LPF_battery(3);

// testing the 1e filter
double frequency = 800 ; // Hz - is set from setRFLinkRate, so this value isn't important
double mincutoff = 0.35 ; // min=.4, b=.06 looks pretty good on bench test. Maybe try slightly lower min & higher beta?
double beta = 0.06 ;     
double dcutoff = 1.0 ;   // orig desc: this one should be ok

OneEuroFilter f_roll(frequency, mincutoff, beta, dcutoff);
OneEuroFilter f_pitch(frequency, mincutoff, beta, dcutoff);
OneEuroFilter f_throttle(frequency, mincutoff, beta, dcutoff);
OneEuroFilter f_yaw(frequency, mincutoff, beta, dcutoff);

// -----------------------------------

void initADC()
{
  adc1_config_width(ADC_WIDTH_BIT_12);

  // TODO convert from the target PIN numbers to ADC channels
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_6);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_6);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);

  // battery sensor
  adc2_config_channel_atten(ADC2_CHANNEL_4, ADC_ATTEN_6db);

  // init the battery LPF
  delay(10);
  int batRaw;
  for(int i=0; i<200; i++) {
    adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_BIT_12, &batRaw);
    LPF_battery.update(batRaw);
  }

}

/**
 * reading the gimbals
 * 
 * POC version, simplistic reading strategy. Investigate more efficient ADC handling
*/
void refreshGimbalData()
{
  static unsigned int nSamples = 0;

  const unsigned int OVERSAMPLE = (ExpressLRS_currAirRate_Modparams->interval / 250);

  // TODO stop treating reversed differently to non-reversed. Maths can handle signs.
  // Values as read from the ADC - will change if width or atten are changed
  // roll is reversed
  const uint16_t ADC_ROLL_MIN = 667;
  const uint16_t ADC_ROLL_CTR = 1985;
  const uint16_t ADC_ROLL_MAX = 3098;

  // not reversed
  const uint16_t ADC_PITCH_MIN = 1147;
  const uint16_t ADC_PITCH_CTR = 2096;
  const uint16_t ADC_PITCH_MAX = 3125;

  // reversed, no centre needed for throttle
  const uint16_t ADC_THROTTLE_MIN = 825;
  const uint16_t ADC_THROTTLE_MAX = 3280;

  // not reversed
  const uint16_t ADC_YAW_MIN = 855;
  const uint16_t ADC_YAW_CTR = 2018;
  const uint16_t ADC_YAW_MAX = 3255;

  // the initializers are unnecessary but prevent compiler warnings
  uint32_t raw_adc_roll     = 0;
  uint32_t raw_adc_pitch    = 0;
  uint32_t raw_adc_throttle = 0;
  uint32_t raw_adc_yaw      = 0;

  // (over)sample the ADC
  // TODO is it better to interleave the channels or do them one after the other?
  // for(int i=0; i<OVERSAMPLE; i++) {
  //   // TODO convert from gpio to adc_channel
  //   raw_adc_roll     = LPF_roll.update(adc1_get_raw(ADC1_CHANNEL_0));
  //   raw_adc_pitch    = LPF_pitch.update(adc1_get_raw(ADC1_CHANNEL_1));
  //   raw_adc_throttle = LPF_throttle.update(adc1_get_raw(ADC1_CHANNEL_2));
  //   raw_adc_yaw      = LPF_yaw.update(adc1_get_raw(ADC1_CHANNEL_3));
  // }

  raw_adc_roll = 0;
  raw_adc_pitch = 0;
  raw_adc_throttle = 0;
  raw_adc_yaw = 0;
  for(int i=0; i<OVERSAMPLE; i++) {
    // TODO convert from gpio to adc_channel
    raw_adc_roll     += adc1_get_raw(ADC1_CHANNEL_0);
    raw_adc_pitch    += adc1_get_raw(ADC1_CHANNEL_1);
    raw_adc_throttle += adc1_get_raw(ADC1_CHANNEL_2);
    raw_adc_yaw      += adc1_get_raw(ADC1_CHANNEL_3);
  }
  raw_adc_roll     /= OVERSAMPLE;
  raw_adc_pitch    /= OVERSAMPLE;
  raw_adc_throttle /= OVERSAMPLE;
  raw_adc_yaw      /= OVERSAMPLE;

  // todo - use the timestamp system for auto freq?
  double roll_1e     = f_roll.filter(raw_adc_roll);
  double pitch_1e    = f_pitch.filter(raw_adc_pitch);
  double throttle_1e = f_throttle.filter(raw_adc_throttle);
  double yaw_1e      = f_yaw.filter(raw_adc_yaw);

  // logging for tuning the filter
  // Serial.print(raw_adc_roll);
  // Serial.print(" ");Serial.println(roll_1e);

  // convert the doubles back to integers
  raw_adc_roll     = roll_1e;
  raw_adc_pitch    = pitch_1e;
  raw_adc_throttle = throttle_1e;
  raw_adc_yaw      = yaw_1e;

  nSamples++;

  // limit and scale the results

  // crsf uses a reduced range, and BF expects to see it.
  const uint32_t MAX_OUT = 1811;
  const uint32_t MID_OUT =  992;
  const uint32_t MIN_OUT =  172;

  // roll is reversed
  uint32_t adc_roll;
  if (raw_adc_roll <= ADC_ROLL_MIN) {
    adc_roll = MAX_OUT;
  } else if (raw_adc_roll >= ADC_ROLL_MAX) {
    adc_roll = MIN_OUT;
  } else if (raw_adc_roll > ADC_ROLL_CTR) {
  // if (raw_adc_roll > ADC_ROLL_CTR) {
    // high half scaling, with reversed output
    adc_roll = MID_OUT - (raw_adc_roll - ADC_ROLL_CTR) * (MAX_OUT - MID_OUT) / (ADC_ROLL_MAX - ADC_ROLL_CTR);
  } else {
    // low half scaling, with reversed output
    adc_roll = MID_OUT + ((ADC_ROLL_CTR - raw_adc_roll) * (MID_OUT - MIN_OUT) / (ADC_ROLL_CTR - ADC_ROLL_MIN));
  }

  // pitch is not reversed
  uint32_t adc_pitch;
  if (raw_adc_pitch <= ADC_PITCH_MIN) {
    adc_pitch = MIN_OUT;
  } else if (raw_adc_pitch >= ADC_PITCH_MAX) {
    adc_pitch = MAX_OUT;
  } else if (raw_adc_pitch > ADC_PITCH_CTR) {
    // high half scaling
    adc_pitch = MID_OUT + (raw_adc_pitch - ADC_PITCH_CTR) * (MAX_OUT - MID_OUT) / (ADC_PITCH_MAX - ADC_PITCH_CTR);
  } else {
    // low half scaling, with reversed output
    adc_pitch = MID_OUT - ((ADC_PITCH_CTR - raw_adc_pitch) * (MID_OUT - MIN_OUT) / (ADC_PITCH_CTR - ADC_PITCH_MIN));
  }

  // throttle is reversed, but doesn't need a centre position
  uint32_t adc_throttle;
  if (raw_adc_throttle <= ADC_THROTTLE_MIN) {
    adc_throttle = MAX_OUT;
  } else if (raw_adc_throttle >= ADC_THROTTLE_MAX) {
    adc_throttle = MIN_OUT;
  } else {
    adc_throttle = MIN_OUT + ((ADC_THROTTLE_MAX - raw_adc_throttle) * (MAX_OUT - MIN_OUT) / (ADC_ROLL_MAX - ADC_ROLL_MIN));
  }

  // yaw is not reversed
  uint32_t adc_yaw;
  if (raw_adc_yaw <= ADC_YAW_MIN) {
    adc_yaw = MIN_OUT;
  } else if (raw_adc_yaw >= ADC_YAW_MAX) {
    adc_yaw = MAX_OUT;
  } else if (raw_adc_yaw > ADC_YAW_CTR) {
    // high half scaling
    adc_yaw = MID_OUT + (raw_adc_yaw - ADC_YAW_CTR) * (MAX_OUT - MID_OUT) / (ADC_YAW_MAX - ADC_YAW_CTR);
  } else {
    // low half scaling, with reversed output
    adc_yaw = MID_OUT - ((ADC_YAW_CTR - raw_adc_yaw) * (MID_OUT - MIN_OUT) / (ADC_YAW_CTR - ADC_YAW_MIN));
  }

  // TODO measure the delay caused by these LPFs and figure out the trade-offs between more
  // hardware filtering and/or more oversampling.
  // adc_roll = LPF_roll.update((int32_t)adc_roll);
  // adc_pitch = LPF_pitch.update((int32_t)adc_pitch);
  // adc_throttle = LPF_throttle.update((int32_t)adc_throttle);
  // adc_yaw = LPF_yaw.update((int32_t)adc_yaw);

  // dirty hack to try things out. Pretend the data came from openTX
  crsf.ChannelDataIn[0] = adc_roll;
  crsf.ChannelDataIn[1] = adc_pitch;
  crsf.ChannelDataIn[2] = adc_throttle;
  crsf.ChannelDataIn[3] = adc_yaw;

  // switches
  // TODO these should be properly debounced...
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

  // debug
  // static unsigned long lastDebug = 0;
  // unsigned long now = millis();
  // if (now > (lastDebug + 2000)) {
  //   Serial.printf("aux %d %d\n", crsf.currentSwitches[0], crsf.currentSwitches[1]);
    // Serial.print("roll "); Serial.print(raw_adc_roll);
    // Serial.print(":"); Serial.print(adc_roll);
    // Serial.print(" 1e: ");Serial.println(roll_1e);
  //   Serial.print(" pitch "); Serial.print(raw_adc_pitch);
  //   Serial.print(":"); Serial.print(adc_pitch);
  //   Serial.print(" throttle "); Serial.print(raw_adc_throttle);
  //   Serial.print(":"); Serial.print(adc_throttle);
  //   Serial.print(" yaw "); Serial.print(raw_adc_yaw);
  //   Serial.print(":"); Serial.print(adc_yaw);

  //   Serial.print(" samples/s "); Serial.println(nSamples*1000/(now-lastDebug));

  //   Serial.print("ADC: "); Serial.print(minADC);
  //   Serial.print(":"); Serial.print(maxADC);
  //   Serial.print(":"); Serial.println(totalADC/nSamples);

  //   Serial.printf("AUX: %d %d %d %d\n", a1, a2, a3, a4);
    
  //   lastDebug = now;
  //   nSamples = 0;
  //   totalADC = 0;
  // }
  
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

void ICACHE_RAM_ATTR Generate4ChannelData_10bit()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + RC_DATA_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Serial.println("TODO generate OTA packet for DAG");
  Radio.TXdataBuffer[1] = ((CRSF_to_UINT10(crsf.ChannelDataIn[0]) & 0b1111111100) >> 2);
  // Radio.TXdataBuffer[2] = ((CRSF_to_UINT10(crsf.ChannelDataIn[1]) & 0b1111111100) >> 2);
  // Radio.TXdataBuffer[3] = ((CRSF_to_UINT10(crsf.ChannelDataIn[2]) & 0b1111111100) >> 2);
  // Radio.TXdataBuffer[4] = ((CRSF_to_UINT10(crsf.ChannelDataIn[3]) & 0b1111111100) >> 2);
  // Radio.TXdataBuffer[5] = ((CRSF_to_UINT10(crsf.ChannelDataIn[0]) & 0b0000000011) << 6) +
  //                         ((CRSF_to_UINT10(crsf.ChannelDataIn[1]) & 0b0000000011) << 4) +
  //                         ((CRSF_to_UINT10(crsf.ChannelDataIn[2]) & 0b0000000011) << 2) +
  //                         ((CRSF_to_UINT10(crsf.ChannelDataIn[3]) & 0b0000000011) << 0);
}

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
    Serial.println("Unable to send MSP command. Packet too long.");
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
  // update the 1e filters
  double freq = 1000000.0 / ModParams->interval;
  Serial.print("setting 1e freq to "); Serial.println(freq);
  f_roll.setFrequency(freq);
  #endif
}

uint8_t ICACHE_RAM_ATTR decTLMrate()
{
  Serial.println("dec TLM");
  uint8_t currTLMinterval = (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;

  if (currTLMinterval < (uint8_t)TLM_RATIO_1_2)
  {
    ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)(currTLMinterval + 1);
    Serial.println(currTLMinterval);
  }
  return (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;
}

uint8_t ICACHE_RAM_ATTR incTLMrate()
{
  Serial.println("inc TLM");
  uint8_t currTLMinterval = (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;

  if (currTLMinterval > (uint8_t)TLM_RATIO_NO_TLM)
  {
    ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)(currTLMinterval - 1);
  }
  return (uint8_t)ExpressLRS_currAirRate_Modparams->TLMinterval;
}

void ICACHE_RAM_ATTR decRFLinkRate()
{
  Serial.println("dec RFrate");
  SetRFLinkRate(ExpressLRS_currAirRate_Modparams->index + 1);
}

void ICACHE_RAM_ATTR incRFLinkRate()
{
  Serial.println("inc RFrate");
  SetRFLinkRate(ExpressLRS_currAirRate_Modparams->index - 1);
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
    if (modresult != 0) // wait for tlm response because it's time
    {
      return;
    }
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
  ProcessTLMpacket();
}

void ICACHE_RAM_ATTR TXdoneISR()
{
  NonceTX++; // must be done before callback
  HandleFHSS();
  HandleTLM();
  RadioIsIdle = true;

  // is there a pending rate change?
  if (nextRFLinkRate >= 0) {
    SetRFLinkRate((uint8_t)nextRFLinkRate);
    isRXconnected = false;
    LastTLMpacketRecvMillis = 0; // kick the tx into reconnecting straight away
    nextRFLinkRate = -1; // so that the button can be used again
    lcdNeedsRedraw = true; // trigger a screen update to show the new rate
  }

  // is there a pending power change?
  if (nextRadioPower != NONE_PENDING) {
    Radio.SetOutputPower(nextRadioPower);
    Serial.print("power set to ");Serial.println(nextRadioPower);
    nextRadioPower = NONE_PENDING; // so that the button can be used again
    lcdNeedsRedraw = true; // trigger a screen update to show the new rate
  }
}

void redrawDisplay()
{
  tft.fillScreen(TFT_BLUE);

  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.setTextFont(2);

  tft.drawString("ExpressLRS", TFT_WIDTH/2, 0);

  tft.setTextDatum(TL_DATUM);

  float pmw = pow10(((float)Radio.currPWR)/10.0f);

  Serial.print(Radio.currPWR); Serial.print(" "); Serial.println(pmw);

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

  // TODO add battery voltage
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

      // give the user time to do something about it
      delay(100);
    }

  } while (!throttleSafe || anySwitchSet);

  // tft.fillScreen(TFT_BLUE);
}

void setup()
{
#ifdef PLATFORM_ESP32
  // Serial.begin(115200);
  Serial.begin(460800);
  #ifdef USE_UART2
    Serial2.begin(400000);
  #endif
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
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
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
  tft.init();
  tft.setRotation(2);

  // switches
  pinMode(GPIO_AUX1, INPUT_PULLUP);
  pinMode(GPIO_AUX2, INPUT_PULLUP);
  pinMode(GPIO_AUX3, INPUT_PULLUP);
  pinMode(GPIO_AUX4, INPUT_PULLUP);

  // ttgo buttons
  pinMode(GPIO_BUTTON1, INPUT);
  pinMode(GPIO_BUTTON2, INPUT);

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

  Serial.println("ExpressLRS TX Module Booted...");

  // setup the ADC channels:
  initADC();

  POWERMGNT.init();
  Radio.currFreq = GetInitialFreq(); //set frequency first or an error will occur!!!
  Radio.Begin();
  //Radio.SetSyncWord(UID[3]);
  // POWERMGNT.setDefaultPower();
  Radio.SetOutputPower(radioPower);

  SetRFLinkRate(RATE_DEFAULT);
  // crsf.Begin();

  startupSafetyCheck();

  // redrawDisplay();

  hwTimer.init();
  // hwTimer.stop(); //comment to automatically start the RX timer and leave it running

}

void loop()
{
  static uint32_t lastDebugOutput = 0;
  static int button1state = 1;
  static int button2state = 1;

  static unsigned long b1LastChange = 0;
  static unsigned long b2LastChange = 0;

  // buttons read 0 when pressed
  // button 1 for packet rate
  int b1 = digitalRead(GPIO_BUTTON1);
  if (b1 != button1state && millis() > (b1LastChange + 100)) {
    // record the new state
    button1state = b1;
    b1LastChange = millis();
    if (b1 == 0 && nextRFLinkRate < 0) {
      // don't do the change here or the conflict with the radio can crash the tx
      // set the variable which will trigger the change at the next opportunity
      nextRFLinkRate = (ExpressLRS_currAirRate_Modparams->index + 1) % RATE_MAX;
    }
  }

  // button 2 for power
  int b2 = digitalRead(GPIO_BUTTON2);
  if (b2 != button2state && millis() > (b2LastChange + 100)) {
    // record the new state
    button2state = b2;
    b2LastChange = millis();
    if (b2 == 0) {
      nextRadioPower = radioPower + 1;
      if (nextRadioPower > MAX_PRE_PA_POWER) {
        nextRadioPower = MIN_PRE_PA_POWER;
      }
      radioPower = nextRadioPower;
    }
  }


  // while(UpdateParamReq){
  //   HandleUpdateParameter();
  // }

#ifdef FEATURE_OPENTX_SYNC
  // Serial.println(crsf.OpenTXsyncOffset);
#endif

  if (millis() > (lastDebugOutput + 500)) {  // TODO not really debug anymore, change name
    lastDebugOutput = millis();

    // LastTLMpacketRecvMillis gets set to 0 after a rate change, and we need a full redraw to show the new rate
    // TODO add a global flag for requesting an LCD update
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


  if (millis() > (RX_CONNECTION_LOST_TIMEOUT + LastTLMpacketRecvMillis))
  {
    isRXconnected = false;
    #if defined(TARGET_R9M_TX) || defined(TARGET_R9M_LITE_TX)
    digitalWrite(GPIO_PIN_LED_RED, LOW);
    #endif
  }
  else
  {
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

delay(50); // stop this method from eating all available cpu
}

void ICACHE_RAM_ATTR TimerCallbackISR()
{
  static unsigned long lastDebug = 0;
  unsigned long start = micros();

  if (!UpdateRFparamReq)
  {
    RadioIsIdle = false;
    SendRCdataToRF();
  }
  else
  {
    NonceTX++;
  }

  unsigned long adc_start = micros();
  refreshGimbalData();
  unsigned long adc_end = micros();

  unsigned long tRF = adc_start - start;
  unsigned long tADC = adc_end - adc_start;

  totalADC += tADC;
  if (tADC > maxADC) {
    maxADC = tADC;
  } else if (tADC < minADC) {
    minADC = tADC;
  }

  // not really cpu load, more like timing window ustilisation
  cpuLoad =  LPF_cpuLoad.update((tADC+tRF)*100/ExpressLRS_currAirRate_Modparams->interval);

  unsigned long now = adc_end/1000;
  if (now > (lastDebug + 5000)) {
    lastDebug = now;
    Serial.print("tRF ");    Serial.print(tRF);
    Serial.print(", tADC "); Serial.print(tADC);
    Serial.print(", total ");Serial.print(tADC+tRF);
    Serial.print(" cpu ");   Serial.println(cpuLoad);
  }

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
