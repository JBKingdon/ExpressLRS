#include <Arduino.h>
#include "targets.h"
#include "utils.h"
#include "common.h"
#include "LowPassFilter.h"

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
#include "SX127xDriver.h"
SX127xDriver Radio;
#elif defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)
#include "SX1280Driver.h"
SX1280Driver Radio;
#endif

#include "CRSF.h"
#include "FHSS.h"
// #include "Debug.h"
#include "rx_LinkQuality.h"
#include "OTA.h"
// #include "msp.h"
#include "msptypes.h"

#ifdef PLATFORM_ESP8266
#include "ESP8266_WebUpdate.h"
#include "ESP8266_hwTimer.h"
#endif

#ifdef PLATFORM_STM32
#include "STM32_UARTinHandler.h"
#include "STM32_hwTimer.h"
#endif

//// CONSTANTS ////
#define BUTTON_SAMPLE_INTERVAL 150
#define WEB_UPDATE_PRESS_INTERVAL 2000 // hold button for 2 sec to enable webupdate mode
#define BUTTON_RESET_INTERVAL 4000     //hold button for 4 sec to reboot RX
#define WEB_UPDATE_LED_FLASH_INTERVAL 25
#define SEND_LINK_STATS_TO_FC_INTERVAL 50    // normal rate
// #define SEND_LINK_STATS_TO_FC_INTERVAL 5    // hires debug rate
///////////////////

#define DEBUG_SUPPRESS // supresses debug messages on uart

// #define BAUDRATE 460800 // for targetted debug output

// #define DIVERSITY_DEV_MODE // couples the diversity mode to AUX4

#define DIVERSITY_FLIP_IN_TOCK    // move the post rx antenna flip to tock() so that happens even when a packet is dropped

// NB ANTENNA_SWITCH define moved to targets.h

uint8_t antenna = 0;        // currently active antenna
DiversityModes_e divMode = DIV_RSSI;
int8_t lastPacketRSSI[2];   // last RSSI for a successful packet on each antenna

// int8_t lastRXrssi=0;

hwTimer hwTimer;

CRSF crsf(Serial); //pass a serial port object to the class for it to use

/// Filters ////////////////
LPF LPF_PacketInterval(3);
LPF LPF_Offset(2);
LPF LPF_OffsetDx(4);
LPF LPF_UplinkRSSI0(5);
LPF LPF_UplinkRSSI1(5);
////////////////////////////

uint8_t scanIndex = RATE_DEFAULT;
uint8_t CURR_RATE_MAX = RATE_MAX;

int32_t HWtimerError;
int32_t RawOffset;
int32_t Offset;
int32_t OffsetDx;
int32_t prevOffset;
RXtimerState_e RXtimerState;
uint32_t GotConnectionMillis = 0;
uint32_t ConsiderConnGoodMillis = 1000; // minimum time before we can consider a connection to be 'good'
bool lowRateMode = false;

bool LED = false;

// debug counters
uint32_t rxCount=0, crcErrorCount=0;

//// Variables Relating to Button behaviour ////
bool buttonPrevValue = true; //default pullup
bool buttonDown = false;     //is the button current pressed down?
uint32_t buttonLastSampled = 0;
uint32_t buttonLastPressed = 0;

bool webUpdateMode = false;
bool webUpdateDisabled = false;
uint32_t webUpdateLedFlashIntervalLast;
///////////////////////////////////////////////

volatile uint8_t NonceRX = 0; // nonce that we THINK we are up to.

bool alreadyFHSS = false;
bool alreadyTLMresp = false;

uint32_t headroom;
uint32_t headroom2;
uint32_t beginProcessing;
uint32_t doneProcessing;

//////////////////////////////////////////////////////////////

///////Variables for Telemetry and Link Quality///////////////
uint32_t ModuleBootTime = 0;
uint32_t LastValidPacketMicros = 0;
uint32_t LastValidPacketPrevMicros = 0; //Previous to the last valid packet (used to measure the packet interval)
unsigned long LastValidPacket = 0;           //Time the last valid packet was recv
uint32_t LastSyncPacket = 0;            //Time the last valid packet was recv

uint32_t SendLinkStatstoFCintervalLastSent = 0;

int16_t RFnoiseFloor; //measurement of the current RF noise floor
///////////////////////////////////////////////////////////////

uint32_t PacketIntervalError;
uint32_t PacketInterval;

/// Variables for Sync Behaviour ////
uint32_t RFmodeLastCycled = 0;
///////////////////////////////////////

// flip to the other antenna
// no-op if ANTENNA_SWITCH not defined
void ICACHE_RAM_ATTR switchAntenna()
{
    #ifdef ANTENNA_SWITCH

    antenna = !antenna;

    digitalWrite(ANTENNA_SWITCH, antenna);

    #ifdef ANTENNA_SWITCH_CMP
    digitalWrite(ANTENNA_SWITCH_CMP, !antenna);
    #endif

    #endif
}

// use the specified antenna
// no-op if ANTENNA_SWITCH not defined
// void ICACHE_RAM_ATTR setAntenna(uint8_t x)
// {
//     #ifdef ANTENNA_SWITCH

//     // EARLY RETURN for invalid input, or if there is no change to the existing setting
//     if (x > 1 || antenna == x) return;

//     antenna = x;
//     digitalWrite(ANTENNA_SWITCH, antenna);

//     #endif
// }


void ICACHE_RAM_ATTR getRFlinkInfo()
{
    int8_t lastRSSI = Radio.GetLastPacketRSSI();

    lastPacketRSSI[antenna] = lastRSSI; // save the value for use by the diversity code in SX1280_hal.cpp

    int32_t rssiDBM0 = LPF_UplinkRSSI0.SmoothDataINT;
    int32_t rssiDBM1 = LPF_UplinkRSSI1.SmoothDataINT;
    switch (antenna) {
        case 0:
            rssiDBM0 = LPF_UplinkRSSI0.update(lastRSSI);
            break;
        case 1:
            rssiDBM1 = LPF_UplinkRSSI1.update(lastRSSI);
            break;
    }

    #ifndef USE_ELRS_CRSF_EXTENSIONS
    crsf.PackedRCdataOut.ch15 = UINT10_to_CRSF(map(constrain(LastRSSI, -100, -50), -100, -50, 0, 1023));
    crsf.PackedRCdataOut.ch14 = UINT10_to_CRSF(fmap(linkQuality, 0, 100, 0, 1023));
    #endif

    // rssiDBM on sx1276 is currently in the range -128 to 98, but BF wants a value in the range
    // 0 to 255 that maps to -1 * the negative part of the rssiDBM, so cap at 0.
    // rssiDBM on sx1280 runs -127 to 0 anyway, but this won't hurt
    if (rssiDBM0 > 0) rssiDBM0 = 0;
    if (rssiDBM1 > 0) rssiDBM1 = 0;
    #ifdef USE_ELRS_CRSF_EXTENSIONS
    crsf.LinkStatistics.rssi0 = -rssiDBM0; // negate to match BF
    #ifdef ANTENNA_SWITCH
    crsf.LinkStatistics.rssi1 = -rssiDBM1;
    #else
    crsf.LinkStatistics.rssi1 = 255;    // special value that we can use in BF to suppress the diversity rssi display
    #endif
    // crsf.LinkStatistics.snr = Radio.LastPacketSNR; // * 10; Swapped out snr for rssi1
    crsf.LinkStatistics.link_quality = linkQuality | (antenna << 7); // carry the current antenna info in the top bit of lq
    crsf.LinkStatistics.rf_Mode = RATE_MAX - ExpressLRS_currAirRate_Modparams->index;

    #else
    // #error "old stuff" // if only I could remember why I put this here
    crsf.LinkStatistics.uplink_RSSI_1 = -rssiDBM0; // to match BF
    crsf.LinkStatistics.uplink_RSSI_2 = -rssiDBM1;
    crsf.LinkStatistics.uplink_SNR = Radio.LastPacketSNR; // * 10;
    crsf.LinkStatistics.uplink_Link_quality = linkQuality;
    crsf.LinkStatistics.rf_Mode = RATE_MAX - ExpressLRS_currAirRate_Modparams->index;
    #endif
}

void ICACHE_RAM_ATTR SetRFLinkRate(uint8_t index) // Set speed of RF link (hz)
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen);
    hwTimer.updateInterval(ModParams->interval);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
}

void ICACHE_RAM_ATTR SetTLMRate(expresslrs_tlm_ratio_e TLMrateIn)
{
    ExpressLRS_currAirRate_Modparams->TLMinterval = TLMrateIn;
}

void ICACHE_RAM_ATTR HandleFHSS()
{
    if ((ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) || alreadyFHSS == true)
    {
        return;
    }

    uint8_t modresult = (NonceRX + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    if ((modresult != 0) || (connectionState == disconnected)) // don't hop if disconnected
    {
        return;
    }

    alreadyFHSS = true;
    Radio.SetFrequency(FHSSgetNextFreq());

    // Serial.println("hop");

     // Start another RX unless we're about to send telem
    if ((ExpressLRS_currAirRate_Modparams->TLMinterval == TLM_RATIO_NO_TLM) || 
        (((NonceRX + 1) % (TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval))) != 0)
        )
    {
        Radio.RXnb();
    }
}

void ICACHE_RAM_ATTR HandleSendTelemetryResponse()
{
    if ((connectionState == disconnected) || (ExpressLRS_currAirRate_Modparams->TLMinterval == TLM_RATIO_NO_TLM) || (alreadyTLMresp == true))
    {
        return; // don't bother sending tlm if disconnected or TLM is off
    }

    uint8_t modresult = (NonceRX + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    if (modresult != 0)
    {
        return;
    }

    alreadyTLMresp = true;

    Radio.TXdataBuffer[0] = (DeviceAddr << 2) + 0b11; // address + tlm packet
    Radio.TXdataBuffer[1] = CRSF_FRAMETYPE_LINK_STATISTICS;

    // OpenTX hard codes "rssi" warnings to the LQ sensor for crossfire, so the
    // rssi we send is for display only.
    // OpenTX treats the rssi values as signed.

    // TODO - add a elrs controller specific mode that returns both the rssi raw values

    #ifdef USE_ELRS_CRSF_EXTENSIONS
    uint8_t openTxRSSI = antenna ? crsf.LinkStatistics.rssi1 : crsf.LinkStatistics.rssi0;
    // truncate the range to fit into OpenTX's 8 bit signed value
    if (openTxRSSI > 127)
        openTxRSSI = 127;
    // convert to 8 bit signed value in the negative range (-128 to 0)
    openTxRSSI = 255 - openTxRSSI;
    Radio.TXdataBuffer[2] = openTxRSSI;
    // Radio.TXdataBuffer[3] = add 2nd rssi here

    Radio.TXdataBuffer[3] = (crsf.TLMbattSensor.voltage & 0xFF00) >> 8;
    // Radio.TXdataBuffer[4] = crsf.LinkStatistics.snr; // todo get the snr from somewhere else
    Radio.TXdataBuffer[5] = crsf.LinkStatistics.link_quality & 0x7F; // mask out the active antenna bit

    #else
    uint8_t openTxRSSI = crsf.LinkStatistics.uplink_RSSI_1;
    // truncate the range to fit into OpenTX's 8 bit signed value
    if (openTxRSSI > 127)
        openTxRSSI = 127;
    // convert to 8 bit signed value in the negative range (-128 to 0)
    openTxRSSI = 255 - openTxRSSI;
    Radio.TXdataBuffer[2] = openTxRSSI;

    Radio.TXdataBuffer[3] = (crsf.TLMbattSensor.voltage & 0xFF00) >> 8;
    Radio.TXdataBuffer[4] = crsf.LinkStatistics.uplink_SNR;
    Radio.TXdataBuffer[5] = crsf.LinkStatistics.uplink_Link_quality;
    #endif

    Radio.TXdataBuffer[6] = (crsf.TLMbattSensor.voltage & 0x00FF);

    uint8_t crc = CalcCRC(Radio.TXdataBuffer, 7) + CRCCaesarCipher;
    Radio.TXdataBuffer[7] = crc;
    Radio.TXnb(Radio.TXdataBuffer, 8);

    return;
}

#ifndef TARGET_SX1280
void ICACHE_RAM_ATTR HandleFreqCorr(bool value)
{
    //Serial.println(FreqCorrection);
    if (!value)
    {
        if (FreqCorrection < FreqCorrectionMax)
        {
            FreqCorrection += 61; //min freq step is ~ 61hz
        }
        else
        {
            FreqCorrection = FreqCorrectionMax;
            FreqCorrection = 0; //reset because something went wrong
#ifndef DEBUG_SUPPRESS
            Serial.println("Max pos reasontable freq offset correction limit reached!");
#endif
        }
    }
    else
    {
        if (FreqCorrection > FreqCorrectionMin)
        {
            FreqCorrection -= 61; //min freq step is ~ 61hz
        }
        else
        {
            FreqCorrection = FreqCorrectionMin;
            FreqCorrection = 0; //reset because something went wrong
#ifndef DEBUG_SUPPRESS
            Serial.println("Max neg reasontable freq offset correction limit reached!");
#endif
        }
    }
}
#endif // ndef TARGET_SX1280

void ICACHE_RAM_ATTR HWtimerCallbackTick() // this is 180 out of phase with the other callback
{
    NonceRX++;
    alreadyFHSS = false;
    linkQuality = getRFlinkQuality();
    incrementLQArray();
}

void ICACHE_RAM_ATTR HWtimerCallbackTock()
{
    #ifdef DIVERSITY_DEV_MODE

    static int32_t prevRSSI;        // saved rssi so that we can compare if switching made things better or worse
    static bool antennaSwitched = false;

    // diversity mode controlled by aux4 for debugging crsf.PackedRCdataOut.aux4
    switch (divMode)
    {
        case DIV_DROPPED:
            if (!packetReceivedForPreviousFrame()) {
                // We didn't get a packet so switch the antenna
                prevRSSI = (antenna == 0) ? LPF_UplinkRSSI0.SmoothDataINT : LPF_UplinkRSSI1.SmoothDataINT;
                switchAntenna();
                antennaSwitched = true;
            } else if (antennaSwitched) {
                // We switched last time, so check the rssi didn't get worse
                // XXX switching on smoothed rssi may not be useful. Use the lastPacketRSSI values instead?
                int32_t rssi = (antenna == 0) ? LPF_UplinkRSSI0.SmoothDataINT : LPF_UplinkRSSI1.SmoothDataINT;
                if (rssi < prevRSSI) {
                    // things got worse when we switched, change back
                    switchAntenna();
                    prevRSSI = rssi;
                } else {
                    // all good, we can stay on the current antenna, so clear the flag
                    antennaSwitched = false;
                }
            }
            break;
        case DIV_NONE: // switching is turned off
            // setAntenna(0);
            break;
        case DIV_RSSI: // code is spread between RXdoneISR and SX1280_hal.cpp:doISR
            // XXX TODO this has changed for preamble rssi and this needs updating to match the DIVERSITY_FLIP_IN_TOCK block below
            break;
        default:
            break;
    }

    #elif defined(DIVERSITY_FLIP_IN_TOCK)

    switchAntenna();
    
    #endif // DIVERSITY_DEV_MODE/DIVERSITY_FLIP_IN_TOCK

    HandleFHSS();   // may (will usually) call rxnb to start the next receive
    HandleSendTelemetryResponse();
}

void ICACHE_RAM_ATTR LostConnection()
{
    if (connectionState == disconnected)
    {
        return; // Already disconnected
    }

    connectionStatePrev = connectionState;
    connectionState = disconnected; //set lost connection
    RXtimerState = tim_disconnected;
    FreqCorrection = 0;
    HWtimerError = 0;
    Offset = 0;
    prevOffset = 0;
    LPF_Offset.init(0);

    digitalWrite(GPIO_PIN_LED, 0);        // turn off led
    Radio.SetFrequency(GetInitialFreq()); // in conn lost state we always want to listen on freq index 0
    hwTimer.stop();

    #ifndef DEBUG_SUPPRESS
    Serial.println("lost conn");
    #endif

#ifdef PLATFORM_STM32
    digitalWrite(GPIO_PIN_LED_GREEN, LOW);
#endif
}

void ICACHE_RAM_ATTR TentativeConnection()
{
    hwTimer.resume();
    connectionStatePrev = connectionState;
    connectionState = tentative;
    RXtimerState = tim_disconnected;

    #ifndef DEBUG_SUPPRESS
    Serial.println("tentative conn");
    #endif

    FreqCorrection = 0;
    HWtimerError = 0;
    Offset = 0;
    prevOffset = 0;
    LPF_Offset.init(0);
}

void ICACHE_RAM_ATTR GotConnection()
{
    if (connectionState == connected)
    {
        return; // Already connected
    }

    connectionStatePrev = connectionState;
    connectionState = connected; //we got a packet, therefore no lost connection
    RXtimerState = tim_tentative;
    GotConnectionMillis = millis();

    RFmodeLastCycled = millis();   // give another 3 sec for loc to occur.
    digitalWrite(GPIO_PIN_LED, 1); // turn on led

    #ifndef DEBUG_SUPPRESS
    Serial.println("got conn");
    #endif

    // set the flag to disable webserver mode after first connection
    webUpdateDisabled = true;

#ifdef PLATFORM_STM32
    digitalWrite(GPIO_PIN_LED_GREEN, HIGH);
#endif
}

#ifdef NOPE // stuff not being used that we don't want filling the limited icache
void ICACHE_RAM_ATTR UnpackChannelData_11bit()
{
    crsf.PackedRCdataOut.ch0 = (Radio.RXdataBuffer[1] << 3) + ((Radio.RXdataBuffer[5] & 0b11100000) >> 5);
    crsf.PackedRCdataOut.ch1 = (Radio.RXdataBuffer[2] << 3) + ((Radio.RXdataBuffer[5] & 0b00011100) >> 2);
    crsf.PackedRCdataOut.ch2 = (Radio.RXdataBuffer[3] << 3) + ((Radio.RXdataBuffer[5] & 0b00000011) << 1) + (Radio.RXdataBuffer[6] & 0b10000000 >> 7);
    crsf.PackedRCdataOut.ch3 = (Radio.RXdataBuffer[4] << 3) + ((Radio.RXdataBuffer[6] & 0b01110000) >> 4);
#ifdef One_Bit_Switches
    crsf.PackedRCdataOut.ch4 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00001000);
    crsf.PackedRCdataOut.ch5 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00000100);
    crsf.PackedRCdataOut.ch6 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00000010);
    crsf.PackedRCdataOut.ch7 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00000001);
#endif
}

void ICACHE_RAM_ATTR UnpackChannelData_10bit()
{
    #ifdef USE_ELRS_CRSF_EXTENSIONS
    // XXX untested
    crsf.PackedRCdataOut.chan0 = (Radio.RXdataBuffer[1] << 2) + ((Radio.RXdataBuffer[5] & 0b11000000) >> 6);
    crsf.PackedRCdataOut.chan1 = (Radio.RXdataBuffer[2] << 2) + ((Radio.RXdataBuffer[5] & 0b00110000) >> 4);
    crsf.PackedRCdataOut.chan2 = (Radio.RXdataBuffer[3] << 2) + ((Radio.RXdataBuffer[5] & 0b00001100) >> 2);
    crsf.PackedRCdataOut.chan3 = (Radio.RXdataBuffer[4] << 2) + ((Radio.RXdataBuffer[5] & 0b00000011) >> 0);
    #else
    crsf.PackedRCdataOut.ch0 = UINT10_to_CRSF((Radio.RXdataBuffer[1] << 2) + ((Radio.RXdataBuffer[5] & 0b11000000) >> 6));
    crsf.PackedRCdataOut.ch1 = UINT10_to_CRSF((Radio.RXdataBuffer[2] << 2) + ((Radio.RXdataBuffer[5] & 0b00110000) >> 4));
    crsf.PackedRCdataOut.ch2 = UINT10_to_CRSF((Radio.RXdataBuffer[3] << 2) + ((Radio.RXdataBuffer[5] & 0b00001100) >> 2));
    crsf.PackedRCdataOut.ch3 = UINT10_to_CRSF((Radio.RXdataBuffer[4] << 2) + ((Radio.RXdataBuffer[5] & 0b00000011) >> 0));
    #endif
}

void ICACHE_RAM_ATTR UnpackMSPData()
{
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.flags = 0;
    packet.function = Radio.RXdataBuffer[1];
    packet.addByte(Radio.RXdataBuffer[3]);
    packet.addByte(Radio.RXdataBuffer[4]);
    packet.addByte(Radio.RXdataBuffer[5]);
    packet.addByte(Radio.RXdataBuffer[6]);
    crsf.sendMSPFrameToFC(&packet);
}
#endif // ifdef NOPE

void ICACHE_RAM_ATTR ProcessRFPacket()
{
    beginProcessing = micros();
    uint8_t calculatedCRC = CalcCRC(Radio.RXdataBuffer, 7) + CRCCaesarCipher;
    uint8_t inCRC = Radio.RXdataBuffer[7];
    uint8_t type = Radio.RXdataBuffer[0] & 0b11;
    uint8_t packetAddr = (Radio.RXdataBuffer[0] & 0b11111100) >> 2;

    // Serial.println("packet!");

    if (inCRC != calculatedCRC)
    {
        // Serial.println("CRC error");
        crcErrorCount++;

        // #ifndef DEBUG_SUPPRESS
        // Serial.print("CRC error on RF packet: ");
        // for (int i = 0; i < 8; i++)
        // {
        //     Serial.print(Radio.RXdataBuffer[i], HEX);
        //     Serial.print(",");
        // }
        // Serial.println("");
        // #endif
        return;
    }

    if (packetAddr != DeviceAddr)
    {
        #ifndef DEBUG_SUPPRESS
        Serial.println("Wrong device address on RF packet");
        #endif
        return;
    }

    LastValidPacketPrevMicros = LastValidPacketMicros;
    LastValidPacketMicros = beginProcessing;
    LastValidPacket = millis();

    getRFlinkInfo();

    switch (type)
    {
    case RC_DATA_PACKET: //Standard RC Data Packet
        #if defined SEQ_SWITCHES
        UnpackChannelDataSeqSwitches(Radio.RXdataBuffer, &crsf);
        #elif defined HYBRID_SWITCHES_8
        UnpackChannelDataHybridSwitches8(Radio.RXdataBuffer, &crsf);
        #else
        UnpackChannelData_11bit();
        #endif

        if (connectionState == connected) {
            crsf.sendRCFrameToFC();
        }

        #ifdef DIVERSITY_DEV_MODE
        // development: map aux4 to divMode for diversity testing
        switch (crsf.PackedRCdataOut.aux4)
        {
            case 0:
                divMode = DIV_RSSI;   // rssi by default
                break;
            case 1:
                divMode = DIV_NONE; // middle position to disable diversity
                break;
            default:
                divMode = DIV_DROPPED;
                break;
        }
        #endif // DIVERSITY_DEV_MODE

        break;

    case MSP_DATA_PACKET:
        // UnpackMSPData();
        break;

    case TLM_PACKET: //telemetry packet from master

        // not implimented yet
        break;

    case SYNC_PACKET: //sync packet from master
        if (Radio.RXdataBuffer[4] == UID[3] && Radio.RXdataBuffer[5] == UID[4] && Radio.RXdataBuffer[6] == UID[5])
        {
            LastSyncPacket = millis();
            #ifndef DEBUG_SUPPRESS
            Serial.println("sync");
            #endif
            if ((NonceRX == Radio.RXdataBuffer[2]) && (FHSSgetCurrIndex() == Radio.RXdataBuffer[1]) && (abs(OffsetDx) <= 10) && (linkQuality > 75))
                GotConnection();

            expresslrs_RFrates_e rateIn = (expresslrs_RFrates_e)((Radio.RXdataBuffer[3] & 0b11100000) >> 5);
            uint8_t TLMrateIn = ((Radio.RXdataBuffer[3] & 0b00011100) >> 2);

            if ((ExpressLRS_currAirRate_Modparams->index != rateIn) || (ExpressLRS_currAirRate_Modparams->TLMinterval != (expresslrs_tlm_ratio_e)TLMrateIn))
            { // change link parameters if required
                #ifndef DEBUG_SUPPRESS
                if (ExpressLRS_currAirRate_Modparams->TLMinterval != (expresslrs_tlm_ratio_e)TLMrateIn) {
                    Serial.print("New TLMrate: ");
                    Serial.println(TLMrateIn);
                }
                if (ExpressLRS_currAirRate_Modparams->index != rateIn) {
                    Serial.print("New air-rate: ");
                    Serial.println(rateIn);
                }
                #endif
                ExpressLRS_AirRateNeedsUpdate = true;
                ExpressLRS_currAirRate_Modparams = get_elrs_airRateConfig((expresslrs_RFrates_e)rateIn);
                ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)TLMrateIn;
            }

            if (NonceRX != Radio.RXdataBuffer[2] || FHSSgetCurrIndex() != Radio.RXdataBuffer[1])
            {
                #ifndef DEBUG_SUPPRESS
                if (NonceRX != Radio.RXdataBuffer[2]) {
                    Serial.println("nonce mismatch");
                }
                if (FHSSgetCurrIndex() != Radio.RXdataBuffer[1]) {
                    Serial.println("FHSS mismatch");
                }
                #endif
                FHSSsetCurrIndex(Radio.RXdataBuffer[1]);
                NonceRX = Radio.RXdataBuffer[2];
                TentativeConnection();
                //return;
            }

        }
        break;

    default:
        break;
    }

    HandleFHSS();
    HandleSendTelemetryResponse();
    addPacketToLQ(); // Adds packet to LQ otherwise an artificial drop in LQ is seen due to sending TLM.

    if (connectionState != disconnected)
    {
        HWtimerError = ((LastValidPacketMicros - hwTimer.LastCallbackMicrosTick) % ExpressLRS_currAirRate_Modparams->interval);
        RawOffset = (HWtimerError - (ExpressLRS_currAirRate_Modparams->interval >> 1) + 50); // the offset is because we want the hwtimer tick to occur slightly after the packet would have otherwise been recv
        OffsetDx = LPF_OffsetDx.update(abs(RawOffset - prevOffset));
        Offset = LPF_Offset.update(RawOffset); //crude 'locking function' to lock hardware timer to transmitter, seems to work well enough
        prevOffset = Offset;

        if(RXtimerState == tim_locked){
            hwTimer.phaseShift((Offset >> 3) + timerOffset);
        }
        else
        {
            hwTimer.phaseShift((RawOffset >> 4) + timerOffset);
        }
    }

    #ifndef TARGET_SX1280
    if ((alreadyFHSS == false) || (ExpressLRS_currAirRate_Modparams->index > 2))
    {
        HandleFreqCorr(Radio.GetFrequencyErrorbool()); //corrects for RX freq offset
        Radio.SetPPMoffsetReg(FreqCorrection);         //as above but corrects a different PPM offset based on freq error
    }
    #endif
    doneProcessing = micros();

// #ifndef DEBUG_SUPPRESS
//     Serial.print(RawOffset);
//     Serial.print(":");
//     Serial.print(Offset);
//     Serial.print(":");
//     Serial.print(OffsetDx);
//     Serial.print(":");
//     Serial.println(linkQuality);
// #endif
}

#ifdef USE_WEBSERVER
void beginWebsever()
{
#ifdef PLATFORM_STM32
#else
    hwTimer.stop();
    BeginWebUpdate();
    webUpdateMode = true;
#endif
}
#endif // ifdef USE_WEBSERVER

void sampleButton()
{
    // if gpio 0 is being used for the antenna switch then we can't read the button
    #ifdef ANTENNA_SWITCH
    return;
    #endif

    bool buttonValue = digitalRead(GPIO_PIN_BUTTON);

    if (buttonValue == false && buttonPrevValue == true)
    { //falling edge
        buttonLastPressed = millis();
        buttonDown = true;
        Serial.println("Manual Start");
        Radio.SetFrequency(GetInitialFreq());
        Radio.RXnb();
    }

    if (buttonValue == true && buttonPrevValue == false) //rising edge
    {
        buttonDown = false;
    }

    #ifdef USE_WEBSERVER
    if ((millis() > buttonLastPressed + WEB_UPDATE_PRESS_INTERVAL) && buttonDown) // button held down
    {
        if (!webUpdateMode)
        {
            beginWebsever();
        }
    }
    #endif // ifdef USE_WEBSERVER

    if ((millis() > buttonLastPressed + BUTTON_RESET_INTERVAL) && buttonDown)
    {
#ifdef PLATFORM_ESP8266
        ESP.restart();
#endif
    }

    buttonPrevValue = buttonValue;
}

void ICACHE_RAM_ATTR RXdoneISR()
{
    rxCount++;
    ProcessRFPacket();
    //Serial.println("r");

    // send link stats here so that it can never collide with the uart use in ProcessRFPacket.
    // If we're doing telemetry then processRFPacket didn't send any uart traffic, so it's an
    // ideal time to send linkstats.
    // Can we tell if we just got a sync packet - that's also a good time to send stats
    if (connectionState != disconnected && 
        (alreadyTLMresp || ((millis() - SendLinkStatstoFCintervalLastSent) > SEND_LINK_STATS_TO_FC_INTERVAL))
       )
    {
        crsf.sendLinkStatisticsToFC();
        SendLinkStatstoFCintervalLastSent = millis();
    }

    // TODO Is this the right place to do this? What about dropped packets? Try moving to Tock()
    // prep for the next packet by switching antenna. sx1280 hal will get rssi inst at the beginning
    // of the next packet and switch the antenna back if we made things worse
    #ifndef DIVERSITY_FLIP_IN_TOCK
    if (divMode == DIV_RSSI) {
        switchAntenna();
    }
    #endif
}

void ICACHE_RAM_ATTR TXdoneISR()
{
    alreadyTLMresp = false;
    addPacketToLQ(); // Adds packet to LQ otherwise an artificial drop in LQ is seen due to sending TLM.
    Radio.RXnb();
}

void setup()
{
    delay(100);
    Serial.println("ExpressLRS Module Booting...");

#ifdef PLATFORM_STM32
    Serial.setTx(GPIO_PIN_RCSIGNAL_TX);
    Serial.setRx(GPIO_PIN_RCSIGNAL_RX);
    pinMode(GPIO_PIN_LED_GREEN, OUTPUT);
    pinMode(GPIO_PIN_LED_RED, OUTPUT);
    pinMode(GPIO_PIN_LED, OUTPUT);
    pinMode(GPIO_PIN_BUTTON, INPUT);
#endif

#if defined(PLATFORM_ESP8266) && defined(USE_WEBSERVER)
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    pinMode(GPIO_PIN_LED, OUTPUT);
#endif

#ifdef PLATFORM_STM32
    pinMode(GPIO_PIN_LED_GREEN, OUTPUT);
#endif

#ifdef ANTENNA_SWITCH
    pinMode(ANTENNA_SWITCH, OUTPUT);
    digitalWrite(ANTENNA_SWITCH, antenna);
#else
    pinMode(GPIO_PIN_BUTTON, INPUT);
#endif

// Are we using complementary control on the RF switch?
#ifdef ANTENNA_SWITCH_CMP
    pinMode(ANTENNA_SWITCH_CMP, OUTPUT);
    digitalWrite(ANTENNA_SWITCH_CMP, !antenna);
#endif

#ifdef Regulatory_Domain_AU_915
    Serial.println("Setting 915MHz Mode");
    //Radio.RFmodule = RFMOD_SX1276; //define radio module here
#elif defined Regulatory_Domain_FCC_915
    Serial.println("Setting 915MHz Mode");
    //Radio.RFmodule = RFMOD_SX1276; //define radio module here
#elif defined Regulatory_Domain_EU_868
    Serial.println("Setting 868MHz Mode");
    //Radio.RFmodule = RFMOD_SX1276; //define radio module here
#elif defined Regulatory_Domain_AU_433 || defined Regulatory_Domain_EU_433
    Serial.println("Setting 433MHz Mode");
    //Radio.RFmodule = RFMOD_SX1278; //define radio module here
#endif

    #ifdef BAUDRATE
    Serial.begin(BAUDRATE); // if set, override all the other stuff
    #else
    #ifndef DEBUG_SUPPRESS
    // Serial.begin(230400); // for linux debugging with dodgy uart adapters?
    Serial.begin(460800); // for linux debugging for normal adapters
    #else
    #ifdef USE_ELRS_CRSF_EXTENSIONS
    Serial.begin(691200);   // JBK extended crsf impl. Needs modified BF build to match
    #else
    Serial.begin(420000); // normal crsf baud rate
    #endif // USE_ELRS_CRSF_EXTENSIONS
    #endif // DEBUG_SUPPRESS
    #endif // BAUDRATE

    FHSSrandomiseFHSSsequence();

    Radio.currFreq = GetInitialFreq();
    Radio.Begin();
    //Radio.SetSyncWord(UID[3]);
    #ifdef TARGET_RX_ESP8266_SX1280_V1
    Radio.SetOutputPower(13); //default is max power (12.5dBm for SX1280 RX)
    #else
    Radio.SetOutputPower(0b1111); //default is max power (17dBm for SX127x RX@)
    #endif

    // RFnoiseFloor = MeasureNoiseFloor(); //TODO move MeasureNoiseFloor to driver libs
    // Serial.print("RF noise floor: ");
    // Serial.print(RFnoiseFloor);
    // Serial.println("dBm");

    Radio.RXdoneCallback = &RXdoneISR;
    Radio.TXdoneCallback = &TXdoneISR;

    hwTimer.callbackTock = &HWtimerCallbackTock;
    hwTimer.callbackTick = &HWtimerCallbackTick;

    SetRFLinkRate(0);
    Radio.RXnb();
    crsf.Begin();
    hwTimer.init();
    hwTimer.stop();
}

void loop()
{
    #ifndef DEBUG_SUPPRESS
    static uint32_t lastDebug = 0;

    if ((connectionState != disconnected) && millis() > (lastDebug + 1000)) {
        // Serial.println(linkQuality);
        Serial.print(crcErrorCount);
        Serial.print("/");
        Serial.print(rxCount);
        Serial.print(" ");
        crcErrorCount = 0;
        rxCount = 0;

        // Serial.print("rssi0 "); Serial.print(rssiDebug[0]);
        // Serial.print(" rssi1 "); Serial.print(rssiDebug[1]);

        int32_t rssiDBM0 = LPF_UplinkRSSI0.SmoothDataINT;
        int32_t rssiDBM1 = LPF_UplinkRSSI1.SmoothDataINT;

        Serial.print(" filt "); Serial.print(rssiDBM0);
        Serial.print(" "); Serial.print(rssiDBM1);

        // Serial.print(-crsf.LinkStatistics.rssi);
        // Serial.print("dBm ");
        // Serial.print(RawOffset);
        // Serial.print(":");
        // Serial.print(Offset);
        // Serial.print(":");
        // Serial.print(OffsetDx);
        // Serial.print(":");
        Serial.print(" LQ ");
        Serial.println(linkQuality);
        lastDebug = millis();
    }
    #endif // DEBUG_SUPPRESS

    //crsf.RXhandleUARTout(); //empty the UART out buffer
    //yield(); // to be safe
    //Serial.println(linkQuality);
    //Serial.print(headroom);
    //Serial.println(" Head2:");
    //Serial.println(headroom2);
    //crsf.RXhandleUARTout(); using interrupt based printing at the moment

#if defined(PLATFORM_ESP8266) && defined(Auto_WiFi_On_Boot) && defined(USE_WEBSERVER)
    if (!webUpdateDisabled && (connectionState == disconnected) && !webUpdateMode && millis() > 20000 && millis() < 21000)
    {
        beginWebsever();
    }

    if (webUpdateMode)
    {
        HandleWebUpdate();
        if (millis() > WEB_UPDATE_LED_FLASH_INTERVAL + webUpdateLedFlashIntervalLast)
        {
            digitalWrite(GPIO_PIN_LED, LED);
            LED = !LED;
            webUpdateLedFlashIntervalLast = millis();
        }
        yield();
        return;
    }
#endif

    if (connectionState == tentative && (linkQuality <= 75 || abs(OffsetDx) > 10 || Offset > 100) && 
            ((millis() - LastSyncPacket) > ExpressLRS_currAirRate_RFperfParams->RFmodeCycleAddtionalTime))
    {
        LostConnection();
        #ifndef DEBUG_SUPPRESS
        Serial.print("Bad sync, aborting");
        if (linkQuality <= 75) Serial.print(" LQ");
        if (abs(OffsetDx) > 10) Serial.print(" oDx");
        if (Offset > 100) Serial.print(" offset");
        Serial.println();
        #endif
        Radio.SetFrequency(GetInitialFreq());
        Radio.RXnb();
        //scanIndex = (RATE_MAX - 1) - ExpressLRS_currAirRate_Modparams->index;
        RFmodeLastCycled = millis();
        LastSyncPacket = millis();
    }

    // if (lowRateMode == false) // this makes it latch to ON if it ever gets triggered
    // {
    //     if (millis() > (LastValidPacket + 60000))
    //     {
    //         lowRateMode = true;
    //         CURR_RATE_MAX = RATE_MAX; //switch between 200hz, 100hz, 50hz, 25hz, 4hz rates
    //         scanIndex = 3;
    //     }
    //     else
    //     {
    //         CURR_RATE_MAX = 4; //switch between 200hz, 100hz, 50hz, rates
    //     }
    // }

    unsigned long now = millis();

    // is it time to try a different packet rate for getting a connection?
    if ((connectionState == disconnected) && 
        !webUpdateMode &&
        ((now - RFmodeLastCycled) > (ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval/2))) //  RFmodeCycleInterval divisor to test faster searching 
    {
        LastSyncPacket = now;                                        // reset this variable
        SetRFLinkRate((expresslrs_RFrates_e)(scanIndex % CURR_RATE_MAX)); //switch between rates
        Radio.RXnb();
        LQreset();
        digitalWrite(GPIO_PIN_LED, LED);
        LED = !LED;

        #ifndef DEBUG_SUPPRESS
        Serial.println(ExpressLRS_currAirRate_Modparams->interval);
        #endif
        
        scanIndex++;
        RFmodeLastCycled = now;
    }

    // check if we lost conn
    // old version
    // if ((connectionState == connected) && 
    //     ((now > (LastValidPacket + ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval)) || ((millis() > (LastSyncPacket + 11000)) && linkQuality < 10))
    //     )

    // With the first timeout rewritten in Peho format
    // if ((connectionState == connected) && 
    //     (((millis() - localLVP) > ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval) || ((millis() > (LastSyncPacket + 11000)) && linkQuality < 10))
    //     )

    // don't use the cached value of millis() here, if a packet is received after the value of millis() was saved in 'now',
    // LastValidPacket gets updated to a value that's later than the cached millis, the arithemetic wraps and we get a false failsafe
    // The casts to signed should cope with lastValidPacket being after millis(), but I'm keeping the local var anyway.
    const unsigned long localLVP = LastValidPacket; // make absolutely certain that the isr can't update this after we read millis() below
    if ((connectionState == connected) && ((int32_t)(millis() - localLVP) > (int32_t)ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval)
    // if ((connectionState == connected) && ((int32_t)(millis() - LastValidPacket) > ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval) 
        //  || (((now - LastSyncPacket) > 11000) && linkQuality < 10) // this doesn't seem safe with noSyncWhenArmed
        )
    {
        LostConnection();
    }

    if ((connectionState == tentative) && linkQuality >= 95) // quicker way to get to good conn state of the sync and link is great off the bat. 
    {
        GotConnection();
    }

    if ((millis() - buttonLastSampled) > BUTTON_SAMPLE_INTERVAL)
    {
        sampleButton();
        buttonLastSampled = millis();
    }

    if ((RXtimerState == tim_tentative) && (millis() > (GotConnectionMillis + ConsiderConnGoodMillis)) && (OffsetDx <= 5))
    {
        RXtimerState = tim_locked;
        #ifndef DEBUG_SUPPRESS
        Serial.println("Timer Considered Locked");
        #endif
    }

    #ifdef PLATFORM_STM32
    STM32_RX_HandleUARTin();
    #endif

    delay(10);
}
