#include <Arduino.h>
#include <stdio.h>

#include <TFT_eSPI.h>
#include "Setup_240x240_ST7789.h"

#include "logo2.h"

#include "UI_CoPro_config.h"
#include "UI_CoPro.h"
#include "CRSF.h"


TFT_eSPI tft = TFT_eSPI();

switchState_t switches, newSwitches;

txMode_t txMode;
rxTelem_t rxTelem;

// static volatile bool sendSwitchesToMain = false;

// static unsigned int lastSwitchSentTime = 0;
static unsigned int lastSwitchChangedTime = 0;

static unsigned long armStartTime = 0;
static unsigned long armEndTime = 0;
static unsigned long lastSaveTime = 0;


// time in ms that a switch might bounce for
#define SWITCH_BOUNCE_TIME 3

// test value for the rot enc
uint16_t rotCounter = 0;

// flag to indicate the need for a full redraw of the LCD
bool lcdRedrawNeeded = true;

// flags to indicate that the rotary encoder has been moved
bool paramInc = false;
bool paramDec = false;

uint paramToChange = PARAM_NONE;
uint lastParamToChange = PARAM_NONE; // to save the currently active param if the UI times out

// time of last UI input
static unsigned int lastUIinput = 0;

/**
 * Send current switch positions to the main processor
 * May be called from the ISRs
 */
void sendSwitches()
{
    switchState_u *switchU = (switchState_u*)&switches;

    // copy the new switch values to be sent
    memcpy(&switches, &newSwitches, sizeof(switches)-1);

    // Calculate the crc. Don't include the old crc in the crc calc or it will never match
    switchU->switchState.crc = CalcCRC(switchU->dataBytes, sizeof(switchState_u)-1);

    Serial2.write((const char*)switchU->dataBytes, sizeof(switchState_u));
}

/** Send a parameter change request to the Main processor
 *  This gets called when the rotary encoder is moved. The parameter to be changed
 *  is controlled by 'paramToChange' and whether it is incremented or decremented by
 *  the paramInc/paramDec globals.
 */
void sendParamChange()
{
    paramChangePkt_u pktU;

    if (paramToChange == PARAM_NONE) {
        paramInc = false;
        paramDec = false;
        return;
    }

    pktU.paramChangePkt.eyecatcher = PKT_EYECATCHER;
    pktU.paramChangePkt.packetType = PKT_PARAM_CHANGE;
    pktU.paramChangePkt.payload.param = paramToChange;

    if (paramInc) {
        pktU.paramChangePkt.payload.direction = PARAM_INC;
        paramInc = false;
    } else if (paramDec) {
        pktU.paramChangePkt.payload.direction = PARAM_DEC;
        paramDec = false;
    } else {
        // nothing to do
        return;
    }

    // Calculate the crc. Don't include the old crc in the crc calc or it will never match
    pktU.paramChangePkt.crc = CalcCRC(pktU.dataBytes, sizeof(paramChangePkt_u)-1);

    Serial2.write((const char*)pktU.dataBytes, sizeof(paramChangePkt_u));
}

void sendSaveConfig()
{
    commandPacket_t pkt;

    pkt.eyecatcher = PKT_EYECATCHER;
    pkt.packetType = PKT_SAVE_CONFIG;

    // Calculate the crc. Don't include the old crc in the crc calc or it will never match
    pkt.crc = CalcCRC((uint8_t *)&pkt, sizeof(commandPacket_t)-1);

    Serial2.write((const char*)&pkt, sizeof(commandPacket_t));
}


unsigned int getSwitchValue(uint32_t highPin, uint32_t lowPin)
{
    int aHigh = digitalRead(highPin);
    int aLow = digitalRead(lowPin);

    int s = 0;

    if (aHigh == 0) {
        s = 2;

    } else if (aLow == 1) {
        s = 1;
    }

    return s;
}

void initSwitchValues()
{
    newSwitches.aux1 = getSwitchValue(AUX1_HIGH, AUX1_LOW);
    newSwitches.aux2 = getSwitchValue(AUX2_HIGH, AUX2_LOW);
    newSwitches.aux3 = getSwitchValue(AUX3_HIGH, AUX3_LOW);
    newSwitches.aux4 = getSwitchValue(AUX4_HIGH, AUX4_LOW);
}

void switch1_ISR()
{
    uint32_t now = millis();
    // Serial.println(now - lastSwitchChangedTime);
    lastSwitchChangedTime = now;
    newSwitches.aux1 = getSwitchValue(AUX1_HIGH, AUX1_LOW);;

    // if (switches.aux1 != s) {
    //     unsigned long now = millis();
    //     if (now > (lastSwitchSentTime+10)) {
    //         if (s>0 && switches.aux1==0) {
    //             armStartTime = now;
    //             armEndTime = 0;
    //         } else if (s==0) {
    //             armEndTime = now;
    //         }
    //     }
    //     switches.aux1 = s;
    //     sendSwitches(now);
    // }
}
void switch2_ISR()
{
    lastSwitchChangedTime = millis();
    newSwitches.aux2 = getSwitchValue(AUX2_HIGH, AUX2_LOW);

    // unsigned int s = getSwitchValue(AUX2_HIGH, AUX2_LOW);
    // if (switches.aux2 != s) {
    //     switches.aux2 = s;
    //     sendSwitches(millis());
    // }
}
void switch3_ISR()
{
    lastSwitchChangedTime = millis();
    newSwitches.aux3 = getSwitchValue(AUX3_HIGH, AUX3_LOW);

    // unsigned int s = getSwitchValue(AUX3_HIGH, AUX3_LOW);
    // if (switches.aux3 != s) {
    //     switches.aux3 = s;
    //     sendSwitches(millis());
    // }
}
void switch4_ISR()
{
    lastSwitchChangedTime = millis();
    newSwitches.aux4 = getSwitchValue(AUX4_HIGH, AUX4_LOW);

    // unsigned int s = getSwitchValue(AUX4_HIGH, AUX4_LOW);
    // if (switches.aux4 != s) {
    //     switches.aux4 = s;
    //     sendSwitches(millis());
    // }
}

/** Draw the infrequently updated display elements
 * 
 */
void redrawDisplay()
{
  tft.fillScreen(TFT_BLUE);

  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.setTextFont(4);

  tft.drawString("ExpressLRS 2.4", TFT_WIDTH/2, 0);

  tft.setTextDatum(TL_DATUM);

  tft.setCursor(0, 35, 2);
  tft.print("P ");
  tft.setTextFont(4);
  if (paramToChange == PARAM_POWER) {
    tft.setTextColor(TFT_BLUE, TFT_WHITE);
  }
  tft.printf("%d mW\n", txMode.power);

  tft.setTextColor(TFT_WHITE);
  tft.setTextFont(2);
  tft.print("R ");
  tft.setTextFont(4);
  if (paramToChange == PARAM_RATE) {
    tft.setTextColor(TFT_BLUE, TFT_WHITE);
  } else {
    tft.setTextColor(TFT_WHITE, TFT_BLUE);      
  }
  tft.printf("%d Hz", txMode.airRate);
  
  // display telem interval
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(TFT_WIDTH/2, 35, 2);
  tft.print("T ");
  tft.setTextFont(4);
  if (paramToChange == PARAM_TLM_INT) {
    tft.setTextColor(TFT_BLUE, TFT_WHITE);
  } else {
    tft.setTextColor(TFT_WHITE, TFT_BLUE);      
  }
  tft.printf("%d", txMode.tlmInterval);
}

void rotEnc_ISR()
{
    static uint32_t lastUpdate = 0;

    uint32_t now = millis();
    if (now < (lastUpdate + 75)) {
        return;
    }

    // allow time for bouncing to stop
    // delayMicroseconds(100);
    int encA, prev = 0;
    int sameCount = 0;
    for(int i=0; i<10; i++) {   // loops for up to 10 tries looking for 5 same values TODO #defs
        delayMicroseconds(10);  // NB these delays make this an expensive isr
        encA = digitalRead(ROT_ENC_A);
        if (encA == prev) {
            sameCount++;
            if (sameCount == 5) {
                break;
            }
        } else {
            sameCount = 0;
            prev = encA;
        }
    }

    if (encA == 1 || sameCount < 5) return; // XXX EARLY RETURN

    int encB = digitalRead(ROT_ENC_B);
    prev = encB;
    sameCount = 0;
    for(int i=0; i<10; i++) {   // loops for up to 10 tries looking for 5 same values TODO #defs
        delayMicroseconds(5);
        encB = digitalRead(ROT_ENC_B);
        if (encB == prev) {
            sameCount++;
            if (sameCount == 5) {
                break;
            }
        } else {
            sameCount = 0;
            prev = encB;
        }
    }

    if (encA==0) {
        // Serial.printf("encA %d, encB %d\n", encA, encB);
        lastUpdate = now;
        lastUIinput = now;
        if (!encB) {
            rotCounter++;
            paramInc = true;
        } else {
            rotCounter--;
            paramDec = true;
        }
    }
}

/**
 * Return the battery voltage in 100ths of a volt
 */
uint32_t readVbat()
{
    const uint SAMPLES = 8;

    // TODO check how long this takes
    uint32_t raw = 0;
    for(uint i=0; i<SAMPLES; i++) {
        raw += analogRead(PIN_VBAT);
    }
    raw /= SAMPLES;
    return raw * 734 / 4096; // 3.3 (vref) * 2 (r divider) * calibration * 100 (units) / 4096 (bit width)
}

void setup()
{
    // need to get serial2 up as quickly as possible so that we don't miss data from the main.
    Serial2.begin(460800);

    // get the splash screen up
    tft.init();
    tft.setRotation(2);
    tft.fillScreen(TFT_WHITE);  // 1st call to fillScreen leaves junk
    tft.fillScreen(TFT_WHITE);
    tft.drawBitmap(0, 0, logo2_map, 240, 240, TFT_BLACK);

    // led
    pinMode(PC13, OUTPUT);
    // led is active low, so turn off by setting high
    digitalWrite(PC13, HIGH);

    // for(int i=0; i<3; i++) {
    //     digitalWrite(PC13, LOW);
    //     delay(300);
    //     digitalWrite(PC13, HIGH);
    //     delay(300);
    // }

    pinMode(PIN_VBAT, INPUT_ANALOG);
    analogReadResolution(12);

    pinMode(UI_BUTTON1, INPUT_PULLUP);
    pinMode(UI_BUTTON2, INPUT_PULLUP);

    Serial.begin(460800);
    Serial.println("ExpressTX coprocessor started");

    // Let the user soak in the joy of the elrs splash screen
    delay(1000);

    // fixed parts of the switches packet
    switches.eyecatcher = PKT_EYECATCHER;
    switches.packetType = PKT_SWITCH_STATE;

    // configure pins for switch inputs
    pinMode(AUX1_LOW, INPUT_PULLUP);
    pinMode(AUX1_HIGH, INPUT_PULLUP);

    pinMode(AUX2_LOW, INPUT_PULLUP);
    pinMode(AUX2_HIGH, INPUT_PULLUP);

    pinMode(AUX3_LOW, INPUT_PULLUP);
    pinMode(AUX3_HIGH, INPUT_PULLUP);

    pinMode(AUX4_LOW, INPUT_PULLUP);
    pinMode(AUX4_HIGH, INPUT_PULLUP);

    // init the switch values
    initSwitchValues();
    lastSwitchChangedTime = millis();   // switches will be sent to the main processor from loop()

    // connect the inputs to the isrs
    attachInterrupt(digitalPinToInterrupt(AUX1_LOW),  switch1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(AUX1_HIGH), switch1_ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(AUX2_LOW),  switch2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(AUX2_HIGH), switch2_ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(AUX3_LOW),  switch3_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(AUX3_HIGH), switch3_ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(AUX4_LOW),  switch4_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(AUX4_HIGH), switch4_ISR, CHANGE);

    // setup the rotary encoder
    pinMode(ROT_ENC_A, INPUT_PULLUP);
    pinMode(ROT_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ROT_ENC_A),  rotEnc_ISR, FALLING);

    // init the UI
    redrawDisplay();
}

/**
 * This should just be drawing the changing elements of the display,
 * but it needs some tidy up.
 */
void updateDisplay()
{
    const uint BAT_FILT = 16;
    static uint vBatF = readVbat() * BAT_FILT;

    static bool clearSaveFlag = false;
    static int saveTextWidth = 0;


    tft.setCursor(0, 100, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);

    tft.print("RSSI "); // TODO move to static redraw
    tft.setTextFont(4);
    if (true) {
        tft.printf("%3d \n", (int8_t)rxTelem.rssi_dBm);
    } else {
        tft.println(" --    ");
    }

    tft.setTextFont(2);
    tft.print("SNR  ");
    tft.setTextFont(4);
    if (true) {
        tft.printf("%3d \n", rxTelem.snr);
    } else {
        tft.println(" --    ");
    }

    tft.setTextFont(2);
    tft.print("LQ   ");
    tft.setTextFont(4);
    if (true) {
        tft.printf("%3d  \n", rxTelem.lq);
     } else {
        tft.println(" --    ");
    }

    // if armEndTime isn't set then show a running timer, otherwise show the elapsed last armed time
    char *pstr;
    char buffer[32];
    if (armStartTime == 0) {
        // tft.println("    00:00  ");
        // pstr = (char*)"00:00"; // promise not to overwrite the constant
        pstr = buffer;
        sprintf(pstr, "  00:00");
    } else {
        pstr = buffer;
        unsigned long eTime = (armEndTime == 0) ? millis() : armEndTime;
        unsigned int armTime = (eTime - armStartTime) / 1000;
        unsigned int minutes = armTime / 60;
        unsigned int seconds = armTime % 60;
        sprintf(pstr, "  %02d:%02d", minutes, seconds);
    }

    tft.setTextDatum(TR_DATUM);
    tft.drawString(pstr, TFT_WIDTH, 185);

    tft.setTextDatum(TL_DATUM);
    tft.setCursor(0,185,2);
    tft.print("bat");

    // #define BAT_OVERSAMPLE 4
    // int batRaw = 0;
    // for(int i=0; i<BAT_OVERSAMPLE; i++) {
    //   int sample;
    //   adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_BIT_12, &sample);
    //   batRaw += sample;
    // }
    // batRaw = LPF_battery.update(batRaw/BAT_OVERSAMPLE);

    // // scale must match width and atten
    // const float scale = 7.885 / 4095; // (resistor divider factor * max atten voltage * fudgefactor) / max_reading
    // float vBat = (float) batRaw * scale;

    vBatF = (vBatF - vBatF/BAT_FILT) + readVbat();
    uint32_t vBat = vBatF/BAT_FILT;
    tft.setTextFont(4);
    tft.printf(" %d.%02d ", vBat/100, vBat%100);
    // tft.printf("%s", pstr);
    // tft.printf("%2.2fV", vBat);

    // tft.setTextFont(1);
    // sprintf(buffer, "cpu: %2d%%", cpuLoad);
    // tft.setTextDatum(BR_DATUM);
    // tft.drawString(buffer, TFT_WIDTH, TFT_HEIGHT);

    // tft.setTextDatum(TL_DATUM);

    // int a1 = digitalRead(GPIO_AUX1);
    // int a2 = digitalRead(GPIO_AUX2);

    // int b1 = digitalRead(GPIO_BUTTON1);
    // int b2 = digitalRead(GPIO_BUTTON2);

    tft.setCursor(0,230,1);
    tft.printf("A1 %d A2 %d A3 %d A4 %d ", switches.aux1, switches.aux2, switches.aux3, switches.aux4);
    // tft.printf("eeprom.length %d", EEPROM.length()); // 1024 on blue pill

    // tft.setTextDatum(TR_DATUM);
    sprintf(pstr, " cpu %u ", rxTelem.cpu);
    tft.drawRightString(pstr, TFT_WIDTH, 230, 1);

    if (millis() < (lastSaveTime + 1000)) {
        tft.setTextColor(TFT_BLUE, TFT_WHITE);
        saveTextWidth = tft.drawCentreString(" SAVED ", TFT_WIDTH/2, TFT_HEIGHT/2, 4);
        clearSaveFlag = true;
    } else if (clearSaveFlag) {
        tft.fillRect(TFT_WIDTH/2-saveTextWidth/2, TFT_HEIGHT/2, saveTextWidth, tft.fontHeight(4), TFT_BLUE);
        clearSaveFlag = false;
    }
}

// TODO reduce the redundancy in this code
void readFromMainCPU()
{
    // read bytes from serial port until it is empty, building up commands and executing them
    while(Serial2.available()) {
        int c = Serial2.read();
        if (c == PKT_EYECATCHER) {
            // read the command
            c = Serial2.read();
            switch(c) {
                default:
                    Serial.printf("unsupported command %02x\n", c);
                    break;
                case -1: // nothing was available
                    Serial.println("readFromMainCPU incomplete packet");
                    return; // give up on this packet
                    break;
                case PKT_TELEM:
                { // local scope for vars
                    rxTelemPkt_u tlmU;

                    tlmU.dataBytes[0] = PKT_EYECATCHER; // seems daft, but we need it for the crc
                    tlmU.dataBytes[1] = c;  // also needed for crc
                    // get the payload
                    const int len = sizeof(tlmU.rxTelemPkt.payload);
                    for(int i=0; i<len; i++) {
                        c = Serial2.read();
                        if (c == -1) {
                            // ran out of data - maybe worth trying harder?
                            Serial.println("incomplete packet");
                            return;
                        } else {
                            tlmU.dataBytes[2+i] = c;
                        }
                    }
                    // get the crc
                    uint8_t pktCRC = Serial2.read();
                    if (pktCRC == -1) {
                        // ran out of data - maybe worth trying harder?
                        Serial.println("no crc");
                        return;
                    }
                    // calculate the expected crc and compare with the sent one
                    uint8_t expectedCRC = CalcCRC(tlmU.dataBytes, sizeof(tlmU)-1);
                    if (pktCRC != expectedCRC) {
                        Serial.printf("crc mismatch, expected %02X, actual %02X\n", expectedCRC, pktCRC);
                        return;
                    }
                    // put the data somewhere useful!
                    // copy just the payload 
                    memcpy(&rxTelem, &tlmU.rxTelemPkt.payload, sizeof(tlmU.rxTelemPkt.payload));

                    break;
                }
                case PKT_TXMODE:
                { // local scope for vars
                    txModePkt_u modeU;

                    modeU.dataBytes[0] = PKT_EYECATCHER; // seems daft, but we need it for the crc
                    modeU.dataBytes[1] = c;  // also needed for crc
                    // get the payload
                    const int len = sizeof(modeU.txModePkt.payload);
                    for(int i=0; i<len; i++) {
                        c = Serial2.read();
                        if (c == -1) {
                            // ran out of data - maybe worth trying harder?
                            Serial.println("incomplete packet");
                            return;
                        } else {
                            modeU.dataBytes[2+i] = c;
                        }
                    }
                    // get the crc
                    uint8_t pktCRC = Serial2.read();
                    if (pktCRC == -1) {
                        // ran out of data - maybe worth trying harder?
                        Serial.println("no crc");
                        return;
                    }
                    // calculate the expected crc and compare with the sent one
                    uint8_t expectedCRC = CalcCRC(modeU.dataBytes, sizeof(modeU)-1);
                    if (pktCRC != expectedCRC) {
                        Serial.printf("crc mismatch, expected %02X, actual %02X\n", expectedCRC, pktCRC);
                        return;
                    }
                    // put the data somewhere useful!
                    // copy just the payload 
                    memcpy(&txMode, &modeU.txModePkt.payload, sizeof(modeU.txModePkt.payload));
                    lcdRedrawNeeded = true;
                }


            }
        } // if eyecatcher
    } // while data available

}

void loop()
{
    static unsigned long lastTFTUpdate = 0;
    static unsigned long button1LastPress = 0;
    static unsigned long button2LastPress = 0;
    static bool b2Down = false;

    if (Serial2.available()) {
        readFromMainCPU();
    }

    unsigned long now = millis();

    // has there been a switch change, and has it been long enough to stop bouncing?
    if (lastSwitchChangedTime && now > (lastSwitchChangedTime + SWITCH_BOUNCE_TIME))
    {
        lastSwitchChangedTime = 0;  // do this first to make the window for missing a switch change as small as possible
        // if the arm switch changed, do the timer stuff
        if (switches.aux1 != newSwitches.aux1) {
            if (newSwitches.aux1 == 2) {
                // we have become armed, so start the timer and clear previous end time
                armStartTime = now;
                armEndTime = 0;
            } else if (switches.aux1 == 2) {
                // we used to be armed, so record the end time
                armEndTime = now;
            }
        }
        sendSwitches();
    }

    // read buttons
    // if button1 pressed then round-robin through the adjustable params
    int b1 = digitalRead(UI_BUTTON1);
    if (b1 == 0 && now > (button1LastPress + 500)) {
        button1LastPress = now;
        lastUIinput = now;
        if (paramToChange == PARAM_NONE && lastParamToChange != PARAM_NONE) {
            paramToChange = lastParamToChange;
            lastParamToChange = PARAM_NONE; // to make sure we only use this once after a timeout
        } else {
            paramToChange = (paramToChange + 1) % N_PARAM;
        }
        lcdRedrawNeeded = true;
    }

    // if button2 pressed then save the config (by sending a command to the main processor)
    // debounce
    if (now > (button2LastPress + 1000)) {
        int b2 = digitalRead(UI_BUTTON2);
        if (!b2Down && (b2 == 0)) {
            b2Down = true;      // track the button state to prevent auto-repeat
            button2LastPress = now;
            lastUIinput = now;
            lastSaveTime = now;

            // send a 'save' command to the main processor
            sendSaveConfig();

            updateDisplay();    // get the 'saved' indicator on screen

        } else if (b2Down && b2 == 1) {
            b2Down = false;
        }
    }

    if (lcdRedrawNeeded) {
        redrawDisplay();
        lcdRedrawNeeded = false;
    }

    // if (sendSwitchesToMain) {
    //     sendSwitches();
    // }

    if (paramInc || paramDec) {
        sendParamChange();
    }

    if (now > (lastTFTUpdate + 200)) {
        // tft.setCursor(0, 220);
        // tft.setTextColor(TFT_WHITE, TFT_BLUE);
        // tft.printf("s: %d %d %d %d ", switches.aux1, switches.aux2, switches.aux3, switches.aux4);
        lastTFTUpdate = now;
        // Serial.printf("switches %d %d %d %d\n", switches.aux1, switches.aux2, switches.aux3, switches.aux4);
        updateDisplay();
    }

    // timeout the param change UI?
    if ((paramToChange != PARAM_NONE) && (now > (lastUIinput + 15000))) {
        lastParamToChange = paramToChange;
        paramToChange = PARAM_NONE;
        lcdRedrawNeeded = true;
    }

    delay(5);
}