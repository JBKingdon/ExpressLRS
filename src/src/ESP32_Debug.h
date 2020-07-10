#pragma once 

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
#include "SX127xDriver.h"
#elif defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)
#include "SX1280Driver.h"
#endif

void DEBUG_PrintRadioPacketStats()
{
    Serial.print("SPItime: ");
    Serial.println(Radio.TXspiTime);
    Serial.print("TotalTime: ");
    Serial.println(Radio.TimeOnAir);
    Serial.print("HeadRoom: ");
    Serial.println(Radio.HeadRoom);
    Serial.print("PacketCount(HZ): ");
    Serial.println(Radio.PacketCount);
}