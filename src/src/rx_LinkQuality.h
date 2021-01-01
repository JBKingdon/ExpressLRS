#define MAX_LQ 99

volatile uint8_t linkQuality = 0;
volatile uint8_t linkQualityArray[MAX_LQ] = {0};
volatile uint32_t linkQualityArrayCounter = 0;
volatile uint8_t linkQualityArrayIndex = 0;

#include "common.h"

void ICACHE_RAM_ATTR incrementLQArray()
{
    linkQualityArrayCounter++;
    linkQualityArrayIndex = linkQualityArrayCounter % MAX_LQ;
    linkQualityArray[linkQualityArrayIndex] = 0;
}

// bool ICACHE_RAM_ATTR packetReceivedForCurrentFrame()
// {
//     return linkQualityArray[linkQualityArrayIndex] != 0;
// }

bool ICACHE_RAM_ATTR packetReceivedForPreviousFrame()
{
    uint32_t prevIndex = (linkQualityArrayIndex == 0) ? (MAX_LQ - 1) : (linkQualityArrayIndex - 1);

    return (linkQualityArray[prevIndex] != 0);
}


void ICACHE_RAM_ATTR addPacketToLQ()
{
    linkQualityArray[linkQualityArrayIndex] = 1;
}

int ICACHE_RAM_ATTR getRFlinkQuality()
{
    int LQ = 0;

    for (int i = 0; i < MAX_LQ; i++)
    {
        LQ += linkQualityArray[i];
    }

    return LQ;
}

int ICACHE_RAM_ATTR LQreset()
{
    for (int i = 0; i < MAX_LQ; i++)
    {
        linkQualityArray[i] = 0;
    }
    return 0;
}