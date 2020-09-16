// defines for the UI co-processor

#ifndef COPRO_H
#define COPRO_H

#ifdef TARGET_TX_DAG_V1_COPRO

#define PACKED __attribute__((packed))

#else

// #define PACKED // nothing

#endif // TARGET_TX_DAG_V1_COPRO

// All uart packets begin with an eyecatcher (and end with a crc)
#define PKT_EYECATCHER 0x55

// Packet types
#define PKT_SWITCH_STATE 1
#define PKT_TELEM        2
#define PKT_TXMODE       3
#define PKT_PARAM_CHANGE 4
#define PKT_SAVE_CONFIG  5

// params

#define PARAM_NONE      0
#define PARAM_POWER     1
#define PARAM_RATE      2
#define PARAM_TLM_INT   3

#define N_PARAM 4   // including 'none'

// direction of change

#define PARAM_INC  1
#define PARAM_DEC  2

// structures

// simplest packet with no payload. Used for commands such as save
typedef struct {
    unsigned eyecatcher : 8;
    unsigned packetType : 8;
    unsigned crc : 8;
} PACKED commandPacket_t;


// uart packet for switch state

// switchState is used to transmit switch positions from the co-pro to the main processor
// switchState needs to be sent with minimal latency

typedef struct switchState_s 
{
    unsigned eyecatcher : 8;
    unsigned packetType : 8;
    unsigned aux1 : 2;  // 4 switches packed into 1 byte of payload
    unsigned aux2 : 2;
    unsigned aux3 : 2;
    unsigned aux4 : 2;
    unsigned crc : 8;
} PACKED switchState_t;

union switchState_u
{
    uint8_t dataBytes[sizeof(switchState_t)];
    switchState_t switchState;
};

// txMode is for sending the config of the tx to the co-pro for display
// it is not expected to be sent often

typedef struct txMode_s
{
    unsigned power       : 16;  // radio transmit power in mW
    unsigned airRate     : 16;  // rf packet rate, Hz
    unsigned tlmInterval : 8;   // telemetry packet interval (in packets)
} PACKED txMode_t;

typedef struct txModePkt_s
{
    uint8_t eyecatcher;
    uint8_t packetType;
    txMode_t payload;
    uint8_t crc;
} txModePkt_t;

union txModePkt_u
{
    uint8_t dataBytes[sizeof(txModePkt_t)];
    txModePkt_t txModePkt;
};

// rxTelem is for sending the basic stats on the uplink to the co-pro for display
// these packets will be arriving throughout the flight

typedef struct rxTelem_s
{
    unsigned lq : 8;
    unsigned rssi_dBm : 8;
    unsigned snr : 8;
    unsigned cpu : 8;   // this is a debug stat, but since we currently only have 1 it's efficient to add it here
} PACKED rxTelem_t;

typedef struct rxTelemPkt_s
{
    uint8_t eyecatcher;
    uint8_t packetType;
    rxTelem_t payload;
    uint8_t crc;
} rxTelemPkt_t;

union rxTelemPkt_u
{
    uint8_t dataBytes[sizeof(rxTelemPkt_s)];
    rxTelemPkt_s rxTelemPkt;
};

// param changes

typedef struct paramChange_s
{
    unsigned param : 4;
    unsigned direction : 2;
    unsigned padding : 2;
} PACKED paramChange_t;

typedef struct paramChangePkt_s
{
    uint8_t eyecatcher;
    uint8_t packetType;
    paramChange_t payload;
    uint8_t crc;
} paramChangePkt_t;

union paramChangePkt_u
{
    uint8_t dataBytes[sizeof(paramChangePkt_s)];
    paramChangePkt_t paramChangePkt;
};


#endif // ndef COPRO_H