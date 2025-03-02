#pragma once 

#include "targets.h"
#include "elrs_eeprom.h"
#include "luaParams.h"

#define TX_CONFIG_VERSION   2
#define RX_CONFIG_VERSION   2
#define UID_LEN             6

typedef struct {
    uint32_t    version;
    uint32_t    rate;
    uint32_t    tlm;
    uint32_t    power;
    char        ssid[33];
    char        password[33];
} tx_config_t;

class TxConfig
{
public:
    void Load();
    void Commit();

    // Getters
    uint32_t GetRate() const { return m_config.rate; }
    uint32_t GetTlm() const { return m_config.tlm; }
    uint32_t GetPower() const { return m_config.power; }
    bool     IsModified() const { return m_modified; }
    const char* GetSSID() const { return m_config.ssid; }
    const char* GetPassword() const { return m_config.password; }

    // Setters
    void SetRate(uint32_t rate);
    void SetTlm(uint32_t tlm);
    void SetPower(uint32_t power);
    void SetDefaults();
    void SetStorageProvider(ELRS_EEPROM *eeprom);
    void SetSSID(const char *ssid);
    void SetPassword(const char *password);

private:
    tx_config_t m_config;
    ELRS_EEPROM *m_eeprom;
    bool        m_modified;
};

///////////////////////////////////////////////////

typedef struct {
    uint32_t    version;
    bool        isBound;
    uint8_t     uid[UID_LEN];
    uint8_t     powerOnCounter;
    char        ssid[33];
    char        password[33];
} rx_config_t;

class RxConfig
{
public:
    void Load();
    void Commit();

    // Getters
    bool     GetIsBound() const {
        #ifdef MY_UID
            return true;
        #else
            return m_config.isBound;
        #endif
    }
    const uint8_t* GetUID() const { return m_config.uid; }
    uint8_t  GetPowerOnCounter() const { return m_config.powerOnCounter; }
    bool     IsModified() const { return m_modified; }
    const char* GetSSID() const { return m_config.ssid; }
    const char* GetPassword() const { return m_config.password; }

    // Setters
    void SetIsBound(bool isBound);
    void SetUID(uint8_t* uid);
    void SetPowerOnCounter(uint8_t powerOnCounter);
    void SetDefaults();
    void SetStorageProvider(ELRS_EEPROM *eeprom);
    void SetSSID(const char *ssid);
    void SetPassword(const char *password);

private:
    rx_config_t m_config;
    ELRS_EEPROM *m_eeprom;
    bool        m_modified;
};
