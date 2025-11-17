#pragma once

#include <cstdint>
#include <cstring>
#include "dw3000_regs.h"
#include "Arduino.h"

// ----------- Nachrichten-Layout-Definitionen-----------
#define MSG_SN_IDX 2
#define MSG_DID_IDX 5
#define MSG_SID_IDX 7
#define MSG_FUNC_IDX 9
#define MSG_T_REPLY_IDX 10
#define RESP_MSG_TS_LEN 4
// ----------- UWB Protokoll Funktionscodes -----------
#define FUNC_CODE_POLL 0xE2
#define FUNC_CODE_ACK 0xE3
#define FUNC_CODE_RANGE 0xE4
#define FUNC_CODE_FINAL 0xE5
#define FUNC_CODE_RESET 0xE6
#define FUNC_CODE_DISCOVERY_BROADCAST 0xD1
#define FUNC_CODE_DISCOVERY_BLINK 0xD2
#define FUNC_CODE_RANGING_CONFIG 0x20

#define MAX_UWB_MESSAGE_LEN 127

class mac_802_15_04
{
public:
    static const uint16_t RANGING_MSG_LEN = 16;

    mac_802_15_04();

    // ---- Builder-Methoden ----
    void buildRangingMessage(uint8_t seqNum, uint8_t funcCode, uint8_t destShortAddr, uint8_t sourceUid);
    void buildDiscoveryBroadcast(uint8_t seqNum, uint64_t sourceMac);
    void buildDiscoveryBlink(uint8_t seqNum, uint64_t destMac, uint64_t sourceMac);
    void buildRangingConfig(uint8_t seqNum, uint64_t sourceMac, uint64_t destinationMac, uint8_t initiatorUid, uint8_t assignedResponderUid, uint8_t totalDevices);
                            
    void parse(const uint8_t *rxBuffer, uint16_t length);
    bool parseRangingConfig(uint8_t &initiatorUid, uint8_t &assignedResponderUid, uint8_t &totalDevices) const;
    // ---- Getter ----
    uint8_t getFunctionCode() const;
    uint8_t getSourceUid() const;
    uint64_t getSourceMac() const;
    uint64_t getDestinationMac() const;
    uint16_t getSourceAddressShort() const;
    uint8_t getSequenceNumber() const;

    // ---- Zeitstempel & API-Zugriff ----
    void getTimestamp(uint32_t *timestamp) const;
    void setTimestamp(uint64_t timestamp);
    uint8_t *getBuffer();
    uint16_t getLength() const;
    void print() const;

private:
    uint8_t buffer[MAX_UWB_MESSAGE_LEN];
    uint16_t currentLength;
    uint8_t getSourceAddrMode() const;
    uint8_t getDestAddrMode() const;
    uint8_t getHeaderLength() const;
};