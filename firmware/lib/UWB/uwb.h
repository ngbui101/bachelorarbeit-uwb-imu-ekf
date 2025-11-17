#pragma once

#ifndef UWB_H
#define UWB_H

#include "dw3000.h"
#include "mac_802_15_04.h"
#include "dw3000.h"
#include "WiFi.h"

struct RangingPartner
{
    uint64_t mac_address;
    uint8_t uid;
    double distance;
};


class mac_802_15_04;

// ----------- Systemkonfiguration & Pins -----------
#define INTERVAL 5
#define UWB_RST 27
#define UWB_IRQ 34
#define UWB_SS 4
#define MAX_NODES 7

// ----------- Ranging Configuration -----------
extern uint8_t UID;
extern uint8_t INITIATOR_UID;
extern int NUM_NODES;
extern int WAIT_NUM;

// ----------- Rollen-spezifische Konfiguration -----------

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
// ----------- Timing-Konstanten -----------
#define TX_TO_RX_DLY_UUS 100
#define RX_TO_TX_DLY_UUS 3200
#define RX_TIMEOUT_UUS 800000

// ----------- Globale Variablen -----------
extern mac_802_15_04 txMessage;
extern mac_802_15_04 rxMessage;
//
extern uint8_t func_code;

extern uint8_t rx_buffer[MAX_UWB_MESSAGE_LEN];
extern uint8_t frame_seq_nb;
extern uint32_t status_reg;
extern int target_uids[MAX_NODES - 1];
extern bool wait_poll, wait_ack, wait_range, wait_final;
extern int counter;
extern int ret;
extern uint64_t poll_tx_ts, poll_rx_ts, range_tx_ts, ack_tx_ts, range_rx_ts;
extern uint32_t t_reply_1[MAX_NODES - 1];
extern uint64_t t_reply_2;
extern uint64_t t_round_1[MAX_NODES - 1];
extern uint32_t t_round_2[MAX_NODES - 1];
extern double tof, distance;
extern unsigned long previous_debug_millis, current_debug_millis;
extern int millis_since_last_serial_print;
extern uint32_t tx_time;
extern uint64_t tx_ts;
extern uint64_t myMacAddress;
extern RangingPartner known_devices[MAX_NODES];
extern int known_devices_count;
extern unsigned long last_discovery_millis;
extern bool configInitatorMode;
extern bool configResponderMode;
// ----------- Funktionsprototypen -----------
void start_uwb();
void configInitiator();
void configResponder();
void initiator();
void responder();
void setRangingConfiguration(uint8_t initiatorUid, uint8_t myAssignedUid, uint8_t totalDevices);
void responder_loop();
void initiator_loop();
void uwb_sniffer_loop();
void printdebug();
String formatUwbDataToString();

#endif