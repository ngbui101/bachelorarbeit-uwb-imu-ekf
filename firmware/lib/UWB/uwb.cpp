#include "uwb.h"
#include <sys/time.h>
#include "mqtt.h"

// Globale DW3000-Konfigurationen
dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0};
extern dwt_txconfig_t txconfig_options;

uint32_t int_mask =
    SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK | // TX ok
    SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | // RX ok
    SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK | // Frame Sync Loss
    SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK | // Frame Timeout
    SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | // PHY Header Error
    SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK | // Preamble Timeout
    SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK | // Receiver Start-up Timeout
    SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK;  // frame crc error

volatile bool g_new_message_received = false;
volatile bool g_rx_error = false;
volatile bool g_wait_to_msg_ok = false;
volatile uint16_t g_received_frame_len = 0;

mac_802_15_04 txMessage;
mac_802_15_04 rxMessage;
uint8_t func_code;
uint8_t rx_buffer[MAX_UWB_MESSAGE_LEN];
uint8_t frame_seq_nb = 0;
uint32_t status_reg = 0;
uint8_t waitConfigStartTime = 0;
bool wait_poll = true;
bool wait_ack = false;
bool wait_range = false;
bool wait_final = false;
bool send_config = false;
bool configInitatorMode = false;
bool configResponderMode = false;
bool m_rangingCycleCompleted = false;
int counter = 0;
int ret;
uint8_t UID = 0;
uint8_t INITIATOR_UID = 0;
int NUM_NODES = 0;
int WAIT_NUM = 0;
uint64_t myMacAddress = 0;
#define MAX_ANCHORS (MAX_NODES - 1)
const int DISCOVERY_WINDOW_MS = 200;
const int MAX_RESPONSE_DELAY_MS = 50;
uint64_t discovered_macs[MAX_ANCHORS];
int discovered_count = 0;
uint64_t poll_tx_ts, poll_rx_ts, range_tx_ts, ack_tx_ts, range_rx_ts;
uint32_t t_reply_1[MAX_NODES - 1];
uint64_t t_reply_2;
uint64_t t_round_1[MAX_NODES - 1];
uint32_t t_round_2[MAX_NODES - 1];
double tof, distance;
unsigned long previous_debug_millis = 0;
unsigned long current_debug_millis = 0;
unsigned long last_distance_publish = 0;
unsigned long update_interval = 200;

uint32_t tx_time;
uint64_t tx_ts;
int target_uids[MAX_NODES - 1];
RangingPartner known_devices[MAX_NODES];
int known_devices_count = 0;
unsigned long last_discovery_millis = 0;

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    g_received_frame_len = cb_data->datalength;
    g_new_message_received = true;
}

static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    g_rx_error = true;
}
static void tx_ok_cb(const dwt_cb_data_t *cb_data)
{
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    g_wait_to_msg_ok = false;
}

bool knowThisDevice(uint64_t mac)
{
    for (int i = 0; i < known_devices_count; i++)
    {
        if (known_devices[i].mac_address == mac)
        {
            return true;
        }
    }
    return false;
}

bool addDeviceToKnownDevices(uint64_t mac)
{
    if (known_devices_count < MAX_NODES)
    {
        known_devices[known_devices_count].mac_address = mac;
        known_devices[known_devices_count].distance = 0.0;
        known_devices[known_devices_count].uid = INITIATOR_UID + (known_devices_count + 1) * 4;
        known_devices_count++;
        return true;
    }
    return false;
}

void deleteKnownDeviceWithUID(uint8_t uid)
{
    int index = -1;
    for (int i = 0; i < known_devices_count; i++)
    {
        if (known_devices[i].uid == uid)
        {
            index = i;
            break;
        }
    }
    if (index == -1)
    {
        return;
    }
    for (int i = index; i < known_devices_count - 1; i++)
    {
        known_devices[i].mac_address = known_devices[i + 1].mac_address;
    }

    known_devices[known_devices_count - 1].mac_address = 0;
    known_devices[known_devices_count - 1].uid = 0;
    known_devices[known_devices_count - 1].distance = -1;
    known_devices_count--;
}

bool updateKnownDevices(uint64_t mac, uint8_t uid)
{
    for (int i = 0; i < known_devices_count; i++)
    {

        if (known_devices[i].mac_address == mac)
        {
            known_devices[i].uid = uid;
            return true;
        }
    }
    // If not found return false
    return false;
}

void updateDistance(uint8_t uid, double new_distance)
{
    for (int i = 0; i < known_devices_count; i++)
    {
        if (known_devices[i].uid == uid)
        {
            known_devices[i].distance = new_distance;
            return;
        }
    }
}

void print_frame_data(const uint8_t *data, uint16_t length)
{
    Serial.print("DEBUG RX (len=");
    Serial.print(length);
    Serial.print("): ");
    for (int i = 0; i < length; ++i)
    {
        if (data[i] < 0x10)
        {
            Serial.print("0");
        }
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
void set_target_uids()
{
    if (NUM_NODES <= 1)
        return;
    int target_idx = 0;
    if (UID != INITIATOR_UID)
    {
        target_uids[target_idx++] = INITIATOR_UID;
    }
    for (int i = 1; i < NUM_NODES; i++)
    {
        uint8_t responder_uid = INITIATOR_UID + (i * 4);
        if (responder_uid != UID)
        {
            if (target_idx < (NUM_NODES - 1))
            {
                target_uids[target_idx++] = responder_uid;
            }
        }
    }
}
void setRangingConfiguration(uint8_t initiatorUid, uint8_t myAssignedUid, uint8_t totalDevices)
{
    UID = myAssignedUid;
    INITIATOR_UID = initiatorUid;
    NUM_NODES = totalDevices;
    if (UID > INITIATOR_UID)
    {
        WAIT_NUM = (UID - INITIATOR_UID) / 4;
    }
    else
    {
        WAIT_NUM = 0;
    }
    set_target_uids();
}
void start_uwb()
{
    WiFi.mode(WIFI_MODE_STA);
    uint8_t mac[6];
    WiFi.macAddress(mac);
    // Die MAC-Adresse wird korrekt in der uint64_t-Variable gespeichert
    myMacAddress = ((uint64_t)mac[0] << 40) | ((uint64_t)mac[1] << 32) |
                   ((uint64_t)mac[2] << 24) | ((uint64_t)mac[3] << 16) |
                   ((uint64_t)mac[4] << 8) | (uint64_t)mac[5];

    UART_init();
    spiBegin(UWB_IRQ, UWB_RST);
    spiSelect(UWB_SS);
    delay(200);

    while (!dwt_checkidlerc())
    {
        UART_puts((char *)"IDLE FAILED\r\n");
        while (1)
            ;
    }
    dwt_softreset();
    delay(200);

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        UART_puts((char *)"INIT FAILED\r\n");
        while (1)
            ;
    }
    dwt_setleds(DWT_LEDS_DISABLE);
    if (dwt_configure(&config))
    {
        UART_puts((char *)"CONFIG FAILED\r\n");
        while (1)
            ;
    }

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(TX_TO_RX_DLY_UUS);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    Serial.print("My 48-bit UWB MAC: ");
    char macStr[13];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println(macStr);
    Serial.println("UWB started");
}

void configInitiator()
{
    dwt_setrxtimeout(RX_TIMEOUT_UUS);
    if (known_devices_count > 0)
    {
        uint8_t total_devices = known_devices_count + 1;
        for (int i = 0; i < known_devices_count; i++)
        {
            uint64_t responder_mac = known_devices[i].mac_address;
            uint8_t assigned_uid = INITIATOR_UID + ((i + 1) * 4);
            txMessage.buildRangingConfig(frame_seq_nb++, myMacAddress, responder_mac, INITIATOR_UID, assigned_uid, total_devices);
            dwt_writetxdata(txMessage.getLength(), txMessage.getBuffer(), 0);
            dwt_writetxfctrl(txMessage.getLength() + 2, 0, 0);
            dwt_starttx(DWT_START_TX_IMMEDIATE);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

            setRangingConfiguration(INITIATOR_UID, UID, total_devices);

            wait_ack = false;
            wait_final = false;
            counter = 0;
            delay(1);
        }
    }
    else if (known_devices_count == 0)
    {
        UID = (uint8_t)random(1, 50) * 4 + 2;
        INITIATOR_UID = UID;
    }
    configInitatorMode = true;
}
void configResponder()
{
    pinMode(UWB_IRQ, INPUT_PULLUP);
    dwt_setcallbacks(tx_ok_cb, rx_ok_cb, rx_err_cb, rx_err_cb, NULL, NULL);
    dwt_setinterrupt(int_mask, 0x0, DWT_ENABLE_INT_ONLY);
    attachInterrupt(digitalPinToInterrupt(UWB_IRQ), dwt_isr, RISING);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    configResponderMode = true;
}

void initiator()
{
    if (!wait_ack && !wait_final && (counter == 0))
    {
        wait_ack = true;
        txMessage.buildRangingMessage(frame_seq_nb, FUNC_CODE_POLL, 0, UID);
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(txMessage.getLength(), txMessage.getBuffer(), 0);
        dwt_writetxfctrl(txMessage.getLength(), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    }
    else
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        ;

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        dwt_readrxdata(rx_buffer, frame_len, 0);
        rxMessage.parse(rx_buffer, frame_len);

        if (rxMessage.getFunctionCode() == FUNC_CODE_DISCOVERY_BLINK)
        {
            uint64_t responderMac = rxMessage.getSourceMac();
            if (!knowThisDevice(responderMac))
            {
                addDeviceToKnownDevices(responderMac);
            }
            configInitatorMode = false;
            return;
        }
        // wrong uid
        if (rxMessage.getSourceUid() != target_uids[counter])
        {
            configInitatorMode = false;
            counter = 0;
            wait_ack = false;
            wait_final = false;
            return;
        }

        if (wait_ack)
        {
            poll_tx_ts = get_tx_timestamp_u64();
            t_round_1[counter] = get_rx_timestamp_u64() - poll_tx_ts;
            rxMessage.getTimestamp(&t_reply_1[counter]);
            counter++;
        }
        else
        {
            rxMessage.getTimestamp(&t_round_2[counter]);
            counter++;
        }
    }
    else
    { // Timeout oder Fehler
        if (known_devices_count > 0 && counter < (NUM_NODES - 1))
        {
            uint8_t failed_uid = target_uids[counter];
            if (failed_uid != INITIATOR_UID)
            {
                deleteKnownDeviceWithUID(failed_uid);
                configInitatorMode = false;
            }
        }
        wait_ack = false;
        wait_final = false;
        counter = 0;
        delay(1);
        return;
    }

    if (wait_ack && (counter == NUM_NODES - 1))
    {
        tx_time = (get_rx_timestamp_u64() + (RX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(tx_time);
        txMessage.buildRangingMessage(frame_seq_nb, FUNC_CODE_RANGE, 0, UID);
        dwt_writetxdata(txMessage.getLength(), txMessage.getBuffer(), 0);
        dwt_writetxfctrl(txMessage.getLength(), 0, 1);
        if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS)
        {
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                ;
            wait_ack = false;
            wait_final = true;
            counter = 0;
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        }
        return;
    }

    if (wait_final && (counter == NUM_NODES - 1))
    {
        current_debug_millis = millis();
        // Serial.print(current_debug_millis - previous_debug_millis);
        // Serial.print("ms\t");
        for (int i = 0; i < counter; i++)
        {
            t_reply_2 = get_tx_timestamp_u64() - (t_round_1[i] + poll_tx_ts);
            tof = ((double)t_round_1[i] * t_round_2[i] - (double)t_reply_1[i] * t_reply_2) /
                  ((double)t_round_1[i] + t_round_2[i] + t_reply_1[i] + t_reply_2);
            distance = tof * DWT_TIME_UNITS * SPEED_OF_LIGHT;
            bool valid = (distance > 0) && (distance < 1000);
            if (!valid)
            {
                configInitatorMode = false;
                return;
            }
            updateDistance(target_uids[i], distance);
            // Serial.print(target_uids[i]);
            // Serial.print("\t");
            // Serial.print(distance);
            // Serial.print(" m\t");
        }
        // Serial.println();
        m_rangingCycleCompleted = true;
        previous_debug_millis = current_debug_millis;
        counter = 0;
        wait_ack = false;
        wait_final = false;
        frame_seq_nb++;
        // delay(INTERVAL);
    }
    // }
}

void responder()
{
    if (func_code == FUNC_CODE_POLL || func_code == FUNC_CODE_RANGE)
    {
        if (rxMessage.getSourceUid() != INITIATOR_UID)
        {
            tx_time = (get_rx_timestamp_u64() + (random(800, RX_TO_TX_DLY_UUS) * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(tx_time);
            txMessage.buildDiscoveryBlink(rxMessage.getSequenceNumber(), 0, myMacAddress);
            dwt_writetxdata(txMessage.getLength() - 2, txMessage.getBuffer(), 0);
            dwt_writetxfctrl(txMessage.getLength(), 0, 0);
            if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS)
            {
                g_wait_to_msg_ok = true;
            }
            return;
        }
    }
    else if (func_code == FUNC_CODE_RANGING_CONFIG)
    {
        if (rxMessage.getDestinationMac() == myMacAddress)
        {
            uint8_t initiator_uid = 0, my_assigned_uid = 0, total_devices = 0;
            if (rxMessage.parseRangingConfig(initiator_uid, my_assigned_uid, total_devices))
            {
                setRangingConfiguration(initiator_uid, my_assigned_uid, total_devices);
                wait_poll = true;
                wait_range = false;
                counter = 0;
            }
        }
        return;
    }
    else if (func_code == FUNC_CODE_RESET)
    {
        wait_poll = true;
        wait_range = false;
        counter = 0;
        return;
    }
    // wrong uid
    if (rxMessage.getSourceUid() != target_uids[counter])
    {
        INITIATOR_UID = 0;
        wait_poll = true;
        wait_range = false;
        counter = 0;
        return;
    }

    if (wait_poll)
    {
        if (counter == 0)
        {
            poll_rx_ts = get_rx_timestamp_u64();
        }
        counter++;
    }
    else if (wait_range)
    {
        if (counter == 0)
        {
            range_rx_ts = get_rx_timestamp_u64();
        }
        counter++;
    }

    if (wait_poll && counter == WAIT_NUM)
    {
        tx_time = (get_rx_timestamp_u64() + (RX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(tx_time);
        tx_ts = (((uint64_t)(tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        txMessage.buildRangingMessage(frame_seq_nb, FUNC_CODE_ACK, 0, UID);
        txMessage.setTimestamp(tx_ts - poll_rx_ts);
        dwt_writetxdata(txMessage.getLength(), txMessage.getBuffer(), 0);
        dwt_writetxfctrl(txMessage.getLength(), 0, 1);
        if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS)
        {
            g_wait_to_msg_ok = true;
        }
    }

    if (wait_poll && counter == NUM_NODES - 1)
    {
        counter = 0;
        wait_poll = false;
        wait_range = true;
        return;
    }
    if (wait_range && counter == WAIT_NUM)
    {
        ack_tx_ts = get_tx_timestamp_u64();
        tx_time = (get_rx_timestamp_u64() + (RX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(tx_time);
        txMessage.buildRangingMessage(frame_seq_nb, FUNC_CODE_FINAL, 0, UID);
        txMessage.setTimestamp(range_rx_ts - ack_tx_ts);
        dwt_writetxdata(txMessage.getLength(), txMessage.getBuffer(), 0);
        dwt_writetxfctrl(txMessage.getLength(), 0, 1);
        if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS)
        {
            g_wait_to_msg_ok = true;
        }
    }

    if (wait_range && counter == NUM_NODES - 1)
    {
        counter = 0;
        wait_poll = true;
        wait_range = false;
        frame_seq_nb++;
        return;
    }
}

void responder_loop()
{
    if (!configResponderMode)
    {
        configResponder();
    }
    if (g_wait_to_msg_ok)
    {
        return;
    }
    if (g_new_message_received)
    {
        dwt_readrxdata(rx_buffer, g_received_frame_len, 0);
        rxMessage.parse(rx_buffer, g_received_frame_len);
        func_code = rxMessage.getFunctionCode();
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        responder();
        g_new_message_received = false;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
    else if (g_rx_error)
    {
        g_rx_error = false;
        wait_poll = true;
        wait_range = false;
        counter = 0;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
}

int formatUwbDataToString(char* output_buffer, size_t buffer_size)
{
    int offset = 0;
    char dist_buffer[15];
    if (buffer_size == 0)
    {
        return 0;
    }
    output_buffer[0] = '\0';
    struct timeval tv;
    gettimeofday(&tv, NULL); 
 
    uint64_t timestamp_ns = (uint64_t)tv.tv_sec * 1000000000L + (uint64_t)tv.tv_usec * 1000L;

    int chars_written = snprintf(output_buffer, buffer_size, "%llu;", timestamp_ns);

    if (chars_written < 0 || chars_written >= buffer_size) {
        return 0; 
    }
    offset += chars_written; 
    for (int i = 0; i < known_devices_count; i++)
    {
        size_t remaining_space = buffer_size - offset;
        
        chars_written = snprintf(output_buffer + offset, remaining_space, "%llx,",
                                 known_devices[i].mac_address);
        if (chars_written < 0 || chars_written >= remaining_space)
        {
            break; 
        }
        offset += chars_written;
        remaining_space -= chars_written;
        dtostrf(known_devices[i].distance, 1, 2, dist_buffer);
        chars_written = snprintf(output_buffer + offset, remaining_space, "%s;",
                                 dist_buffer);

        if (chars_written < 0 || chars_written >= remaining_space)
        {
            break; 
        }
        offset += chars_written;
    }
    return offset;
}

void initiator_loop()
{
    if (!configInitatorMode)
    {
        configInitiator();
    }
    if (millis() - last_distance_publish > update_interval - 25)
    {
        initiator();
        if (m_rangingCycleCompleted)
        {
            const int PAYLOAD_BUFFER_SIZE = 300;
            char my_payload[PAYLOAD_BUFFER_SIZE];
            int string_l = formatUwbDataToString(my_payload, PAYLOAD_BUFFER_SIZE);
            if (string_l > 0) 
            {   
                while (!mqtt_publish("uwb/data", my_payload));
                {
                }
            }
            last_distance_publish = millis();
            m_rangingCycleCompleted = false;
        }
    }
}

void uwb_sniffer_loop()
{
    if (!configResponderMode)
    {
        configResponder();
    }
    if (g_new_message_received)
    {
        dwt_readrxdata(rx_buffer, g_received_frame_len, 0);
        rxMessage.parse(rx_buffer, g_received_frame_len);
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        g_new_message_received = false;
        print_frame_data(rx_buffer, g_received_frame_len);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    else if (g_rx_error)
    {
        g_rx_error = false;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

void printdebug()
{
    Serial.print("g_wait_to_msg_ok ");
    Serial.println(g_wait_to_msg_ok);
    Serial.print("g_new_message_received ");
    Serial.println(g_new_message_received);
    Serial.print("g_rx_error ");
    Serial.println(g_rx_error);
    Serial.print("Status: ");
    Serial.println(dwt_read32bitreg(SYS_STATUS_ID));
}
