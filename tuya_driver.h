#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <vector>
#include "esp_err.h"

// --- TUYA CONSTANTS ---
#define TUYA_HEADER_0 0x55
#define TUYA_HEADER_1 0xAA

// DP IDs
#define DP_POWER    1
#define DP_SET_TEMP 2
#define DP_CUR_TEMP 3
#define DP_MODE     4   

// Tuya Modes (Fixed Naming)
#define MODE_HIGH 0
#define MODE_LOW  1
#define MODE_ECO  2

// State Struct
typedef struct {
    bool power;
    int target_temp;    
    int current_temp;   
    uint8_t mode;       
} heater_state_t;

// Callback Type
typedef void (*tuya_state_change_cb_t)(const heater_state_t *state);

class TuyaHeaterDriver {
public:
    TuyaHeaterDriver();

    esp_err_t Init(int tx_pin, int rx_pin);

    // Main polling function
    void Poll();
    
    // Heartbeat/Query function
    void SendHeartbeat();

    // Setters
    void SetPower(bool on);
    void SetMode(uint8_t mode);
    void SetTemp(int temp);

    void SetStateCallback(tuya_state_change_cb_t cb);
    heater_state_t GetState() const { return m_state; }

private:
    heater_state_t m_state;
    tuya_state_change_cb_t m_callback;
    int m_uart_num;
    std::vector<uint8_t> rx_buffer;

    void ProcessPacket(const std::vector<uint8_t> &packet);
    void SendCommand(uint8_t dp_id, uint8_t type, std::vector<uint8_t> value);
    void NotifyStateChange();
};