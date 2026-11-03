#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // Required for memcpy/memmove
#include <atomic>
#include "esp_err.h"

// --- TUYA CONSTANTS ---
#define TUYA_HEADER_0 0x55
#define TUYA_HEADER_1 0xAA

// DP IDs
#define DP_POWER    1
#define DP_SET_TEMP 2
#define DP_CUR_TEMP 3
#define DP_MODE     4   
#define DP_SCREEN   101

// Tuya Modes
#define MODE_COMFORT   0
#define MODE_ECO       1
#define MODE_ANTIFROST 2   // Where the heater starts after a power cut; has no setpoint

// Fixed buffer size to avoid Heap allocation (Thread Safe)
#define RX_BUF_SIZE 512

typedef struct {
    bool power;
    int target_temp;    
    int current_temp;   
    uint8_t mode;
    bool screen_on;       
} heater_state_t;

typedef void (*tuya_state_change_cb_t)(const heater_state_t *state);
typedef void (*tuya_reset_cb_t)(); 

class TuyaHeaterDriver {
public:
    TuyaHeaterDriver();

    esp_err_t Init(int tx_pin, int rx_pin);
    void Poll();
    void SendHeartbeat();
    void RunRestore();      // Poll task: restores comfort + setpoint after a power cut
    bool HasReport() const { return m_have_report.load(); }

    // Setters
    void SetSavedTemp(int temp);    // The setpoint to restore; call before any report reaches Matter
    void TurnOn();
    void SetPower(bool on);
    void SetMode(uint8_t mode);
    void SetTemp(int temp);
    void SetPowerAndMode(bool on, uint8_t mode);
    void SetScreen(bool on);

    void SetStateCallback(tuya_state_change_cb_t cb);
    void SetResetCallback(tuya_reset_cb_t cb);

    heater_state_t GetState() const { return m_state; }

private:
    heater_state_t m_state;
    tuya_state_change_cb_t m_callback;
    tuya_reset_cb_t m_reset_callback; 
    
    int m_uart_num;
    
    // --- MEMORY FIX: FIXED ARRAY INSTEAD OF VECTOR ---
    uint8_t rx_buffer[RX_BUF_SIZE];
    int rx_count;

    // Reset Detection Variables
    int64_t last_toggle_time;
    int toggle_count;

    // Restore after a power cut (see RunRestore)
    enum restore_step_t { RESTORE_ARMED, RESTORE_CHECK_MODE, RESTORE_SET_MODE, RESTORE_SET_TEMP, RESTORE_DONE };
    std::atomic<restore_step_t> m_restore;
    std::atomic<bool> m_heat_request;
    std::atomic<int> m_desired_temp;
    std::atomic<bool> m_have_report;
    int64_t m_restore_due;
    int m_restore_tries;
    bool m_restore_query_sent;
    int m_reported_mode;
    int m_reported_temp;    // -1 until the heater has reported one
    bool m_mode_seen;       // A report for this DP arrived since the flag was last cleared
    bool m_temp_seen;

    void ProcessPacket(const uint8_t *data, int len);
    void SendCommand(uint8_t dp_id, uint8_t type, const uint8_t *value, int len);
    void NotifyStateChange();
    void WriteTemp(int temp);
    void StartRestoreStep(restore_step_t step, int64_t delay_us);
    void FinishRestore();
};