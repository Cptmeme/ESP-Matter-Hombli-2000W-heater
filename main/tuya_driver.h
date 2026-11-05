#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // Required for memcpy/memmove
#include <atomic>
#include <mutex>
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

    // Poll task
    void Poll();                    // Reads and handles the heater's reports
    void Service(bool may_query);   // Sends Matter's commands one at a time, runs the restore, asks for status
    bool HasReport() const { return m_have_report.load(); }

    // Matter thread. Each updates what Matter shows straight away; the poll task then
    // sends it to the heater.
    void SetSavedTemp(int temp);    // The setpoint to restore; call before any report reaches Matter
    void TurnOn();                  // Heat: on, in comfort
    void TurnOff();
    void SetTemp(int temp);
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

    // Matter commands, handed from the Matter thread to the poll task
    enum command_t : uint8_t { CMD_OFF, CMD_HEAT, CMD_TEMP, CMD_SCREEN };
    struct queued_t { command_t cmd; int value; int64_t at; };
    static const int QUEUE_LEN = 16;
    std::mutex m_queue_lock;
    queued_t m_queue[QUEUE_LEN];
    int m_queue_head;
    int m_queue_count;

    // Values Matter asked for that the heater hasn't confirmed yet (poll task only)
    enum { W_POWER, W_MODE, W_TEMP, W_SCREEN, W_COUNT };
    struct want_t { bool active; int value; };
    want_t m_want[W_COUNT];
    int m_reported[W_COUNT];        // Last reported value, -1 until reported. Screen: 1 = on
    bool m_seen[W_COUNT];           // A report for this value arrived since the flag was cleared

    // The write waiting for the heater's answer
    int m_write;                    // W_* or -1
    int m_write_value;
    int m_write_tries;
    bool m_write_queried;
    int64_t m_write_due;
    int64_t m_next_write_at;
    int64_t m_last_tx_at;
    int64_t m_power_on_at;
    int64_t m_last_off_at;
    int64_t m_next_query_at;

    // Restore after a power cut (see RunRestore)
    enum restore_step_t { RESTORE_ARMED, RESTORE_CHECK_MODE, RESTORE_DONE };
    restore_step_t m_restore;
    bool m_restore_writes;          // The pending comfort and setpoint come from the restore
    int64_t m_restore_due;
    int m_restore_tries;
    std::atomic<int> m_desired_temp;
    std::atomic<bool> m_have_report;

    void Queue(command_t cmd, int value);
    void ApplyQueued();
    void RunRestore(int64_t now);
    void RunWriter(int64_t now);
    void StartWrite(int w, int64_t now);
    void SendWrite(int w, int value);
    void SyncFromReported(int w);
    void CancelComfort();
    void FinishRestore();
    bool CanSend(int64_t now) const;
    void SendQuery();
    void ProcessPacket(const uint8_t *data, int len);
    void SendCommand(uint8_t dp_id, uint8_t type, const uint8_t *value, int len);
    void NotifyStateChange();
};
