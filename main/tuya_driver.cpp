#include "tuya_driver.h"
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>

// --- NEW: Required for vTaskDelay ---
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "TUYA_DRIVER";

#define BAUD_RATE 9600

// Restore after a power cut (see RunRestore)
#define RESTORE_WAKE_US   300000    // Turned on from Matter: let the MCU wake before the first write
#define RESTORE_SETTLE_US 500000    // Turned on at the heater: let it settle before asking its mode
#define RESTORE_GAP_US    300000    // Between comfort being confirmed and the setpoint write
#define RESTORE_WAIT_US   500000    // Wait for a report after a write or a query
#define RESTORE_MAX_TRIES 5

TuyaHeaterDriver::TuyaHeaterDriver() {
    m_callback = nullptr;
    m_reset_callback = nullptr;
    m_uart_num = UART_NUM_1;
    
    m_state.power = false;
    m_state.target_temp = 22;
    m_state.current_temp = 20;
    m_state.mode = MODE_COMFORT;
    m_state.screen_on = true;

    // Init Buffer Counter
    rx_count = 0;
    memset(rx_buffer, 0, RX_BUF_SIZE);

    // Reset detection init
    last_toggle_time = 0;
    toggle_count = 0;

    // Restore after a power cut
    m_restore = RESTORE_ARMED;
    m_heat_request = false;
    m_desired_temp = m_state.target_temp;
    m_have_report = false;
    m_restore_due = 0;
    m_restore_tries = 0;
    m_restore_query_sent = false;
    m_reported_mode = -1;
    m_reported_temp = -1;
    m_mode_seen = false;
    m_temp_seen = false;
}

esp_err_t TuyaHeaterDriver::Init(int tx_pin, int rx_pin) {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver with internal buffer (buffer size x2)
    esp_err_t err = uart_driver_install((uart_port_t)m_uart_num, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    
    err = uart_param_config((uart_port_t)m_uart_num, &uart_config);
    if (err != ESP_OK) return err;
    
    err = uart_set_pin((uart_port_t)m_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return err;
}

void TuyaHeaterDriver::SendCommand(uint8_t dp_id, uint8_t type, const uint8_t *value, int len) {
    // Fixed stack buffer for constructing commands (Max 64 bytes is plenty for Tuya)
    uint8_t frame[64];
    int idx = 0;

    frame[idx++] = TUYA_HEADER_0;
    frame[idx++] = TUYA_HEADER_1;
    frame[idx++] = 0x00; // Ver
    frame[idx++] = 0x06; // Command

    // Data Len = DP_ID(1) + Type(1) + Len(2) + Value(len)
    uint16_t data_len = 1 + 1 + 2 + len;
    frame[idx++] = (data_len >> 8) & 0xFF;
    frame[idx++] = data_len & 0xFF;

    frame[idx++] = dp_id;
    frame[idx++] = type;
    frame[idx++] = (len >> 8) & 0xFF;
    frame[idx++] = len & 0xFF;

    memcpy(&frame[idx], value, len);
    idx += len;
    
    uint8_t cs = 0;
    for (int i = 0; i < idx; i++) cs += frame[i];
    frame[idx++] = cs;
    
    uart_write_bytes((uart_port_t)m_uart_num, (const char*)frame, idx);
}

// --- UPDATED FUNCTION: Split Command with Delay ---
void TuyaHeaterDriver::SetPowerAndMode(bool on, uint8_t mode) {
    // 1. Send Power Command
    SetPower(on);
    
    // 2. Wait 300ms for the MCU to wake up and verify the checksum
    // The ESP32 task will pause here, allowing the heater to beep/turn on.
    vTaskDelay(pdMS_TO_TICKS(300)); 
    
    // 3. Send Mode Command
    // Only send if we are turning ON. If turning OFF, mode setting is usually ignored anyway.
    if (on) {
        SetMode(mode);
    }
}

void TuyaHeaterDriver::SendHeartbeat() {
    uint8_t query[] = {0x55, 0xAA, 0x00, 0x08, 0x00, 0x00, 0x07};
    uart_write_bytes((uart_port_t)m_uart_num, (const char*)query, 7);
}

void TuyaHeaterDriver::Poll() {
    // Read directly into the buffer at the current offset
    int remaining_space = RX_BUF_SIZE - rx_count;
    if (remaining_space <= 0) {
        // Safety: Buffer full, reset
        rx_count = 0;
        remaining_space = RX_BUF_SIZE;
    }

    // Read directly to static memory
    int len = uart_read_bytes((uart_port_t)m_uart_num, &rx_buffer[rx_count], remaining_space, pdMS_TO_TICKS(50));
    
    if (len > 0) {
        rx_count += len;
        
        // Loop to process all available packets
        while (rx_count >= 7) { 
            
            // 1. Check Header
            if (rx_buffer[0] != TUYA_HEADER_0 || rx_buffer[1] != TUYA_HEADER_1) {
                memmove(rx_buffer, &rx_buffer[1], rx_count - 1);
                rx_count--;
                continue;
            }
            
            // 2. Check Payload Length
            uint16_t payload_len = (rx_buffer[4] << 8) | rx_buffer[5];
            int total_len = 6 + payload_len + 1; 
            
            // Wait for full packet
            if (rx_count < total_len) {
                break; 
            }
            
            // 3. Verify Checksum
            uint8_t calc_cs = 0;
            for(int i=0; i < total_len - 1; i++) calc_cs += rx_buffer[i];
            
            if (calc_cs == rx_buffer[total_len - 1]) {
                ProcessPacket(rx_buffer, total_len);
            }
            
            // 4. Remove processed packet
            int remaining = rx_count - total_len;
            if (remaining > 0) {
                memmove(rx_buffer, &rx_buffer[total_len], remaining);
            }
            rx_count = remaining;
        }
    }
}

void TuyaHeaterDriver::ProcessPacket(const uint8_t *packet, int len) {
    if (packet[3] != 0x07) return; // Command Word (0x07 = Status Report)
    
    int pos = 6;
    int end = len - 1;
    bool first = !m_have_report.exchange(true);
    bool changed = false;
    
    while (pos < end) {
        if (pos + 4 > end) break;

        uint8_t dp_id = packet[pos];
        uint16_t data_len = (packet[pos+2] << 8) | packet[pos+3];
        int val_idx = pos + 4;
        
        if (val_idx + data_len > end) break;

        int32_t val = 0;
        if (data_len == 1) val = packet[val_idx];
        else if (data_len == 4) {
            val = (packet[val_idx] << 24) | (packet[val_idx+1] << 16) | 
                  (packet[val_idx+2] << 8) | packet[val_idx+3];
        }
        
        // --- FACTORY RESET & STATE LOGIC ---
        if (dp_id == DP_POWER) {
            bool new_power = (val == 1);
            if (m_state.power != new_power) {
                int64_t now = esp_timer_get_time();
                if (now - last_toggle_time < 3000000) { 
                    toggle_count++;
                } else {
                    toggle_count = 1; 
                }
                last_toggle_time = now;
                
                ESP_LOGI(TAG, "Power Toggle Detected! Count: %d/10", toggle_count);

                if (toggle_count >= 10) {
                    ESP_LOGW(TAG, "FACTORY RESET SEQUENCE DETECTED!");
                    if (m_reset_callback) m_reset_callback();
                    toggle_count = 0;
                }
                m_state.power = new_power; 
                changed = true; 
            }
        }
        else if (dp_id == DP_SET_TEMP) {
            m_reported_temp = val;
            m_temp_seen = true;
            // Until the restore has finished this can be the heater's power-on default,
            // which must not replace the saved setpoint in Matter.
            if (m_restore.load() == RESTORE_DONE && m_state.target_temp != val) { m_state.target_temp = val; changed = true; }
        }
        else if (dp_id == DP_CUR_TEMP) {
            if (m_state.current_temp != val) { m_state.current_temp = val; changed = true; }
        }
        else if (dp_id == DP_MODE) {
            m_reported_mode = val;
            m_mode_seen = true;
            if (m_state.mode != val) { m_state.mode = val; changed = true; }
        }
        else if (dp_id == DP_SCREEN) {
            // Inverted Logic: 0=On, 1=Off
            bool is_on = (val == 0);
            if (m_state.screen_on != is_on) { 
                m_state.screen_on = is_on; 
                changed = true; 
            }
        }
        pos += 4 + data_len;
    }
    
    // The first report always syncs Matter: after a power cut the heater is off, while
    // Matter may still remember Heat, and no value has changed from the driver's defaults.
    if (changed || first) NotifyStateChange();
}

void TuyaHeaterDriver::NotifyStateChange() {
    if (m_callback) m_callback(&m_state);
}

void TuyaHeaterDriver::SetPower(bool on) {
    uint8_t val = on ? 1 : 0;
    SendCommand(DP_POWER, 0x01, &val, 1);
    m_state.power = on;
    NotifyStateChange();
}
void TuyaHeaterDriver::WriteTemp(int temp) {
    uint8_t val[4];
    val[0] = (temp >> 24) & 0xFF;
    val[1] = (temp >> 16) & 0xFF;
    val[2] = (temp >> 8) & 0xFF;
    val[3] = temp & 0xFF;
    SendCommand(DP_SET_TEMP, 0x02, val, 4);
}
void TuyaHeaterDriver::SetTemp(int temp) {
    m_desired_temp = temp;
    WriteTemp(temp);
    m_state.target_temp = temp;
    NotifyStateChange();
}
void TuyaHeaterDriver::SetMode(uint8_t mode) {
    SendCommand(DP_MODE, 0x04, &mode, 1);
    m_state.mode = mode;
    NotifyStateChange();
}

void TuyaHeaterDriver::SetScreen(bool on) {
    // Inverted Logic: 0=On, 1=Off
    uint8_t val = on ? 0 : 1;
    SendCommand(DP_SCREEN, 0x01, &val, 1);
    m_state.screen_on = on;
    NotifyStateChange();
}

// --- RESTORE AFTER A POWER CUT ---
// A power cut leaves the heater off. Once turned on it is in anti-frost, which has no
// setpoint, and leaving anti-frost resets the setpoint to 25 C. So, the first time the
// heater is on after a boot:
//   - Turned on from Matter: set comfort, then the saved setpoint.
//   - Turned on at the heater, or already on: the same if it reports anti-frost. Any other
//     mode means it kept its settings (only the ESP restarted), so nothing is written.
// This never writes DP1, so the heater stays off until someone turns it on. It runs once
// per boot, so anti-frost chosen on the heater later on is left alone.

void TuyaHeaterDriver::SetSavedTemp(int temp) {
    m_desired_temp = temp;
    m_state.target_temp = temp;
}

void TuyaHeaterDriver::TurnOn() {
    if (m_restore.load() == RESTORE_DONE) {
        // Keeps its blocking delay: a Matter setpoint write that follows stays 300 ms behind DP1
        SetPowerAndMode(true, MODE_COMFORT);
    } else {
        SetPower(true);
        m_heat_request = true;
    }
}

void TuyaHeaterDriver::StartRestoreStep(restore_step_t step, int64_t delay_us) {
    m_restore = step;
    m_restore_tries = 0;
    m_restore_query_sent = false;
    m_restore_due = esp_timer_get_time() + delay_us;
}

void TuyaHeaterDriver::FinishRestore() {
    m_restore = RESTORE_DONE;
    // From here on the heater's setpoint counts again
    if (m_reported_temp >= 0 && m_state.target_temp != m_reported_temp) {
        m_state.target_temp = m_reported_temp;
        NotifyStateChange();
    }
}

void TuyaHeaterDriver::RunRestore() {
    bool heat = m_heat_request.exchange(false);
    int64_t now = esp_timer_get_time();
    restore_step_t step = m_restore.load();

    if (step == RESTORE_DONE) {
        if (heat) SetMode(MODE_COMFORT);   // Finished just as a Heat command came in
        return;
    }

    if (heat && (step == RESTORE_ARMED || step == RESTORE_CHECK_MODE)) {
        ESP_LOGI(TAG, "Restore: first Heat since boot, setting comfort and %d C", m_desired_temp.load());
        StartRestoreStep(RESTORE_SET_MODE, RESTORE_WAKE_US);
        return;
    }

    if (step == RESTORE_ARMED) {
        if (m_state.power) StartRestoreStep(RESTORE_CHECK_MODE, RESTORE_SETTLE_US);
        return;
    }

    if (step == RESTORE_CHECK_MODE) {
        if (!m_state.power) {
            m_restore = RESTORE_ARMED;
            return;
        }
        // Only a report after our query counts: an earlier one can predate the turn-on
        if (m_restore_tries > 0 && m_mode_seen) {
            if (m_reported_mode == MODE_ANTIFROST) {
                ESP_LOGW(TAG, "Restore: heater is in anti-frost, setting comfort and %d C", m_desired_temp.load());
                StartRestoreStep(RESTORE_SET_MODE, RESTORE_GAP_US);
            } else {
                ESP_LOGI(TAG, "Restore: heater kept its settings (mode %d)", m_reported_mode);
                FinishRestore();
            }
            return;
        }
        if (now < m_restore_due) return;
        if (m_restore_tries >= RESTORE_MAX_TRIES) {
            ESP_LOGW(TAG, "Restore: heater did not report its mode, giving up");
            FinishRestore();
            return;
        }
        m_restore_tries++;
        m_mode_seen = false;
        SendHeartbeat();
        m_restore_due = now + RESTORE_WAIT_US;
        return;
    }

    // RESTORE_SET_MODE and RESTORE_SET_TEMP: write, ask for a report if none comes, write again
    bool is_mode = (step == RESTORE_SET_MODE);
    int want = is_mode ? MODE_COMFORT : m_desired_temp.load();
    bool seen = is_mode ? m_mode_seen : m_temp_seen;
    int got = is_mode ? m_reported_mode : m_reported_temp;

    if (m_restore_tries > 0 && seen && got == want) {
        if (is_mode) {
            StartRestoreStep(RESTORE_SET_TEMP, RESTORE_GAP_US);
        } else {
            ESP_LOGI(TAG, "Restore: done, comfort at %d C", want);
            FinishRestore();
        }
        return;
    }
    if (now < m_restore_due) return;

    if (m_restore_tries > 0 && !m_restore_query_sent) {
        m_restore_query_sent = true;
        SendHeartbeat();
    } else if (m_restore_tries < RESTORE_MAX_TRIES) {
        m_restore_tries++;
        m_restore_query_sent = false;
        m_mode_seen = false;
        m_temp_seen = false;
        if (is_mode) {
            SetMode(MODE_COMFORT);
        } else {
            WriteTemp(want);
        }
    } else if (m_state.power) {
        ESP_LOGW(TAG, "Restore: heater did not take the %s, giving up", is_mode ? "mode" : "setpoint");
        FinishRestore();   // Matter shows the heater's real values
        return;
    } else {
        m_restore = RESTORE_ARMED;   // Turned off meanwhile: try again at the next turn-on
        return;
    }
    m_restore_due = now + RESTORE_WAIT_US;
}

void TuyaHeaterDriver::SetStateCallback(tuya_state_change_cb_t cb) {
    m_callback = cb;
}
void TuyaHeaterDriver::SetResetCallback(tuya_reset_cb_t cb) {
    m_reset_callback = cb;
}