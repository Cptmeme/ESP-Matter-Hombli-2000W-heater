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

TuyaHeaterDriver::TuyaHeaterDriver() {
    m_callback = nullptr;
    m_reset_callback = nullptr;
    m_uart_num = UART_NUM_1;
    
    m_state.power = false;
    m_state.target_temp = 22;
    m_state.current_temp = 20;
    m_state.mode = MODE_HIGH;
    m_state.screen_on = true;

    // Init Buffer Counter
    rx_count = 0;
    memset(rx_buffer, 0, RX_BUF_SIZE);

    // Reset detection init
    last_toggle_time = 0;
    toggle_count = 0;
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
            if (m_state.target_temp != val) { m_state.target_temp = val; changed = true; }
        }
        else if (dp_id == DP_CUR_TEMP) {
            if (m_state.current_temp != val) { m_state.current_temp = val; changed = true; }
        }
        else if (dp_id == DP_MODE) {
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
    
    if (changed) NotifyStateChange();
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
void TuyaHeaterDriver::SetTemp(int temp) {
    uint8_t val[4];
    val[0] = (temp >> 24) & 0xFF;
    val[1] = (temp >> 16) & 0xFF;
    val[2] = (temp >> 8) & 0xFF;
    val[3] = temp & 0xFF;
    SendCommand(DP_SET_TEMP, 0x02, val, 4);
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

void TuyaHeaterDriver::SetStateCallback(tuya_state_change_cb_t cb) {
    m_callback = cb;
}
void TuyaHeaterDriver::SetResetCallback(tuya_reset_cb_t cb) {
    m_reset_callback = cb;
}