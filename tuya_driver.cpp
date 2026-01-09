#include "tuya_driver.h"
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <string.h>

static const char *TAG = "TUYA_DRIVER";

#define UART_BUF_SIZE 512
#define BAUD_RATE 9600

TuyaHeaterDriver::TuyaHeaterDriver() {
    m_callback = nullptr;
    m_uart_num = UART_NUM_1;
    m_state.power = false;
    m_state.target_temp = 22;
    m_state.current_temp = 20;
    m_state.mode = MODE_HIGH;
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
    
    // Fixed: Casts to uart_port_t
    esp_err_t err = uart_driver_install((uart_port_t)m_uart_num, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    
    err = uart_param_config((uart_port_t)m_uart_num, &uart_config);
    if (err != ESP_OK) return err;
    
    err = uart_set_pin((uart_port_t)m_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return err;
}

void TuyaHeaterDriver::SendCommand(uint8_t dp_id, uint8_t type, std::vector<uint8_t> value) {
    std::vector<uint8_t> frame = {TUYA_HEADER_0, TUYA_HEADER_1, 0x00, 0x06};
    
    uint16_t data_len = 1 + 1 + 2 + value.size();
    frame.push_back((data_len >> 8) & 0xFF);
    frame.push_back(data_len & 0xFF);
    
    frame.push_back(dp_id);
    frame.push_back(type);
    frame.push_back((value.size() >> 8) & 0xFF);
    frame.push_back(value.size() & 0xFF);
    frame.insert(frame.end(), value.begin(), value.end());
    
    uint8_t cs = 0;
    for (size_t i = 0; i < frame.size(); i++) cs += frame[i];
    frame.push_back(cs);
    
    uart_write_bytes((uart_port_t)m_uart_num, (const char*)frame.data(), frame.size());
}

void TuyaHeaterDriver::SendHeartbeat() {
    uint8_t query[] = {0x55, 0xAA, 0x00, 0x08, 0x00, 0x00, 0x07};
    uart_write_bytes((uart_port_t)m_uart_num, (const char*)query, 7);
}

void TuyaHeaterDriver::Poll() {
    uint8_t data[128];
    int len = uart_read_bytes((uart_port_t)m_uart_num, data, 128, pdMS_TO_TICKS(50));
    
    if (len > 0) {
        // --- DEBUG LOGGING ---
        // This will verify if we are receiving data from Python
        ESP_LOGI(TAG, "RX %d bytes", len);
        ESP_LOG_BUFFER_HEX(TAG, data, len);
        // ---------------------

        rx_buffer.insert(rx_buffer.end(), data, data + len);
        
        while (rx_buffer.size() >= 7) {
            if (rx_buffer[0] != TUYA_HEADER_0 || rx_buffer[1] != TUYA_HEADER_1) {
                rx_buffer.erase(rx_buffer.begin());
                continue;
            }
            
            uint16_t payload_len = (rx_buffer[4] << 8) | rx_buffer[5];
            size_t total_len = 6 + payload_len + 1;
            
            if (rx_buffer.size() < total_len) break;
            
            std::vector<uint8_t> packet(rx_buffer.begin(), rx_buffer.begin() + total_len);
            uint8_t calc_cs = 0;
            for(size_t i=0; i<total_len-1; i++) calc_cs += packet[i];
            
            if (calc_cs == packet.back()) {
                ProcessPacket(packet);
            }
            
            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + total_len);
        }
    }
}

void TuyaHeaterDriver::ProcessPacket(const std::vector<uint8_t> &packet) {
    if (packet[3] != 0x07) return; 
    
    ESP_LOGI(TAG, "Parsing Packet...");
    int pos = 6;
    int end = packet.size() - 1;
    bool changed = false;
    
    while (pos < end) {
        uint8_t dp_id = packet[pos];
        uint16_t len = (packet[pos+2] << 8) | packet[pos+3];
        int val_idx = pos + 4;
        
        if (val_idx + len > end) break;

        int32_t val = 0;
        if (len == 1) val = packet[val_idx];
        else if (len == 4) {
            val = (packet[val_idx] << 24) | (packet[val_idx+1] << 16) | 
                  (packet[val_idx+2] << 8) | packet[val_idx+3];
        }
        
        ESP_LOGI(TAG, "DP ID: %d Value: %ld", dp_id, (long)val);

        if (dp_id == DP_POWER) {
            if (m_state.power != (val == 1)) { m_state.power = (val == 1); changed = true; }
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
        pos += 4 + len;
    }
    
    if (changed) NotifyStateChange();
}

void TuyaHeaterDriver::NotifyStateChange() {
    if (m_callback) m_callback(&m_state);
}

void TuyaHeaterDriver::SetPower(bool on) {
    std::vector<uint8_t> val = { (uint8_t)(on ? 1 : 0) };
    SendCommand(DP_POWER, 0x01, val); 
    m_state.power = on;
    NotifyStateChange();
}
void TuyaHeaterDriver::SetTemp(int temp) {
    std::vector<uint8_t> val = { (uint8_t)((temp >> 24) & 0xFF), (uint8_t)((temp >> 16) & 0xFF), (uint8_t)((temp >> 8) & 0xFF), (uint8_t)(temp & 0xFF) };
    SendCommand(DP_SET_TEMP, 0x02, val); 
    m_state.target_temp = temp;
    NotifyStateChange();
}
void TuyaHeaterDriver::SetMode(uint8_t mode) {
    std::vector<uint8_t> val = { mode };
    SendCommand(DP_MODE, 0x04, val); 
    m_state.mode = mode;
    NotifyStateChange();
}
void TuyaHeaterDriver::SetStateCallback(tuya_state_change_cb_t cb) {
    m_callback = cb;
}