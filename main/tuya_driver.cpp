#include "tuya_driver.h"
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>

#include <freertos/FreeRTOS.h>

static const char *TAG = "TUYA_DRIVER";

#define BAUD_RATE 9600

// Writes go to the heater one at a time, each waiting for the heater to report the value back
#define FRAME_GAP_US      100000    // Between any two frames: a frame right behind another can get lost
#define WRITE_GAP_US      300000    // After a confirmed write, before the next one
#define WRITE_WAIT_US     500000    // Wait for the reply to a write, then ask, then write again
#define WRITE_MAX_TRIES   5
#define WAKE_US           300000    // After the heater comes on, before writing its mode
#define OFF_WINS_US       2000000   // Heat this soon after Off is ignored (see ApplyQueued)

// Status queries
#define QUERY_FIRST_US    2000000   // Until the heater first answers
#define QUERY_EVERY_US    20000000

// Restore after a power cut (see RunRestore)
#define RESTORE_SETTLE_US 500000    // Turned on at the heater: let it settle before asking its mode
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

    m_queue_head = 0;
    m_queue_count = 0;
    for (int w = 0; w < W_COUNT; w++) {
        m_want[w] = {false, 0};
        m_reported[w] = -1;
        m_seen[w] = false;
    }
    m_write = -1;
    m_write_value = 0;
    m_write_tries = 0;
    m_write_queried = false;
    m_write_due = 0;
    m_next_write_at = 0;
    m_last_tx_at = -FRAME_GAP_US;
    m_power_on_at = 0;
    m_last_off_at = -10LL * OFF_WINS_US;
    m_next_query_at = 0;

    m_restore = RESTORE_ARMED;
    m_restore_writes = false;
    m_restore_due = 0;
    m_restore_tries = 0;
    m_desired_temp = m_state.target_temp;
    m_have_report = false;
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
    m_last_tx_at = esp_timer_get_time();
}

void TuyaHeaterDriver::SendQuery() {
    uint8_t query[] = {0x55, 0xAA, 0x00, 0x08, 0x00, 0x00, 0x07};
    uart_write_bytes((uart_port_t)m_uart_num, (const char*)query, 7);
    m_last_tx_at = esp_timer_get_time();
}

bool TuyaHeaterDriver::CanSend(int64_t now) const {
    return now - m_last_tx_at >= FRAME_GAP_US;
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

// While Matter is waiting for the heater to take a value, reports for it only update
// m_reported: Matter keeps showing what it asked for. Once the write is confirmed or given
// up, SyncFromReported shows the heater's real value.
void TuyaHeaterDriver::ProcessPacket(const uint8_t *packet, int len) {
    if (packet[3] != 0x07) return; // Command Word (0x07 = Status Report)

    int pos = 6;
    int end = len - 1;
    bool first = !m_have_report.exchange(true);
    bool changed = false;
    int64_t now = esp_timer_get_time();

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

        if (dp_id == DP_POWER) {
            int was = m_reported[W_POWER];
            bool new_power = (val == 1);
            m_reported[W_POWER] = new_power ? 1 : 0;
            m_seen[W_POWER] = true;
            if (new_power && was != 1) m_power_on_at = now;
            if (!m_want[W_POWER].active) {
                // Turned off at the heater: a comfort write still waiting is out of date
                if (!new_power && was == 1) CancelComfort();
                if (m_state.power != new_power) {
                    // --- FACTORY RESET: power toggled on the heater 10x within 3s windows ---
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
        }
        else if (dp_id == DP_SET_TEMP) {
            m_reported[W_TEMP] = val;
            m_seen[W_TEMP] = true;
            // Until the restore has finished this can be the heater's power-on default,
            // which must not replace the saved setpoint in Matter.
            if (m_restore == RESTORE_DONE && !m_want[W_TEMP].active && m_state.target_temp != val) {
                m_state.target_temp = val;
                changed = true;
            }
        }
        else if (dp_id == DP_CUR_TEMP) {
            if (m_state.current_temp != val) { m_state.current_temp = val; changed = true; }
        }
        else if (dp_id == DP_MODE) {
            m_reported[W_MODE] = val;
            m_seen[W_MODE] = true;
            if (!m_want[W_MODE].active && m_state.mode != val) { m_state.mode = val; changed = true; }
        }
        else if (dp_id == DP_SCREEN) {
            // Inverted Logic: 0=On, 1=Off
            bool is_on = (val == 0);
            m_reported[W_SCREEN] = is_on ? 1 : 0;
            m_seen[W_SCREEN] = true;
            if (!m_want[W_SCREEN].active && m_state.screen_on != is_on) {
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

// --- MATTER COMMANDS ---
// Apple Home sends a scene as several writes at once, for example a setpoint, Heat and Off.
// Writing each to the heater as it arrived put frames back to back, and the heater could
// miss one (an Off, leaving it on). Now each command only updates what Matter shows and is
// queued; the poll task sends the result one write at a time.

void TuyaHeaterDriver::Queue(command_t cmd, int value) {
    std::lock_guard<std::mutex> lock(m_queue_lock);
    if (m_queue_count == QUEUE_LEN) {   // Full: drop the oldest, the newest counts most
        m_queue_head = (m_queue_head + 1) % QUEUE_LEN;
        m_queue_count--;
    }
    m_queue[(m_queue_head + m_queue_count) % QUEUE_LEN] = {cmd, value, esp_timer_get_time()};
    m_queue_count++;
}

void TuyaHeaterDriver::TurnOn() {
    m_state.power = true;
    NotifyStateChange();
    Queue(CMD_HEAT, 1);
}

void TuyaHeaterDriver::TurnOff() {
    m_state.power = false;
    NotifyStateChange();
    Queue(CMD_OFF, 0);
}

void TuyaHeaterDriver::SetTemp(int temp) {
    m_state.target_temp = temp;
    NotifyStateChange();
    Queue(CMD_TEMP, temp);
}

void TuyaHeaterDriver::SetScreen(bool on) {
    m_state.screen_on = on;
    NotifyStateChange();
    Queue(CMD_SCREEN, on ? 1 : 0);
}

void TuyaHeaterDriver::ApplyQueued() {
    queued_t cmds[QUEUE_LEN];
    int n;
    {
        std::lock_guard<std::mutex> lock(m_queue_lock);
        n = m_queue_count;
        for (int i = 0; i < n; i++) cmds[i] = m_queue[(m_queue_head + i) % QUEUE_LEN];
        m_queue_head = 0;
        m_queue_count = 0;
    }

    for (int i = 0; i < n; i++) {
        const queued_t &c = cmds[i];
        switch (c.cmd) {
        case CMD_OFF:
            ESP_LOGI(TAG, "Matter: Off");
            m_last_off_at = c.at;
            m_want[W_POWER] = {true, 0};
            CancelComfort();
            break;

        case CMD_HEAT:
            // A scene that turns the heater off can send Heat as well, just before or after
            // the Off. Off wins, whichever order they come in.
            if (c.at - m_last_off_at < OFF_WINS_US) {
                ESP_LOGW(TAG, "Matter: Heat ignored, Off came %lld ms before it", (long long)((c.at - m_last_off_at) / 1000));
                if (m_state.power) {
                    m_state.power = false;
                    NotifyStateChange();
                }
                break;
            }
            ESP_LOGI(TAG, "Matter: Heat");
            m_want[W_POWER] = {true, 1};
            m_want[W_MODE] = {true, MODE_COMFORT};
            if (m_restore != RESTORE_DONE) {
                ESP_LOGI(TAG, "Restore: first Heat since boot, setting comfort and %d C", m_desired_temp.load());
                m_want[W_TEMP] = {true, m_desired_temp.load()};
                m_restore = RESTORE_DONE;
                m_restore_writes = true;
            }
            break;

        case CMD_TEMP:
            ESP_LOGI(TAG, "Matter: setpoint %d C", c.value);
            m_desired_temp = c.value;
            m_want[W_TEMP] = {true, c.value};   // Sent once the heater is on
            break;

        case CMD_SCREEN: {
            // The heater ignores the screen while it is off and keeps it on, so the switch
            // goes back to showing that rather than waiting for writes that can't work
            bool off = m_want[W_POWER].active ? m_want[W_POWER].value == 0 : m_reported[W_POWER] == 0;
            if (off) {
                ESP_LOGI(TAG, "Matter: screen %s ignored, the heater is off", c.value ? "on" : "off");
                m_want[W_SCREEN].active = false;
                SyncFromReported(W_SCREEN);
            } else {
                m_want[W_SCREEN] = {true, c.value};
            }
            break;
        }
        }
    }
}

// Drops a comfort write that is still waiting. If it came from the restore, the restore runs
// again at the next turn-on.
void TuyaHeaterDriver::CancelComfort() {
    m_want[W_MODE].active = false;
    if (m_restore_writes) {
        m_restore_writes = false;
        m_restore = RESTORE_ARMED;
        m_want[W_TEMP].active = false;
    }
}

void TuyaHeaterDriver::Service(bool may_query) {
    int64_t now = esp_timer_get_time();
    ApplyQueued();
    RunRestore(now);
    RunWriter(now);

    if (may_query && m_write < 0 && now >= m_next_query_at && CanSend(now)) {
        SendQuery();
        m_next_query_at = now + (m_have_report ? QUERY_EVERY_US : QUERY_FIRST_US);
    }
}

void TuyaHeaterDriver::SendWrite(int w, int value) {
    uint8_t b;
    switch (w) {
    case W_POWER:
        b = value ? 1 : 0;
        SendCommand(DP_POWER, 0x01, &b, 1);
        break;
    case W_MODE:
        b = (uint8_t)value;
        SendCommand(DP_MODE, 0x04, &b, 1);
        break;
    case W_TEMP: {
        uint8_t v[4] = {(uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value};
        SendCommand(DP_SET_TEMP, 0x02, v, 4);
        break;
    }
    case W_SCREEN:
        b = value ? 0 : 1;   // Inverted Logic: 0=On, 1=Off
        SendCommand(DP_SCREEN, 0x01, &b, 1);
        break;
    }
}

static const char *const s_write_names[] = {"power", "mode", "setpoint", "screen"};

void TuyaHeaterDriver::StartWrite(int w, int64_t now) {
    m_write = w;
    m_write_value = m_want[w].value;
    m_write_tries = 1;
    m_write_queried = false;
    m_seen[w] = false;
    ESP_LOGI(TAG, "Writing %s = %d", s_write_names[w], m_write_value);
    SendWrite(w, m_write_value);
    m_write_due = now + WRITE_WAIT_US;
}

void TuyaHeaterDriver::SyncFromReported(int w) {
    bool changed = false;
    int r = m_reported[w];
    if (r < 0) return;
    switch (w) {
    case W_POWER:
        if (m_state.power != (r == 1)) { m_state.power = (r == 1); changed = true; }
        break;
    case W_MODE:
        if (m_state.mode != r) { m_state.mode = r; changed = true; }
        break;
    case W_TEMP:
        if (m_restore == RESTORE_DONE && m_state.target_temp != r) { m_state.target_temp = r; changed = true; }
        break;
    case W_SCREEN:
        if (m_state.screen_on != (r == 1)) { m_state.screen_on = (r == 1); changed = true; }
        break;
    }
    if (changed) NotifyStateChange();
}

// Sends one write at a time and waits for the heater to report the written value. No reply
// in time: ask for a status report, then write again. Power goes first and stops any other
// write in progress. Comfort and the setpoint wait until the heater is on.
void TuyaHeaterDriver::RunWriter(int64_t now) {
    if (m_write >= 0) {
        int w = m_write;
        if (!m_want[w].active || m_want[w].value != m_write_value ||
            (w != W_POWER && m_want[W_POWER].active)) {
            m_write = -1;   // Replaced or cancelled; a value still wanted is written again below
        } else if (m_seen[w] && m_reported[w] == m_write_value) {
            ESP_LOGI(TAG, "Heater took %s = %d", s_write_names[w], m_write_value);
            m_want[w].active = false;
            m_write = -1;
            m_next_write_at = now + WRITE_GAP_US;
            SyncFromReported(w);
            if (m_restore_writes && !m_want[W_MODE].active && !m_want[W_TEMP].active) {
                ESP_LOGI(TAG, "Restore: done");
                m_restore_writes = false;
            }
            return;
        } else if (now < m_write_due || !CanSend(now)) {
            return;
        } else if (!m_write_queried) {
            SendQuery();
            m_write_queried = true;
            m_write_due = now + WRITE_WAIT_US;
            return;
        } else if (m_write_tries < WRITE_MAX_TRIES) {
            m_write_tries++;
            m_write_queried = false;
            m_seen[w] = false;
            SendWrite(w, m_write_value);
            m_write_due = now + WRITE_WAIT_US;
            return;
        } else {
            ESP_LOGW(TAG, "Heater did not take %s = %d, giving up", s_write_names[w], m_write_value);
            m_want[w].active = false;
            m_write = -1;
            m_next_write_at = now + WRITE_GAP_US;
            if (w == W_POWER && m_write_value == 1) CancelComfort();   // Never came on
            if (w == W_MODE && m_want[W_TEMP].active) {
                // Still in anti-frost, which has no setpoint
                m_want[W_TEMP].active = false;
                SyncFromReported(W_TEMP);
            }
            if (w == W_MODE || w == W_TEMP) m_restore_writes = false;
            SyncFromReported(w);
            return;
        }
    }

    if (now < m_next_write_at || !CanSend(now)) return;

    if (m_want[W_POWER].active) {
        StartWrite(W_POWER, now);
        return;
    }
    bool on = (m_reported[W_POWER] == 1);
    if (on && m_want[W_MODE].active) {
        if (now - m_power_on_at >= WAKE_US) StartWrite(W_MODE, now);
        return;
    }
    if (on && m_want[W_TEMP].active) {
        StartWrite(W_TEMP, now);
        return;
    }
    if (m_want[W_SCREEN].active) {
        if (m_reported[W_POWER] == 0) {   // Turned off meanwhile: see CMD_SCREEN
            m_want[W_SCREEN].active = false;
            SyncFromReported(W_SCREEN);
            return;
        }
        StartWrite(W_SCREEN, now);
    }
}

// --- RESTORE AFTER A POWER CUT ---
// A power cut leaves the heater off. Once turned on it is in anti-frost, which has no
// setpoint, and leaving anti-frost resets the setpoint to 25 C. So, the first time the
// heater is on after a boot:
//   - Turned on from Matter: set comfort, then the saved setpoint (see CMD_HEAT).
//   - Turned on at the heater, or already on: the same if it reports anti-frost. Any other
//     mode means it kept its settings (only the ESP restarted), so nothing is written.
// This never writes power, so the heater stays off until someone turns it on. It runs once
// per boot, so anti-frost chosen on the heater later on is left alone.

void TuyaHeaterDriver::SetSavedTemp(int temp) {
    m_desired_temp = temp;
    m_state.target_temp = temp;
}

void TuyaHeaterDriver::FinishRestore() {
    m_restore = RESTORE_DONE;
    // From here on the heater's setpoint counts again
    if (!m_want[W_TEMP].active) SyncFromReported(W_TEMP);
}

void TuyaHeaterDriver::RunRestore(int64_t now) {
    switch (m_restore) {
    case RESTORE_DONE:
        return;

    case RESTORE_ARMED:
        if (m_reported[W_POWER] == 1 && !m_want[W_POWER].active) {
            m_restore = RESTORE_CHECK_MODE;
            m_restore_tries = 0;
            m_restore_due = now + RESTORE_SETTLE_US;
        }
        return;

    case RESTORE_CHECK_MODE:
        if (m_reported[W_POWER] != 1) {
            m_restore = RESTORE_ARMED;
            return;
        }
        // Only a report after our query counts: an earlier one can predate the turn-on
        if (m_restore_tries > 0 && m_seen[W_MODE]) {
            if (m_reported[W_MODE] == MODE_ANTIFROST) {
                ESP_LOGW(TAG, "Restore: heater is in anti-frost, setting comfort and %d C", m_desired_temp.load());
                m_want[W_MODE] = {true, MODE_COMFORT};
                m_want[W_TEMP] = {true, m_desired_temp.load()};
                m_restore = RESTORE_DONE;
                m_restore_writes = true;
            } else {
                ESP_LOGI(TAG, "Restore: heater kept its settings (mode %d)", m_reported[W_MODE]);
                FinishRestore();
            }
            return;
        }
        if (now < m_restore_due || !CanSend(now)) return;
        if (m_restore_tries >= RESTORE_MAX_TRIES) {
            ESP_LOGW(TAG, "Restore: heater did not report its mode, giving up");
            FinishRestore();
            return;
        }
        m_restore_tries++;
        m_seen[W_MODE] = false;
        SendQuery();
        m_restore_due = now + WRITE_WAIT_US;
        return;
    }
}

void TuyaHeaterDriver::SetStateCallback(tuya_state_change_cb_t cb) {
    m_callback = cb;
}
void TuyaHeaterDriver::SetResetCallback(tuya_reset_cb_t cb) {
    m_reset_callback = cb;
}
