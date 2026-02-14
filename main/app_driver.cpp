#include <esp_log.h>
#include <stdlib.h>
#include <string.h>
#include <esp_matter.h>
#include <app_priv.h>
#include <app/reporting/reporting.h> 
#include <platform/CHIPDeviceLayer.h>
#include "iot_button.h"
#include "button_gpio.h"
#include <app/server/Server.h>
#include <app/server/CommissioningWindowManager.h>

#include "tuya_driver.h"

using namespace chip::app::Clusters;
using namespace chip::app::Clusters::Thermostat;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t thermostat_endpoint_id;
extern uint16_t screen_endpoint_id;
static TuyaHeaterDriver heater;

// Global Temp for AAI
int16_t g_current_temp_int = 2000; 

#define BUTTON_GPIO_PIN 23

// --- POLL TASK ---
static void tuya_poll_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Tuya Poll Task Started");
    while (1) {
        heater.Poll(); 
        
        static int ticks = 0;
        if (ticks++ > 200) { 
            heater.SendHeartbeat();
            ticks = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- THREAD BRIDGE ---
struct AppEventData {
    heater_state_t state;
};

static void AppDriverUpdateTask(intptr_t context)
{
    AppEventData *data = (AppEventData *)context;
    if (!data) return;

    g_current_temp_int = data->state.current_temp * 100;
    MatterReportingAttributeChangeCallback(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::LocalTemperature::Id);

    esp_matter_attr_val_t target_val = esp_matter_int16(data->state.target_temp * 100);
    esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::OccupiedHeatingSetpoint::Id, &target_val);

    uint8_t matter_mode = (uint8_t)Thermostat::SystemModeEnum::kOff;
    if (data->state.power) matter_mode = (uint8_t)Thermostat::SystemModeEnum::kHeat;
    
    esp_matter_attr_val_t mode_val = esp_matter_enum8(matter_mode);
    esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &mode_val);

    uint16_t running_state = 0; // Idle
    if (data->state.power) {
        if (data->state.current_temp < (data->state.target_temp + 1)) {
            running_state = 1; // Heating
        }
    }
    esp_matter_attr_val_t run_val = esp_matter_bitmap16(running_state);
    esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::ThermostatRunningState::Id, &run_val);


    if (screen_endpoint_id != 0) {
        esp_matter_attr_val_t screen_val = esp_matter_bool(data->state.screen_on);
        esp_matter::attribute::report(screen_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &screen_val);
    }

    free(data);
}

static void tuya_state_change_callback(const heater_state_t *state)
{
    if (thermostat_endpoint_id == 0) return;
    AppEventData *data = (AppEventData *)malloc(sizeof(AppEventData));
    if (data) {
        data->state = *state;
        chip::DeviceLayer::PlatformMgr().ScheduleWork(AppDriverUpdateTask, (intptr_t)data);
    }
}

// --- FACTORY RESET HANDLER ---
static void tuya_reset_callback()
{
    ESP_LOGW(TAG, "Initiating Factory Reset due to Power Button sequence...");
    esp_matter::factory_reset();
}

static esp_err_t app_driver_thermostat_set_value(void *handle, esp_matter_attr_val_t *val, uint32_t attribute_id)
{
    if (attribute_id == Thermostat::Attributes::SystemMode::Id) {
        uint8_t mode = val->val.u8;
        if (mode == (uint8_t)Thermostat::SystemModeEnum::kOff) {
            heater.SetPower(false);
        } else {
            // Turn ON + Force High Mode (Combined Packet)
            heater.SetPowerAndMode(true, MODE_HIGH);
        }
    }
    else if (attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id) {
        heater.SetTemp(val->val.i16 / 100);
    }
    return ESP_OK;
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    if (endpoint_id == thermostat_endpoint_id && cluster_id == Thermostat::Id) {
        return app_driver_thermostat_set_value(driver_handle, val, attribute_id);
    }

    else if (endpoint_id == screen_endpoint_id) {
        if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
            bool on = val->val.b;
            ESP_LOGI(TAG, "Matter Command: Set Screen %s", on ? "ON" : "OFF");
            heater.SetScreen(on);
        }
    }
    return ESP_OK;
}

esp_err_t app_driver_thermostat_set_defaults(uint16_t endpoint_id) { return ESP_OK; }

app_driver_handle_t app_driver_thermostat_init()
{
    heater.Init(TUYA_TX_PIN, TUYA_RX_PIN);
    heater.SetStateCallback(tuya_state_change_callback);
    // Register the Reset Callback
    heater.SetResetCallback(tuya_reset_callback);
    
    xTaskCreate(tuya_poll_task, "tuya_poll", 4096, NULL, 5, NULL);
    return (app_driver_handle_t)1;
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button: Commissioning Window");
    chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (!commissionMgr.IsCommissioningWindowOpen()) {
        commissionMgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                                   chip::CommissioningWindowAdvertisement::kDnssdOnly);
    }
}

app_driver_handle_t app_driver_button_init()
{
    button_config_t btn_cfg = {0};
    button_gpio_config_t btn_gpio_cfg = { .gpio_num = BUTTON_GPIO_PIN, .active_level = 0 };
    button_handle_t btn_handle = NULL;
    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn_handle);
    if (err == ESP_OK && btn_handle) {
        iot_button_register_cb(btn_handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);
        return (app_driver_handle_t)btn_handle;
    }
    return NULL;
}