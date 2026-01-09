#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <app/AttributeAccessInterface.h>
#include <app/AttributeAccessInterfaceRegistry.h>
#include <app/util/attribute-storage.h>

static const char *TAG = "app_main";
uint16_t thermostat_endpoint_id = 0;

extern int16_t g_current_temp_int; 

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

class LocalTempAccessor : public chip::app::AttributeAccessInterface
{
public:
    LocalTempAccessor() : AttributeAccessInterface(chip::Optional<chip::EndpointId>::Missing(), Thermostat::Id) {}
};

static LocalTempAccessor sLocalTempAccessor;

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg) {
    if (event->Type == chip::DeviceLayer::DeviceEventType::kCommissioningComplete) {
        ESP_LOGI(TAG, "Commissioning complete");
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data) { return ESP_OK; }

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    if (type == PRE_UPDATE) {
        return app_driver_attribute_update((app_driver_handle_t)priv_data, endpoint_id, cluster_id, attribute_id, val);
    }
    return ESP_OK;
}

extern "C" void app_main()
{
    nvs_flash_init();

    app_driver_handle_t thermostat_handle = app_driver_thermostat_init();
    app_driver_handle_t button_handle = app_driver_button_init();
    app_reset_button_register(button_handle);

    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);

    esp_matter::endpoint::thermostat::config_t thermostat_config = {};
    thermostat_config.thermostat.feature_flags = esp_matter::cluster::thermostat::feature::heating::get_id();

    endpoint_t *endpoint = esp_matter::endpoint::thermostat::create(node, &thermostat_config, ENDPOINT_FLAG_NONE, thermostat_handle);
    thermostat_endpoint_id = endpoint::get_id(endpoint);

    esp_matter::cluster_t *cluster = esp_matter::cluster::get(endpoint, Thermostat::Id);
    
    // Create attributes if they don't exist (LocalTemperature is already created by default)
    if (cluster) {
        // System Mode
        if (!esp_matter::attribute::get(cluster, Thermostat::Attributes::SystemMode::Id)) {
            esp_matter::attribute::create(cluster, Thermostat::Attributes::SystemMode::Id, ATTRIBUTE_FLAG_NONVOLATILE | ATTRIBUTE_FLAG_WRITABLE, esp_matter_enum8(0));
        }
        
        // Heating Setpoint
        if (!esp_matter::attribute::get(cluster, Thermostat::Attributes::OccupiedHeatingSetpoint::Id)) {
            esp_matter::attribute::create(cluster, Thermostat::Attributes::OccupiedHeatingSetpoint::Id, ATTRIBUTE_FLAG_NONVOLATILE | ATTRIBUTE_FLAG_WRITABLE, esp_matter_int16(2000));
        }

        // Running State
        if (!esp_matter::attribute::get(cluster, Thermostat::Attributes::ThermostatRunningState::Id)) {
            esp_matter::attribute::create(cluster, Thermostat::Attributes::ThermostatRunningState::Id, ATTRIBUTE_FLAG_NULLABLE, esp_matter_bitmap16(0));
        }

        // Force Heating Only Features
        esp_matter_attr_val_t feature_val = esp_matter_bitmap32(1); 
        attribute::update(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::FeatureMap::Id, &feature_val);
        
        esp_matter_attr_val_t seq_val = esp_matter_enum8(2); 
        attribute::update(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::ControlSequenceOfOperation::Id, &seq_val);
    }

    // Register AAI to intercept LocalTemperature reads
    chip::app::AttributeAccessInterfaceRegistry::Instance().Register(&sLocalTempAccessor);

    #if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
    #endif

    esp_matter::start(app_event_cb);
    app_driver_thermostat_set_defaults(thermostat_endpoint_id);

    #if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::init();
    #endif
}