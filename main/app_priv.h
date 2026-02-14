#pragma once

#include <esp_err.h>
#include <esp_matter.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

// PIN CONFIG (Non-Conflicting with Console)
#define TUYA_TX_PIN 16
#define TUYA_RX_PIN 17

typedef void *app_driver_handle_t;

app_driver_handle_t app_driver_thermostat_init();
app_driver_handle_t app_driver_button_init();

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val);

esp_err_t app_driver_thermostat_set_defaults(uint16_t endpoint_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG() { .radio_mode = RADIO_MODE_NATIVE, }
#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG() { .host_connection_mode = HOST_CONNECTION_MODE_NONE, }
#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG() { .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, }
#endif