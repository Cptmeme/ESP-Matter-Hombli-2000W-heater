// What Matter's Basic Information cluster reports as manufacturer, model and serial number,
// which Apple Home shows in the heater's settings. Everything else comes from the SDK's
// default provider unchanged, including the vendor and product IDs used for pairing.
#include <esp_mac.h>
#include <esp_matter_providers.h>
#include <lib/support/CodeUtils.h>
#include <platform/ESP32/ConfigurationManagerImpl.h>
#include <platform/internal/GenericDeviceInstanceInfoProvider.ipp>
#include <stdio.h>
#include <string.h>

#include "app_priv.h"

#define DEVICE_VENDOR_NAME  "Hombli"
#define DEVICE_PRODUCT_NAME "Smart Convector Heater 2000W"

using namespace chip::DeviceLayer;

namespace {

class HeaterInfoProvider : public Internal::GenericDeviceInstanceInfoProvider<Internal::ESP32Config> {
public:
    HeaterInfoProvider() : GenericDeviceInstanceInfoProvider(ConfigurationManagerImpl::GetDefaultInstance()) {}

    CHIP_ERROR GetVendorName(char *buf, size_t size) override { return Copy(buf, size, DEVICE_VENDOR_NAME); }
    CHIP_ERROR GetProductName(char *buf, size_t size) override { return Copy(buf, size, DEVICE_PRODUCT_NAME); }

    // The chip's base MAC address, e.g. 404CCA42DAF4: unique per heater, and kept through
    // reflashing and factory resets. Not esp_efuse_mac_get_default(): on chips with a Thread
    // radio it writes the 8-byte EUI-64 form.
    CHIP_ERROR GetSerialNumber(char *buf, size_t size) override
    {
        uint8_t mac[6];
        VerifyOrReturnError(esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK, CHIP_ERROR_INTERNAL);
        char serial[13];
        snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return Copy(buf, size, serial);
    }

private:
    static CHIP_ERROR Copy(char *buf, size_t size, const char *value)
    {
        size_t len = strlen(value);
        VerifyOrReturnError(len < size, CHIP_ERROR_BUFFER_TOO_SMALL);
        memcpy(buf, value, len + 1);
        return CHIP_NO_ERROR;
    }
};

} // namespace

void app_device_info_init()
{
    static HeaterInfoProvider provider;
    esp_matter::set_custom_device_instance_info_provider(&provider);
}
