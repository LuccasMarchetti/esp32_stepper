#include "NvsStorage.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char* TAG = "NvsStorage";

esp_err_t NvsStorage::init_default() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

NvsStorage::NvsStorage(const char* namespace_name) : namespace_(namespace_name) {}

NvsStorage::~NvsStorage() = default;

bool NvsStorage::write_blob(const char* key, const void* data, size_t length) {
    nvs_handle_t handle;
    if (nvs_open(namespace_, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }

    esp_err_t err = nvs_set_blob(handle, key, data, length);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    
    nvs_close(handle);
    return (err == ESP_OK);
}

bool NvsStorage::read_blob(const char* key, void* data, size_t length) {
    nvs_handle_t handle;
    if (nvs_open(namespace_, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    size_t required_len = length;
    esp_err_t err = nvs_get_blob(handle, key, data, &required_len);
    nvs_close(handle);

    return (err == ESP_OK && required_len == length);
}