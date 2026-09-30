#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"

/**
 * @brief Helper class to handle NVS (Non-Volatile Storage) operations.
 */
class NvsStorage {
public:
    static esp_err_t init_default();
    
    explicit NvsStorage(const char* namespace_name);
    ~NvsStorage();

    bool write_blob(const char* key, const void* data, size_t length);
    bool read_blob(const char* key, void* data, size_t length);

private:
    const char* namespace_;
};