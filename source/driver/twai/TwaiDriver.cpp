#include "TwaiDriver.hpp"
#include <cstring>

namespace Twai {

TwaiDriver::TwaiDriver() = default;

TwaiDriver::~TwaiDriver() {
    end();
}

bool TwaiDriver::begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t baudRate, uint32_t queueSize) {
    if (m_initialized) {
        ESP_LOGW(TAG, "Driver already initialized");
        return false;
    }

    // Configure TWAI node for ESP-IDF 6.x
    twai_onchip_node_config_t node_config = {};
    node_config.io_cfg.tx = txPin;
    node_config.io_cfg.rx = rxPin;
    node_config.bit_timing.bitrate = baudRate;
    node_config.tx_queue_depth = queueSize;
    node_config.rx_queue_depth = queueSize;

    // Create TWAI node instance
    if (twai_new_node_onchip(&node_config, &m_nodeHandle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TWAI node");
        return false;
    }

    // Enable TWAI node operation
    if (twai_node_enable(m_nodeHandle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable TWAI node");
        twai_node_delete(m_nodeHandle);
        m_nodeHandle = nullptr;
        return false;
    }

    // Create FreeRTOS queue for non-blocking TX requests
    m_txQueue = xQueueCreate(queueSize, sizeof(TwaiFrame));
    if (!m_txQueue) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        twai_node_disable(m_nodeHandle);
        twai_node_delete(m_nodeHandle);
        m_nodeHandle = nullptr;
        return false;
    }

    m_initialized = true;

    xTaskCreate(rxTaskWorker, "twai_rx_task", TASK_STACK_SIZE, this, TASK_PRIORITY, &m_rxTaskHandle);
    xTaskCreate(txTaskWorker, "twai_tx_task", TASK_STACK_SIZE, this, TASK_PRIORITY, &m_txTaskHandle);

    ESP_LOGI(TAG, "TWAI driver (esp_driver_twai v6.x) initialized successfully");
    return true;
}

void TwaiDriver::end() {
    if (!m_initialized) return;

    m_initialized = false;

    if (m_rxTaskHandle) {
        vTaskDelete(m_rxTaskHandle);
        m_rxTaskHandle = nullptr;
    }

    if (m_txTaskHandle) {
        vTaskDelete(m_txTaskHandle);
        m_txTaskHandle = nullptr;
    }

    if (m_txQueue) {
        vQueueDelete(m_txQueue);
        m_txQueue = nullptr;
    }

    if (m_nodeHandle) {
        twai_node_disable(m_nodeHandle);
        twai_node_delete(m_nodeHandle);
        m_nodeHandle = nullptr;
    }

    ESP_LOGI(TAG, "TWAI driver stopped and deleted");
}

void TwaiDriver::setOnReceive(RxCallback callback) {
    m_rxCallback = callback;
}

bool TwaiDriver::send(const TwaiFrame& frame) {
    if (!m_initialized || !m_txQueue) return false;

    if (frame.length > 8) {
        ESP_LOGE(TAG, "Payload length exceeds CAN limit (8 bytes)");
        return false;
    }

    // Non-blocking queue push (ticksToWait = 0)
    if (xQueueSend(m_txQueue, &frame, 0) != pdPASS) {
        ESP_LOGE(TAG, "TX Queue full, dropping frame ID: 0x%lX", (unsigned long)frame.id);
        return false;
    }
    return true;
}

bool TwaiDriver::send(uint32_t id, const uint8_t* data, uint8_t length, bool extId, bool rtr) {
    if (length > 8) {
        ESP_LOGE(TAG, "Buffer length exceeds CAN limit (8 bytes)");
        return false;
    }

    TwaiFrame frame{};
    frame.id = id;
    frame.length = length;
    frame.isExtended = extId;
    frame.isRtr = rtr;

    if (data && length > 0) {
        std::memcpy(frame.data.data(), data, length);
    }

    return send(frame);
}

bool TwaiDriver::send(uint32_t id, const std::array<uint8_t, 8>& data, uint8_t length, bool extId, bool rtr) {
    return send(id, data.data(), length, extId, rtr);
}

void TwaiDriver::rxTaskWorker(void* arg) {
    auto* self = static_cast<TwaiDriver*>(arg);
    
    // Static stack buffer for incoming IDF frame data
    uint8_t rxBuffer[8];
    twai_frame_t rxFrame{};
    rxFrame.buffer = rxBuffer;
    rxFrame.buffer_len = sizeof(rxBuffer);

    while (self->m_initialized) {
        // Non-blocking receive with small timeout for task responsiveness
        if (twai_node_receive(self->m_nodeHandle, &rxFrame, 10) == ESP_OK) {
            if (self->m_rxCallback) {
                TwaiFrame frame{};
                frame.id = rxFrame.header.id;
                frame.length = static_cast<uint8_t>(rxFrame.dlc);
                frame.isExtended = rxFrame.header.ide;
                frame.isRtr = rxFrame.header.rtr;

                if (rxFrame.dlc > 0 && rxFrame.dlc <= 8) {
                    std::memcpy(frame.data.data(), rxBuffer, rxFrame.dlc);
                }

                self->m_rxCallback(frame);
            }
        }
    }
    vTaskDelete(NULL);
}

void TwaiDriver::txTaskWorker(void* arg) {
    auto* self = static_cast<TwaiDriver*>(arg);
    TwaiFrame queuedFrame;

    while (self->m_initialized) {
        if (xQueueReceive(self->m_txQueue, &queuedFrame, pdMS_TO_TICKS(10)) == pdPASS) {
            twai_frame_t idfFrame{};
            idfFrame.header.id = queuedFrame.id;
            idfFrame.header.ide = queuedFrame.isExtended ? 1 : 0;
            idfFrame.header.rtr = queuedFrame.isRtr ? 1 : 0;
            idfFrame.dlc = queuedFrame.length;
            idfFrame.buffer = queuedFrame.data.data();
            idfFrame.buffer_len = queuedFrame.length;

            if (twai_node_transmit(self->m_nodeHandle, &idfFrame, 100) != ESP_OK) {
                ESP_LOGE(TAG, "Transmission timeout/error for ID 0x%lX", (unsigned long)queuedFrame.id);
            }
        }
    }
    vTaskDelete(NULL);
}

} // namespace Twai