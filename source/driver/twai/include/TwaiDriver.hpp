#ifndef TWAI_DRIVER_HPP
#define TWAI_DRIVER_HPP

#include <cstdint>
#include <array>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ESP-IDF 6.x TWAI component headers
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"

namespace Twai {

/**
 * @brief Zero-allocation, fixed-size structure representing a CAN frame.
 */
struct TwaiFrame {
    uint32_t id = 0;
    uint8_t length = 0;
    std::array<uint8_t, 8> data{};
    bool isExtended = false;
    bool isRtr = false;
};

using RxCallback = std::function<void(const TwaiFrame&)>;

class TwaiDriver {
public:
    TwaiDriver();
    ~TwaiDriver();

    TwaiDriver(const TwaiDriver&) = delete;
    TwaiDriver& operator=(const TwaiDriver&) = delete;

    /**
     * @brief Initializes the TWAI node and background tasks (ESP-IDF 6.x API)
     * @param txPin GPIO pin for TWAI TX
     * @param rxPin GPIO pin for TWAI RX
     * @param baudRate Bus speed in bps (default: 500000)
     * @param queueSize Size of background TX queue
     * @return true if successfully initialized, false otherwise
     */
    bool begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t baudRate = 500000, uint32_t queueSize = 20);

    /**
     * @brief Stops and deletes the TWAI node, cleaning up tasks and queues
     */
    void end();

    /**
     * @brief Registers callback for incoming frames
     */
    void setOnReceive(RxCallback callback);

    /**
     * @brief Non-blocking transmission of a fixed-size TwaiFrame
     */
    bool send(const TwaiFrame& frame);

    /**
     * @brief Non-blocking transmission using raw pointer (max 8 bytes)
     */
    bool send(uint32_t id, const uint8_t* data, uint8_t length, bool extId = false, bool rtr = false);

    /**
     * @brief Non-blocking transmission using std::array (max 8 bytes)
     */
    bool send(uint32_t id, const std::array<uint8_t, 8>& data, uint8_t length, bool extId = false, bool rtr = false);

private:
    static void rxTaskWorker(void* arg);
    static void txTaskWorker(void* arg);

    bool m_initialized = false;
    twai_node_handle_t m_nodeHandle = nullptr;
    QueueHandle_t m_txQueue = nullptr;
    TaskHandle_t m_rxTaskHandle = nullptr;
    TaskHandle_t m_txTaskHandle = nullptr;

    RxCallback m_rxCallback = nullptr;

    static constexpr const char* TAG = "TWAI_DRIVER";
    static constexpr uint16_t TASK_STACK_SIZE = 4096;
    static constexpr UBaseType_t TASK_PRIORITY = 5;
};

} // namespace Twai

#endif // TWAI_DRIVER_HPP