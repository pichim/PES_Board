/**
 * @file    SPISlaveDMA.h
 * @brief   SPI2 slave with DMA (STM32F446, NUCLEO_F446RE), non-blocking:
 *          - Uses DMA for full-frame transfers (TX+RX) with hardware NSS.
 *          - No busy-waiting; a thread sleeps briefly and polls DMA TC flags.
 *          - No HAL callback overrides (avoids Mbed link conflicts).
 *          - Re-arms DMA immediately for the next master frame.
 */

// TODO:
// - Check sampling time (PERIOD_MUS = 50?)
// - Check thread priority (osPriorityBelowNormal?)

#ifndef SPI_SLAVE_DMA_H_
#define SPI_SLAVE_DMA_H_

#include "mbed.h"
#include "ThreadFlag.h"
// #include <cstdint>

extern "C" {
#include "stm32f4xx_hal.h"
}

using namespace std::chrono;

// -----------------------------
// Protocol constants
// -----------------------------
#define SPI_HEADER_MASTER 0x55
#define SPI_HEADER_SLAVE  0x45
#define SPI_NUM_FLOATS    3
#define SPI_MSG_SIZE      (1 + SPI_NUM_FLOATS * 4 + 1) // header + floats(3*4) + crc

// SPIData structure with processing time (names kept as in your new code)
class SPIData
{
public:
    SPIData() { init(); };
    ~SPIData() = default;

    float data[SPI_NUM_FLOATS];
    uint32_t message_count;
    uint32_t failed_count;
    uint32_t last_delta_time_us;
    uint32_t readout_time_us;

    void init()
    {
        for (int i = 0; i < SPI_NUM_FLOATS; i++) data[i] = 0.0f;
        message_count = 0;
        failed_count = 0;
        last_delta_time_us = 0;
        readout_time_us = 0;
    };
};

/**
 * @class   SpiSlaveDMA
 * @brief   Encapsulates SPI2 + DMA in slave mode with non-blocking operation.
 *
 * Design (no EXTI on NSS to avoid AF conflict):
 *  - A dedicated thread loops, sleeping lightly (~100–200 µs), and checks the
 *    DMA TC flags for RX (DMA1 Stream3) and TX (DMA1 Stream4).
 *  - When BOTH are set, the frame is complete:
 *       (1) Clear flags, HAL_SPI_DMAStop() so buffers are safe to touch
 *       (2) Verify CRC, parse floats, update counters/timing
 *       (3) Build next TX frame and re-arm immediately
 *  - No Ticker, no HAL IRQ handlers, no InterruptIn on PB12 (keeps HW NSS intact).
 */
class SpiSlaveDMA {
public:
    // Construct with default SPI2 pinout for NUCLEO_F446RE
    SpiSlaveDMA(PinName led_pin = LED1);

    ~SpiSlaveDMA();

    // Start the worker thread + arm the very first DMA transfer (returns success)
    bool start();

    // Update the outgoing floats (thread-safe; copied atomically into next TX frame)
    void set_reply_data(float f0, float f1, float f2);

    // Has a fresh frame been received since the last get?
    bool has_new_data() const;

    // Retrieve and clear the “new data” flag
    void get_last(SPIData& out);

private:
    static constexpr int64_t PERIOD_MUS = 50;
    // --- mbed parts ---
    Thread       m_Thread;            // worker thread
    Ticker       m_Ticker;
    ThreadFlag   m_ThreadFlag;
    DigitalOut   m_led;               // optional activity LED

    // --- timing ---
    Timer        m_timer;             // for delta between valid frames
    microseconds m_prev_ts{0};

    // --- reply data (updated by user; copied under critical-section) ---
    float m_reply_data[SPI_NUM_FLOATS] = {11.11f, 22.22f, 33.33f};

    // --- last received data + counters ---
    SPIData      m_last;

    // --- DMA-owned buffers (must be static-duration memory; not on stack) ---
    uint8_t m_rx[SPI_MSG_SIZE];
    uint8_t m_tx[SPI_MSG_SIZE];

    // --- new data flag (set when we parsed a valid frame) ---
    volatile bool m_new_data{false};

private:
    // Main worker loop (polls DMA TC flags with a short sleep)
    void threadTask();

    // Build the TX buffer from m_reply_data (CRC included)
    void build_tx();

    // Arm SPI2 DMA safely (clear flags/OVR, ensure streams disabled). Returns success.
    bool try_arm_dma_frame_();

    // CRC helpers
    static uint8_t calculate_crc8(const uint8_t* buf, size_t len);
    static bool    verify_checksum(const uint8_t* buf, size_t len, uint8_t expected_crc);

    void sendThreadFlag();
};

#endif // SPI_SLAVE_DMA_H_
