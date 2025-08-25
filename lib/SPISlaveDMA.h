/**
 * @file    SPISlaveDMA.h
 * @brief   SPI2 slave with DMA (STM32F446, NUCLEO_F446RE), event-driven:
 *          - Uses DMA for full-frame transfers (TX+RX) with hardware NSS.
 *          - A thread sleeps, woken by NSS rising edge (InterruptIn) to check DMA TC flags.
 *          - No HAL DMA/SPI callbacks (avoids Mbed link conflicts).
 *          - Re-arms DMA immediately for the next master frame.
 */

#ifndef SPI_SLAVE_DMA_H_
#define SPI_SLAVE_DMA_H_

#include "mbed.h"
#include "ThreadFlag.h"

extern "C" {
#include "stm32f4xx_hal.h"
}

using namespace std::chrono;

// -----------------------------
// Protocol constants
// -----------------------------
#define SPI_HEADER_MASTER  0x55 // publish (Pi->STM)
#define SPI_HEADER_MASTER2 0x56 // arm-only (Pi->STM), do NOT publish
#define SPI_HEADER_SLAVE   0x45 // data-from-STM (STM->Pi)
#define SPI_NUM_FLOATS     200
#define SPI_MSG_SIZE       (1 + SPI_NUM_FLOATS * 4 + 1) // header + floats + crc

// SPIData structure with processing time
class SPIData
{
public:
    SPIData() {
        init();
    };
    ~SPIData() = default;

    float    data[SPI_NUM_FLOATS];
    uint32_t message_count;
    uint32_t failed_count;
    uint32_t last_delta_time_us;
    uint32_t readout_time_us;

    void init()
    {
        for (int i = 0; i < SPI_NUM_FLOATS; i++)
            data[i] = 0.0f;
        message_count = 0;
        failed_count = 0;
        last_delta_time_us = 0;
        readout_time_us = 0;
    };
};

/**
 * @class   SpiSlaveDMA
 * @brief   Encapsulates SPI2 + DMA in slave mode with InterruptIn-driven operation.
 *
 * Design (NSS edge detection):
 *  - NSS (PB_12) rising edge interrupt signals end of a master frame.
 *  - The worker thread wakes on a thread flag, then checks DMA RX/TX TC flags.
 *  - When BOTH are set, the frame is complete:
 *       (1) Clear flags, HAL_SPI_DMAStop() so buffers are safe to touch
 *       (2) Verify CRC, parse floats, update counters/timing
 *       (3) Build next TX frame and re-arm immediately
 */
class SpiSlaveDMA {
public:
    // Construct with default SPI2 pinout for NUCLEO_F446RE
    explicit SpiSlaveDMA(PinName led_pin = LED1);

    ~SpiSlaveDMA();

    // Arm the first DMA transfer and start the worker thread (returns success)
    bool start();

    // Update the outgoing floats (thread-safe; copied atomically into next TX frame)
    void setReplyData(float f0, float f1, float f2);

    // Has a fresh frame been received since the last get? (lock-free read)
    bool hasNewData() const;

    // Retrieve and clear the “new data” flag
    SPIData getSPIData();

private:
    // Grace window after NSS rising (100 µs) to cover the EXTI → DMA TC race.
    // The worker waits up to this budget for both RX/TX TC flags; on timeout it
    // immediately re-arms so transfers never stall. Marked `inline static constexpr`
    // so this non-integral constant can live in the header without a separate
    // definition (header-safe), and `{100}` avoids needing chrono literals.
    // Tune if your master's SCK or inter-frame timing changes.
    inline static constexpr microseconds TC_WAIT_BUDGET{100};

    // --- mbed parts ---
    Thread       m_Thread;            // worker thread
    InterruptIn  m_InteruptIn_NSS;     // NSS rising edge detection (PB_12)
    ThreadFlag   m_ThreadFlag;
    DigitalOut   m_Led;               // optional activity LED

    // --- timing ---
    Timer        m_Timer;             // for delta between valid frames
    microseconds m_time_previous{0};

    // --- reply data (updated by user; copied under critical-section) ---
    float m_reply_data[SPI_NUM_FLOATS] = {11.11f, 22.22f, 33.33f, 44.44f, 55.55f};

    // --- last received data + counters ---
    SPIData      m_SPIData;

    // --- DMA-owned buffers (must be static-duration memory; not on stack) ---
    uint8_t m_buffer_rx[SPI_MSG_SIZE];
    uint8_t m_buffer_tx[SPI_MSG_SIZE];

    // --- new data flag (set when we parsed a valid frame) ---
    volatile bool m_has_new_data{false};

private:
    // Main worker loop (wakes on NSS edge; checks DMA TC flags)
    void threadTask();

    // Build the TX buffer from m_reply_data (CRC included)
    void buildTX();

    // Arm SPI2 DMA safely (clear flags/OVR, ensure streams disabled). Returns success.
    bool tryArmDmaFrame();

    // CRC helpers
    static uint8_t calculateCRC8(const uint8_t* buf, size_t len);
    static bool    verifyChecksum(const uint8_t* buf, size_t len, uint8_t expected_crc);

    void sendThreadFlag();
};

#endif // SPI_SLAVE_DMA_H_
