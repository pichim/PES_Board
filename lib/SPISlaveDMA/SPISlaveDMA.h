/**
 * @file    SPISlaveDMA.h
 * @brief   SPI2 slave with DMA (STM32F446, NUCLEO_F446RE), event-driven:
 *          - Uses DMA for full-frame transfers (TX+RX) with hardware NSS.
 *          - A thread sleeps, woken by NSS rising edge (InterruptIn) to check DMA TC flags.
 *          - No HAL DMA/SPI callbacks (avoids Mbed link conflicts).
 *          - Re-arms DMA immediately for the next master frame.
 *
 *          Pin selection (SPI2, AF5) is configurable via the constructor:
 *            SCK  : PB_10 or PB_13
 *            MISO : PB_14 or PC_2
 *            MOSI : PB_15 or PC_3
 *            NSS  : PB_9  or PB_12
 *          Pins are validated at start() with MBED_ASSERT.
 *
 *          Multiple instances are supported: all HAL handles (SPI+DMA) are
 *          kept per-instance, and HAL_SPI_MspInit is intentionally left empty.
 *          Works on both Mbed OS and Mbed CE.
 *
 *          Handshake used by the Raspberry Pi master (double-transfer):
 *            1) 0x56 "ARM-ONLY" frame (payload don't-care) — lets the slave finish DMA and re-arm
 *            2) 0x55 "PUBLISH" frame (payload valid)        — slave publishes this one
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
#define SPI_NUM_FLOATS     120
#define SPI_MSG_SIZE       (1 + SPI_NUM_FLOATS * 4 + 1) // header + floats + crc

// SpiData structure with processing time
class SpiData
{
public:
    SpiData() {
        init();
    };
    ~SpiData() = default;

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
 *  - The selected NSS pin rising edge interrupt signals end of a master frame.
 *  - The worker thread wakes on a thread flag, then checks DMA RX/TX TC flags.
 *  - When BOTH are set, the frame is complete:
 *       (1) Clear flags, HAL_SPI_DMAStop() so buffers are safe to touch
 *       (2) Verify CRC, parse floats, update counters/timing
 *       (3) Build next TX frame and re-arm immediately
 *
 * Double-transfer handshake support:
 *  - Accepts 0x55 (publish) and 0x56 (arm-only) headers from the Pi.
 *  - Publishes data to the app only on 0x55; 0x56 is used to guarantee minimal latency on 0x55.
 *  - No LED toggling is performed (kept minimal for deterministic timing).
 */
class SpiSlaveDMA {
public:
    // Construct with explicit SPI2 pins (MOSI, MISO, SCK, NSS)
    explicit SpiSlaveDMA(PinName mosi, PinName miso, PinName sck, PinName nss);

    ~SpiSlaveDMA();

    // Arm the first DMA transfer and start the worker thread (returns success)
    bool start();

    // Update the outgoing floats (thread-safe; copied atomically into next TX frame)
    void setReplyData(float f0, float f1, float f2);

    // Has a fresh frame been received since the last get? (lock-free read)
    bool hasNewData() const;

    // Retrieve and clear the “new data” flag
    SpiData getSPIData();

private:
    // Grace window after NSS rising (200 µs by default) to cover the EXTI → DMA TC race.
    // Tune if your master's SCK or inter-frame timing changes (e.g., faster cadence).
    inline static constexpr microseconds TC_WAIT_BUDGET{200};

    // --- mbed parts ---
    Thread       m_Thread;            // worker thread
    InterruptIn  m_InterruptIn_NSS;   // NSS rising edge detection (selected NSS pin)
    ThreadFlag   m_ThreadFlag;

    // --- selected pins (SPI2) ---
    PinName      m_MOSI;
    PinName      m_MISO;
    PinName      m_SCK;
    PinName      m_NSS;

    // --- timing ---
    Timer        m_Timer;             // for delta between valid frames
    microseconds m_time_previous{0};

    // --- reply data (updated by user; copied under critical-section) ---
    float m_reply_data[SPI_NUM_FLOATS] = {11.11f, 22.22f, 33.33f, 44.44f, 55.55f};

    // --- last received data + counters ---
    SpiData      m_SPIData;

    // --- DMA-owned buffers (must be static-duration memory; not on stack) ---
    uint8_t m_buffer_rx[SPI_MSG_SIZE];
    uint8_t m_buffer_tx[SPI_MSG_SIZE];

    // --- HAL handles (per-instance; enables multiple objects) ---
    SPI_HandleTypeDef  m_hspi2;
    DMA_HandleTypeDef  m_dma_rx;
    DMA_HandleTypeDef  m_dma_tx;

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

    // Configure GPIO clocks, pins (AF5), SPI2 clock and DMA streams for SPI2
    void configureGPIOandDMA();

    // Validate the selected pins belong to the allowed SPI2 sets and are unique
    bool validatePins() const;
};

#endif // SPI_SLAVE_DMA_H_
