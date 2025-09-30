/**
 * @file    SPISlaveDMA.h
 * @brief   STM32F446 (NUCLEO_F446RE) SPIx slave with DMA, event-driven.
 *
 * Features
 *  - DMA-based full-frame transfers (TX+RX) with hardware NSS.
 *  - InterruptIn on NSS rising edge -> worker thread wakes -> checks DMA TC flags.
 *  - No HAL callbacks (avoids Mbed link conflicts), buffers re-armed immediately.
 *  - Double-transfer handshake with master (Pi):
 *      1) 0x56 ARM-ONLY (payload ignore) lets slave finish & re-arm
 *      2) 0x55 PUBLISH (payload valid) publishes data to app
 *  - m_has_new_data is set only when a valid 0x55-PUBLISH frame with
 *    correct CRC was received; 0x56-ARM frames do not raise it.
 *  - Supports SPI1/SPI2/SPI3; pins validated; AF set per instance.
 *  - Robustness: bounded wait for TC, CRC-8, OVR clear, NSS-high guard, auto reset on repeated failures.
 *
 * Notes
 *  - Floats are little-endian (RPi + ARM Cortex-M are LE).
 *  - DataSize = 8-bit, Mode 0 (CPOL=0, CPHA=0).
 *  - Hardware NSS required; one master.
 *  - Works with Mbed OS and Mbed CE.
 */

// Example alternatives (auto-detects SPI instance from the pins you pass)
// -- SPI1 combos --
// SpiSlaveDMA spi(PA_7,  PA_6,  PA_5,  PA_4);    // SPI1, all on Port A
// SpiSlaveDMA spi(PB_5,  PB_4,  PB_3,  PA_15);   // SPI1, PB_3/4/5 + PA_15 NSS
// SpiSlaveDMA spi(PA_7,  PB_4,  PB_3,  PA_4);    // SPI1, mixed ports
// -- SPI2 combos --
// SpiSlaveDMA spi(PC_3,  PC_2,  PB_10, PB_12);   // SPI2, your current wiring
// SpiSlaveDMA spi(PB_15, PB_14, PB_13, PB_9);    // SPI2, all on Port B
// SpiSlaveDMA spi(PC_3,  PB_14, PB_13, PB_12);   // SPI2, mixed ports
// SpiSlaveDMA spi(PB_15, PC_2,  PB_10, PB_9);    // SPI2, mixed ports
// -- SPI3 combos --
// SpiSlaveDMA spi(PC_12, PC_11, PC_10, PA_4);    // SPI3, unique SCK=PC_10
// SpiSlaveDMA spi(PC_12, PB_4,  PC_10, PA_15);   // SPI3, SCK=PC_10 forces SPI3
// SpiSlaveDMA spi(PB_5,  PC_11, PB_3,  PA_4);    // SPI3, PB_3 SCK + PC_11 MISO disambiguates to SPI3

#ifndef SPI_SLAVE_DMA_H_
#define SPI_SLAVE_DMA_H_

#include "mbed.h"
#include "ThreadFlag.h"

extern "C" {
#include "stm32f4xx_hal.h"
}

using namespace std::chrono;

// ----------------------------- Protocol constants -----------------------------
#define SPI_HEADER_MASTER   0x55 // publish (Pi->STM)
#define SPI_HEADER_MASTER2  0x56 // arm-only (Pi->STM), do NOT publish
#define SPI_HEADER_SLAVE    0x45 // data-from-STM (STM->Pi)
#define SPI_NUM_FLOATS      120
#define SPI_MSG_SIZE        (1 + SPI_NUM_FLOATS * 4 + 1) // header + floats + crc

// ----------------------------- SpiData (app-facing) ---------------------------
class SpiData {
public:
    SpiData() {
        init();
    };
    ~SpiData() = default;

    float    data[SPI_NUM_FLOATS];
    uint32_t message_count;
    uint32_t failed_count;
    uint32_t last_delta_time_us;
    // Time (µs) spent in the worker to service the most-recent frame:
    // NSS↑ wake-up -> CRC check / copy -> TX-buffer rebuild → next DMA arm.
    uint32_t readout_time_us;

    void init() {
        for (int i = 0; i < SPI_NUM_FLOATS; i++)
            data[i] = 0.0f;
        message_count = 0;
        failed_count = 0;
        last_delta_time_us = 0;
        readout_time_us = 0;
    };
};

// ----------------------------- SpiSlaveDMA ------------------------------------
class SpiSlaveDMA {
public:
    // Public API: auto-detect SPI instance from the provided pins
    explicit SpiSlaveDMA(PinName mosi, PinName miso, PinName sck, PinName nss);
    ~SpiSlaveDMA();

    // Arm first DMA transfer and start the worker thread (true on success)
    bool start();

    // Update 3 floats (thread-safe; copied atomically into next TX frame)
    void setReplyData(float f0, float f1, float f2);

    // Set a single element (returns false if index out of range)
    bool setReplyData(size_t index, float value);

    // Copy a block into reply buffer; returns number of floats copied
    size_t setReplyData(const float* src, size_t count, size_t dest_offset = 0);

    // Has a fresh frame been received since the last get? (lock-free read)
    bool hasNewData() const { return m_has_new_data; }

    // Retrieve and clear the “new data” flag
    SpiData getSPIData();

private:
    // Internal instance tag (inferred from pins)
    enum class Instance { SPI_1, SPI_2, SPI_3 };
    static Instance inferInstance(PinName mosi, PinName miso, PinName sck, PinName nss);

    // ======================= Tunables =======================
    // Grace window after NSS rising to cover EXTI → DMA TC race
    inline static constexpr microseconds TC_WAIT_BUDGET{200};
    // Max consecutive arm failures before a peripheral reset
    static constexpr uint32_t MAX_CONSECUTIVE_FAILURES = 20;

    // ====================== RTOS/Mbed =======================
    Thread       m_Thread;            // worker thread
    InterruptIn  m_InterruptIn_NSS;   // end-of-frame edge
    ThreadFlag   m_ThreadFlag;

    // ====================== Selected pins ===================
    PinName      m_MOSI;
    PinName      m_MISO;
    PinName      m_SCK;
    PinName      m_NSS;

    // ====================== SPI selection ===================
    Instance     m_instance;

    // ====================== Timing ==========================
    Timer        m_Timer;             // frame delta timing
    microseconds m_time_previous{0};

    // ====================== Reply & State ===================
    float   m_reply_data[SPI_NUM_FLOATS] = {11.11f, 22.22f, 33.33f, 44.44f, 55.55f};
    SpiData m_SPIData;
    volatile bool m_has_new_data{false};
    uint32_t m_consecutive_failures{0};

    // ====================== DMA Buffers =====================
    // in class members
    alignas(4) uint8_t m_buffer_rx[SPI_MSG_SIZE];
    alignas(4) uint8_t m_buffer_tx[SPI_MSG_SIZE];

    // ====================== HAL Handles =====================
    SPI_HandleTypeDef  m_hspi;   // SPI1/2/3 selected by m_instance
    DMA_HandleTypeDef  m_dma_rx;
    DMA_HandleTypeDef  m_dma_tx;

    // ====================== DMA Flag Masks ==================
    uint32_t m_rx_tc_flag{0};
    uint32_t m_tx_tc_flag{0};
    uint32_t m_rx_all_flags{0};
    uint32_t m_tx_all_flags{0};

private:
    // ---------- Worker flow ----------
    void threadTask();
    void buildTX();
    bool tryArmDmaFrame();

    // ---------- CRC helpers ----------
    static uint8_t calculateCRC8(const uint8_t* buf, size_t len);
    static bool    verifyChecksum(const uint8_t* buf, size_t len, uint8_t expected_crc);

    // ---------- ISR hook ------------
    void sendThreadFlag();

    // ---------- HW config -----------
    void configureGPIOandDMA();
    bool validatePins() const;
    void resetSPIPeripheral();
    const char* instanceName() const;

    // ---------- Flag helpers (per DMA stream group) ----------
    static uint32_t tc_flag_for(DMA_Stream_TypeDef* s);
    static uint32_t ht_flag_for(DMA_Stream_TypeDef* s);
    static uint32_t te_flag_for(DMA_Stream_TypeDef* s);
    static uint32_t fe_flag_for(DMA_Stream_TypeDef* s);
};

#endif // SPI_SLAVE_DMA_H_
