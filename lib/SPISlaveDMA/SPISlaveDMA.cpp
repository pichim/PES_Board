/**
 * @file    SPISlaveDMA.cpp
 * @brief   Implementation of SpiSlaveDMA (STM32F446 SPI2 + DMA, InterruptIn-driven).
 *
 * DESIGN CHOICE - NSS Edge Detection:
 *  - Uses InterruptIn on PB12 (NSS) to detect rising edge when the master completes a transfer.
 *  - The worker thread blocks on a thread flag; on wake it checks DMA TC flags (RX+TX).
 *  - This avoids periodic polling and reduces CPU overhead.
 */

#include "SPISlaveDMA.h"

// HAL handles for SPI2 + DMA (file-local)
static SPI_HandleTypeDef  hspi2;
static DMA_HandleTypeDef  hdma_spi2_rx;
static DMA_HandleTypeDef  hdma_spi2_tx;

// CRC8 table (poly 0x07, init 0x00)
static const uint8_t CRC8_TAB[256] = {
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

// MSP init: clocks, pins, and DMA streams for SPI2 (HAL_SPI_Init will call this)
extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance != SPI2) return;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE(); // SPI2 uses DMA1 on STM32F446

    // Configure SPI2 pins to AF5
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PB12 (NSS), PB10 (SCK)
    GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;               // master drives NSS
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PC2 (MISO), PC3 (MOSI)
    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // DMA config (common mapping for F446)
    // RX: DMA1 Stream3, Channel 0
    hdma_spi2_rx.Instance                 = DMA1_Stream3;
    hdma_spi2_rx.Init.Channel             = DMA_CHANNEL_0;
    hdma_spi2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi2_rx.Init.Mode                = DMA_NORMAL;             // one-shot per frame
    hdma_spi2_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_spi2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_spi2_rx);
    __HAL_LINKDMA(hspi, hdmarx, hdma_spi2_rx);

    // TX: DMA1 Stream4, Channel 0
    hdma_spi2_tx.Instance                 = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel             = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode                = DMA_NORMAL;
    hdma_spi2_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_spi2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_spi2_tx);
    __HAL_LINKDMA(hspi, hdmatx, hdma_spi2_tx);

    // We do not enable DMA IRQs here; the thread checks TC flags after NSS edge.
}

// SpiSlaveDMA — public API

SpiSlaveDMA::SpiSlaveDMA(PinName led_pin) : m_Thread(osPriorityNormal)
                                          , m_InteruptIn_NSS(PB_12)  // NSS pin for edge detection
                                          , m_Led(led_pin)
{
    // Configure SPI2 peripheral (slave, mode 0, 8-bit, HW NSS)
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_SLAVE;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;       // full-duplex
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;           // CPOL=0
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;            // CPHA=0 => mode 0
    hspi2.Init.NSS               = SPI_NSS_HARD_INPUT;         // hardware NSS on PB12
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE; // using our own CRC8
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;    // ignored in slave

    MBED_ASSERT(HAL_SPI_Init(&hspi2) == HAL_OK);

    // Timing setup
    m_Timer.start();
    m_time_previous = m_Timer.elapsed_time();

    // NOTE: we do not arm or start the thread here.
    // Users must call start() explicitly from main().
}

SpiSlaveDMA::~SpiSlaveDMA()
{
    HAL_SPI_DMAStop(&hspi2);
    m_InteruptIn_NSS.rise(nullptr);  // Detach interrupt
    m_Thread.terminate();
}

bool SpiSlaveDMA::start()
{
    // Build first TX frame and arm DMA once so we’re ready for the first master transfer
    buildTX();

    // Try once (subsequent retries will happen on next NSS edge)
    if (tryArmDmaFrame()) {
        // Start thread and setup NSS rising edge interrupt (master finished)
        m_Thread.start(callback(this, &SpiSlaveDMA::threadTask));
        m_InteruptIn_NSS.rise(callback(this, &SpiSlaveDMA::sendThreadFlag));
        return true;
    }

    printf("[SPI] Initial arm failed (state=%u err=0x%08lX)\n",
           (unsigned)HAL_SPI_GetState(&hspi2), (unsigned long)hspi2.ErrorCode);
    return false;
}

void SpiSlaveDMA::setReplyData(float f0, float f1, float f2)
{
    core_util_critical_section_enter();
    m_reply_data[0] = f0;
    m_reply_data[1] = f1;
    m_reply_data[2] = f2;
    core_util_critical_section_exit();
}

bool SpiSlaveDMA::hasNewData() const
{
    // Lock-free read of a volatile flag; producer/consumer pattern is safe here on Cortex-M
    return m_has_new_data;
}

SPIData SpiSlaveDMA::getSPIData()
{
    // Copy and clear flag together while IRQs are masked to avoid races
    core_util_critical_section_enter();
    SPIData out = m_SPIData;
    m_has_new_data = false;
    core_util_critical_section_exit();

    return out;
}

// Private helpers

void SpiSlaveDMA::threadTask()
{
    while (true) {
        // Wait for NSS rising edge to signal frame end
        ThisThread::flags_wait_any(m_ThreadFlag);

        const microseconds t0 = m_Timer.elapsed_time();

        // Check DMA TC flags, allow a tiny bounded wait to cover EXTI/TC race
        bool rx_done = false, tx_done = false;
        const microseconds deadline = t0 + TC_WAIT_BUDGET;
        do {
            rx_done = (__HAL_DMA_GET_FLAG(&hdma_spi2_rx, DMA_FLAG_TCIF3_7) != 0U);
            tx_done = (__HAL_DMA_GET_FLAG(&hdma_spi2_tx, DMA_FLAG_TCIF0_4) != 0U);
            if (rx_done && tx_done) break;
        } while (m_Timer.elapsed_time() < deadline);

        // If still not both done, treat as timeout/error: re-arm to avoid stall
        if (!(rx_done && tx_done)) {
            core_util_critical_section_enter();
            m_SPIData.failed_count++;
            core_util_critical_section_exit();

            buildTX();
            (void)tryArmDmaFrame();

            const microseconds t1 = m_Timer.elapsed_time();
            m_SPIData.readout_time_us = duration_cast<microseconds>(t1 - t0).count();
            continue;
        }

        // Acknowledge both flags
        __HAL_DMA_CLEAR_FLAG(&hdma_spi2_rx, DMA_FLAG_TCIF3_7);
        __HAL_DMA_CLEAR_FLAG(&hdma_spi2_tx, DMA_FLAG_TCIF0_4);

        // Clear any SPI overrun **before** stopping DMA/SPI (2-line mode).
        // Rule: read DR then SR while SPI is enabled to clear OVR.
        if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_OVR)) {
            volatile uint32_t d;
            d = hspi2.Instance->DR; d = hspi2.Instance->SR; (void)d;
        }

        // Safe to touch buffers now
        HAL_SPI_DMAStop(&hspi2);

        // Process the just-received frame
        const uint8_t hdr    = m_buffer_rx[0];
        const uint8_t crc_rx = m_buffer_rx[SPI_MSG_SIZE - 1];

        if ((hdr == SPI_HEADER_MASTER || hdr == SPI_HEADER_MASTER2) &&
            verifyChecksum(m_buffer_rx, SPI_MSG_SIZE - 1, crc_rx)) {

            // Only publish on PUBLISH header (0x55); 0x56 is arm-only
            if (hdr == SPI_HEADER_MASTER) {
                core_util_critical_section_enter();
                memcpy(m_SPIData.data, &m_buffer_rx[1], SPI_NUM_FLOATS * sizeof(float));
                m_SPIData.message_count++;

                const microseconds now = m_Timer.elapsed_time();
                m_SPIData.last_delta_time_us = duration_cast<microseconds>(now - m_time_previous).count();
                m_time_previous = now;

                m_has_new_data = true;
                core_util_critical_section_exit();

                m_Led = !m_Led;
            }
        } else {
            core_util_critical_section_enter();
            m_SPIData.failed_count++;
            core_util_critical_section_exit();
        }

        // Build next TX from the freshest reply data and re-arm DMA
        buildTX();

        (void)tryArmDmaFrame(); // if it fails, next NSS edge will retry

        const microseconds t1 = m_Timer.elapsed_time();
        m_SPIData.readout_time_us = duration_cast<microseconds>(t1 - t0).count();
    }
}

void SpiSlaveDMA::buildTX()
{
    // Copy reply payload atomically into the TX buffer
    core_util_critical_section_enter();
    m_buffer_tx[0] = SPI_HEADER_SLAVE;
    memcpy(&m_buffer_tx[1], m_reply_data, SPI_NUM_FLOATS * sizeof(float));
    core_util_critical_section_exit();

    // Compute CRC over header + payload
    m_buffer_tx[SPI_MSG_SIZE - 1] = calculateCRC8(m_buffer_tx, SPI_MSG_SIZE - 1);
}

bool SpiSlaveDMA::tryArmDmaFrame()
{
    // Ensure streams disabled (clears any ongoing transfers/state)
    (void)HAL_DMA_Abort(&hdma_spi2_rx);
    (void)HAL_DMA_Abort(&hdma_spi2_tx);

    // Clear all DMA flags (TC/TE/HT/FE) for both streams
    __HAL_DMA_CLEAR_FLAG(&hdma_spi2_rx, DMA_FLAG_TCIF3_7 | DMA_FLAG_TEIF3_7 |
                                       DMA_FLAG_HTIF3_7 | DMA_FLAG_FEIF3_7);
    __HAL_DMA_CLEAR_FLAG(&hdma_spi2_tx, DMA_FLAG_TCIF0_4 | DMA_FLAG_TEIF0_4 |
                                       DMA_FLAG_HTIF0_4 | DMA_FLAG_FEIF0_4);

    // Force HAL state back to READY if needed
    if (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
        (void)HAL_SPI_Abort(&hspi2);
    }

    // Optional: wait briefly for NSS high before arming — avoids corner cases
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t0) > 5) break;
    }

    // Arm the frame
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(&hspi2, m_buffer_tx, m_buffer_rx, SPI_MSG_SIZE);
    if (st != HAL_OK) {
        static uint32_t s_fail_print_throttle = 0;
        if ((s_fail_print_throttle++ % 100U) == 0U) {
            printf("[SPI] Arm failed: st=%ld, state=%u, err=0x%08lX\n",
                (long)st, (unsigned)HAL_SPI_GetState(&hspi2), (unsigned long)hspi2.ErrorCode);
        }
        return false;
    }
    return true;
}

uint8_t SpiSlaveDMA::calculateCRC8(const uint8_t* buffer, size_t length)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc = CRC8_TAB[crc ^ buffer[i]];
    }
    return crc;
}

bool SpiSlaveDMA::verifyChecksum(const uint8_t* buffer, size_t length, uint8_t expected_crc)
{
    return (calculateCRC8(buffer, length) == expected_crc);
}

void SpiSlaveDMA::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}
