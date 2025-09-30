#include "SPISlaveDMA.h"

// Keep HAL SPI MSP init empty; class configures pins/clocks/DMA itself.
extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) { (void)hspi; }

// ============================ CRC-8 (poly 0x07) ==============================
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

// ============================ Helpers: DMA flag masks =========================
static inline bool is_stream_0_4(DMA_Stream_TypeDef* s) {
    return (s == DMA1_Stream0 || s == DMA2_Stream0 || s == DMA1_Stream4 || s == DMA2_Stream4);
}
static inline bool is_stream_1_5(DMA_Stream_TypeDef* s) {
    return (s == DMA1_Stream1 || s == DMA2_Stream1 || s == DMA1_Stream5 || s == DMA2_Stream5);
}
static inline bool is_stream_2_6(DMA_Stream_TypeDef* s) {
    return (s == DMA1_Stream2 || s == DMA2_Stream2 || s == DMA1_Stream6 || s == DMA2_Stream6);
}
static inline bool is_stream_3_7(DMA_Stream_TypeDef* s) {
    return (s == DMA1_Stream3 || s == DMA2_Stream3 || s == DMA1_Stream7 || s == DMA2_Stream7);
}

uint32_t SpiSlaveDMA::tc_flag_for(DMA_Stream_TypeDef* s) {
    if (is_stream_0_4(s)) return DMA_FLAG_TCIF0_4;
    if (is_stream_1_5(s)) return DMA_FLAG_TCIF1_5;
    if (is_stream_2_6(s)) return DMA_FLAG_TCIF2_6;
    return DMA_FLAG_TCIF3_7;
}
uint32_t SpiSlaveDMA::ht_flag_for(DMA_Stream_TypeDef* s) {
    if (is_stream_0_4(s)) return DMA_FLAG_HTIF0_4;
    if (is_stream_1_5(s)) return DMA_FLAG_HTIF1_5;
    if (is_stream_2_6(s)) return DMA_FLAG_HTIF2_6;
    return DMA_FLAG_HTIF3_7;
}
uint32_t SpiSlaveDMA::te_flag_for(DMA_Stream_TypeDef* s) {
    if (is_stream_0_4(s)) return DMA_FLAG_TEIF0_4;
    if (is_stream_1_5(s)) return DMA_FLAG_TEIF1_5;
    if (is_stream_2_6(s)) return DMA_FLAG_TEIF2_6;
    return DMA_FLAG_TEIF3_7;
}
uint32_t SpiSlaveDMA::fe_flag_for(DMA_Stream_TypeDef* s) {
    if (is_stream_0_4(s)) return DMA_FLAG_FEIF0_4;
    if (is_stream_1_5(s)) return DMA_FLAG_FEIF1_5;
    if (is_stream_2_6(s)) return DMA_FLAG_FEIF2_6;
    return DMA_FLAG_FEIF3_7;
}

// ============================ Public API =====================================

SpiSlaveDMA::SpiSlaveDMA(PinName mosi,
                         PinName miso,
                         PinName sck,
                         PinName nss)
    : m_Thread(osPriorityHigh2)      // raise to Realtime if you want tighter latency
    , m_InterruptIn_NSS(nss)
    , m_MOSI(mosi)
    , m_MISO(miso)
    , m_SCK(sck)
    , m_NSS(nss)
    , m_instance(inferInstance(mosi, miso, sck, nss))
{
    m_Timer.start();
    m_time_previous = m_Timer.elapsed_time();

    std::memset(&m_hspi, 0, sizeof(m_hspi));
    // Instance set in configureGPIOandDMA() just before HAL_SPI_Init
    m_hspi.Init.Mode              = SPI_MODE_SLAVE;
    m_hspi.Init.Direction         = SPI_DIRECTION_2LINES;       // full-duplex
    m_hspi.Init.DataSize          = SPI_DATASIZE_8BIT;
    m_hspi.Init.CLKPolarity       = SPI_POLARITY_LOW;           // CPOL=0
    m_hspi.Init.CLKPhase          = SPI_PHASE_1EDGE;            // CPHA=0 => mode 0
    m_hspi.Init.NSS               = SPI_NSS_HARD_INPUT;         // hardware NSS
    m_hspi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    m_hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
    m_hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE; // use our CRC-8
    m_hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;    // ignored in slave

    std::memset(&m_dma_rx, 0, sizeof(m_dma_rx));
    std::memset(&m_dma_tx, 0, sizeof(m_dma_tx));
}

// ----------- Instance inference (NUCLEO-F446RE pin map) -----------
SpiSlaveDMA::Instance SpiSlaveDMA::inferInstance(PinName mosi, PinName miso, PinName sck, PinName nss) {
    auto in = [](PinName p, std::initializer_list<PinName> set)
    {
        for (auto q : set)
            if (p == q)
                return true;
        return false;
    };

    // 1) Fast path: SCK is unique?
    if (sck == PA_5)                     return Instance::SPI_1;   // SPI1 only
    if (sck == PB_10 || sck == PB_13)    return Instance::SPI_2;   // SPI2 only
    if (sck == PC_10)                    return Instance::SPI_3;   // SPI3 only

    // 2) Ambiguous SCK = PB_3 (SPI1 or SPI3). Disambiguate by unique companions.
    if (sck == PB_3) {
        // Any uniquely-SPI3 companion? -> SPI3
        if (in(miso, {PC_11}) || in(mosi, {PC_12})) return Instance::SPI_3;
        // Any uniquely-SPI1 companion? -> SPI1
        if (in(miso, {PA_6}) || in(mosi, {PA_7}))   return Instance::SPI_1;
        // Inconsistent case guard: PB_3 SCK cannot go with SPI2-only NSS pins
        if (nss == PB_12 || nss == PB_9) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),
                       "Inconsistent pins: SCK=PB_3 cannot be SPI2");
        }
        // Tie-breaker: default to SPI1 for PB_3/PB_4/PB_5 with PA_4/PA_15.
        return Instance::SPI_1;
    }

    // 3) No SCK match: pick by other unique signals (rare edge-cases).
    if (in(mosi, {PC_3}) || in(miso, {PC_2}) || nss == PB_12 || nss == PB_9) return Instance::SPI_2;
    if (in(mosi, {PC_12}) || in(miso, {PC_11}))                              return Instance::SPI_3;
    if (in(mosi, {PA_7})  || in(miso, {PA_6}))                               return Instance::SPI_1;

    MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),
               "Cannot infer SPI instance from the provided pins");
    // Unreachable, but keeps some compilers happy.
    return Instance::SPI_2;
}

SpiSlaveDMA::~SpiSlaveDMA() {
    HAL_SPI_DMAStop(&m_hspi);
    m_InterruptIn_NSS.rise(nullptr);
    m_Thread.terminate();
}

bool SpiSlaveDMA::start() {
    MBED_ASSERT(validatePins());

    // attach wakeup FIRST (configures EXTI), before AF is applied
    m_InterruptIn_NSS.rise(callback(this, &SpiSlaveDMA::sendThreadFlag));

    // Ensure EXTI IRQ for our NSS line runs at a sensible priority.
    // Note: smaller number = higher priority (Cortex-M). 5 is a safe default under Mbed RTOS.
    IRQn_Type nss_irq = EXTI15_10_IRQn;
    if      (m_NSS == PB_12 || m_NSS == PA_15) { nss_irq = EXTI15_10_IRQn; }
    else if (m_NSS == PB_9)                    { nss_irq = EXTI9_5_IRQn;   }
    else if (m_NSS == PA_4)                    { nss_irq = EXTI4_IRQn;     }
    NVIC_SetPriority(nss_irq, 5);
    // (Do not call NVIC_EnableIRQ here; InterruptIn already enables the line.)

    configureGPIOandDMA();                        // sets AF on pins
    MBED_ASSERT(HAL_SPI_Init(&m_hspi) == HAL_OK);

    buildTX();
    if (tryArmDmaFrame()) {
        m_Thread.start(callback(this, &SpiSlaveDMA::threadTask));
        return true;
    }

    printf("[%-4s] Initial arm failed ...\n", instanceName());
    return false;
}

void SpiSlaveDMA::setReplyData(float f0, float f1, float f2) {
    const float tmp[3] = {f0, f1, f2};
    (void)setReplyData(tmp, 3);
}

bool SpiSlaveDMA::setReplyData(size_t index, float value) {
    if (index >= SPI_NUM_FLOATS) return false;
    core_util_critical_section_enter();
    m_reply_data[index] = value;
    core_util_critical_section_exit();
    return true;
}

size_t SpiSlaveDMA::setReplyData(const float* src, size_t count, size_t dest_offset) {
    if (!src || dest_offset >= SPI_NUM_FLOATS || count == 0) return 0;
    const size_t max_copy = SPI_NUM_FLOATS - dest_offset;
    const size_t n = (count < max_copy) ? count : max_copy;

    core_util_critical_section_enter();
    std::memcpy(&m_reply_data[dest_offset], src, n * sizeof(float));
    core_util_critical_section_exit();

    return n;
}

SpiData SpiSlaveDMA::getSPIData() {
    core_util_critical_section_enter();
    SpiData out = m_SPIData;
    m_has_new_data = false;
    core_util_critical_section_exit();
    return out;
}

// ============================ Private: worker =================================

void SpiSlaveDMA::threadTask() {
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        const microseconds t0 = m_Timer.elapsed_time();

        bool rx_done = false, tx_done = false;
        const microseconds deadline = t0 + TC_WAIT_BUDGET;
        do {
            rx_done = (__HAL_DMA_GET_FLAG(&m_dma_rx, m_rx_tc_flag) != 0U);
            tx_done = (__HAL_DMA_GET_FLAG(&m_dma_tx, m_tx_tc_flag) != 0U);
            if (rx_done && tx_done) break;
        } while (m_Timer.elapsed_time() < deadline);

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

        __HAL_DMA_CLEAR_FLAG(&m_dma_rx, m_rx_tc_flag);
        __HAL_DMA_CLEAR_FLAG(&m_dma_tx, m_tx_tc_flag);

        // Clear possible OVR (read DR then SR while SPI enabled)
        if (__HAL_SPI_GET_FLAG(&m_hspi, SPI_FLAG_OVR)) {
            volatile uint32_t d;
            d = m_hspi.Instance->DR; d = m_hspi.Instance->SR; (void)d;
        }

        HAL_SPI_DMAStop(&m_hspi);

        const uint8_t hdr    = m_buffer_rx[0];
        const uint8_t crc_rx = m_buffer_rx[SPI_MSG_SIZE - 1];

        if ((hdr == SPI_HEADER_MASTER || hdr == SPI_HEADER_MASTER2) &&
            verifyChecksum(m_buffer_rx, SPI_MSG_SIZE - 1, crc_rx)) {

            if (hdr == SPI_HEADER_MASTER) {
                core_util_critical_section_enter();
                std::memcpy(m_SPIData.data, &m_buffer_rx[1], SPI_NUM_FLOATS * sizeof(float));
                m_SPIData.message_count++;

                const microseconds now = m_Timer.elapsed_time();
                m_SPIData.last_delta_time_us = duration_cast<microseconds>(now - m_time_previous).count();
                m_time_previous = now;

                m_has_new_data = true;
                core_util_critical_section_exit();
            }
        } else {
            core_util_critical_section_enter();
            m_SPIData.failed_count++;
            core_util_critical_section_exit();
        }

        buildTX();
        (void)tryArmDmaFrame();

        const microseconds t1 = m_Timer.elapsed_time();
        m_SPIData.readout_time_us = duration_cast<microseconds>(t1 - t0).count();
    }
}

void SpiSlaveDMA::buildTX() {
    core_util_critical_section_enter();
    m_buffer_tx[0] = SPI_HEADER_SLAVE;
    std::memcpy(&m_buffer_tx[1], m_reply_data, SPI_NUM_FLOATS * sizeof(float));
    core_util_critical_section_exit();

    m_buffer_tx[SPI_MSG_SIZE - 1] = calculateCRC8(m_buffer_tx, SPI_MSG_SIZE - 1);
}

bool SpiSlaveDMA::tryArmDmaFrame() {
    (void)HAL_DMA_Abort(&m_dma_rx);
    (void)HAL_DMA_Abort(&m_dma_tx);

    __HAL_DMA_CLEAR_FLAG(&m_dma_rx, m_rx_all_flags);
    __HAL_DMA_CLEAR_FLAG(&m_dma_tx, m_tx_all_flags);

    if (HAL_SPI_GetState(&m_hspi) != HAL_SPI_STATE_READY) {
        (void)HAL_SPI_Abort(&m_hspi);
    }

    // Guard: only arm when NSS is high (avoid hang if master holds it low)
    GPIO_TypeDef* nss_port = nullptr;
    uint16_t nss_pinmask = 0;
    if      (m_NSS == PB_12) { nss_port = GPIOB; nss_pinmask = GPIO_PIN_12; }
    else if (m_NSS == PB_9 ) { nss_port = GPIOB; nss_pinmask = GPIO_PIN_9;  }
    else if (m_NSS == PA_4 ) { nss_port = GPIOA; nss_pinmask = GPIO_PIN_4;  }
    else if (m_NSS == PA_15) { nss_port = GPIOA; nss_pinmask = GPIO_PIN_15; }

    if (nss_port != nullptr) {
        static constexpr uint32_t NSS_TIMEOUT_MS = 5;
        const uint32_t t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(nss_port, nss_pinmask) == GPIO_PIN_RESET) {
            if ((HAL_GetTick() - t0) > NSS_TIMEOUT_MS) {
                m_consecutive_failures++;
                if (m_consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                    resetSPIPeripheral();
                }
                return false;
            }
        }
    }

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(&m_hspi, m_buffer_tx, m_buffer_rx, SPI_MSG_SIZE);
    if (st != HAL_OK) {
        m_consecutive_failures++;

        static uint32_t throttle = 0;
        if ((throttle++ % 100U) == 0U) {
            printf("[%s] Arm failed: st=%ld, state=%u, err=0x%08lX (fails=%lu)\n",
                   instanceName(), (long)st, (unsigned)HAL_SPI_GetState(&m_hspi),
                   (unsigned long)m_hspi.ErrorCode, (unsigned long)m_consecutive_failures);
        }

        if (m_consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
            resetSPIPeripheral();
        }
        return false;
    }

    m_consecutive_failures = 0;
    return true;
}

// ============================ CRC helpers =====================================

uint8_t SpiSlaveDMA::calculateCRC8(const uint8_t* buffer, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc = CRC8_TAB[crc ^ buffer[i]];
    }
    return crc;
}

bool SpiSlaveDMA::verifyChecksum(const uint8_t* buffer, size_t length, uint8_t expected_crc) {
    return (calculateCRC8(buffer, length) == expected_crc);
}

// ============================ ISR hook ========================================

void SpiSlaveDMA::sendThreadFlag() {
    m_Thread.flags_set(m_ThreadFlag);
}

// ============================ HW config / pins / DMA ==========================

static void enable_gpio_clk_for(PinName pin) {
    if (pin == PA_4 || pin == PA_5 || pin == PA_6 || pin == PA_7 || pin == PA_15) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (pin == PB_3 || pin == PB_4 || pin == PB_5 || pin == PB_9 || pin == PB_10 || pin == PB_12 || pin == PB_13 || pin == PB_14 || pin == PB_15) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (pin == PC_2 || pin == PC_3 || pin == PC_10 || pin == PC_11 || pin == PC_12) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
}

void SpiSlaveDMA::configureGPIOandDMA() {
    // Clocks: GPIOs + SPIx + DMAx
    enable_gpio_clk_for(m_MOSI);
    enable_gpio_clk_for(m_MISO);
    enable_gpio_clk_for(m_SCK);
    enable_gpio_clk_for(m_NSS);

    if (m_instance == Instance::SPI_1) { __HAL_RCC_SPI1_CLK_ENABLE(); __HAL_RCC_DMA2_CLK_ENABLE(); }
    else                                { __HAL_RCC_DMA1_CLK_ENABLE(); } // SPI2/SPI3 use DMA1
    if (m_instance == Instance::SPI_2) { __HAL_RCC_SPI2_CLK_ENABLE(); }
    if (m_instance == Instance::SPI_3) { __HAL_RCC_SPI3_CLK_ENABLE(); }

    // AF: SPI1/SPI2 = AF5, SPI3 = AF6 (use instance-specific macro names)
    const uint32_t af =
        (m_instance == Instance::SPI_3) ? GPIO_AF6_SPI3 :
        (m_instance == Instance::SPI_2) ? GPIO_AF5_SPI2 : GPIO_AF5_SPI1;

    auto do_pin = [&](PinName pin) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_TypeDef* port = nullptr;
        uint16_t p = 0;

        // Map PinName to port + pinmask (only the pins we support)
        if (pin == PA_4 || pin == PA_5 || pin == PA_6 || pin == PA_7 || pin == PA_15) {
            port = GPIOA;
            p = (pin == PA_4 ) ? GPIO_PIN_4  :
                (pin == PA_5 ) ? GPIO_PIN_5  :
                (pin == PA_6 ) ? GPIO_PIN_6  :
                (pin == PA_7 ) ? GPIO_PIN_7  : GPIO_PIN_15;
        } else if (pin == PB_3 || pin == PB_4 || pin == PB_5 || pin == PB_9 || pin == PB_10 || pin == PB_12 || pin == PB_13 || pin == PB_14 || pin == PB_15) {
            port = GPIOB;
            p = (pin == PB_3 ) ? GPIO_PIN_3  :
                (pin == PB_4 ) ? GPIO_PIN_4  :
                (pin == PB_5 ) ? GPIO_PIN_5  :
                (pin == PB_9 ) ? GPIO_PIN_9  :
                (pin == PB_10) ? GPIO_PIN_10 :
                (pin == PB_12) ? GPIO_PIN_12 :
                (pin == PB_13) ? GPIO_PIN_13 :
                (pin == PB_14) ? GPIO_PIN_14 : GPIO_PIN_15;
        } else if (pin == PC_2 || pin == PC_3 || pin == PC_10 || pin == PC_11 || pin == PC_12) {
            port = GPIOC;
            p = (pin == PC_2 ) ? GPIO_PIN_2  :
                (pin == PC_3 ) ? GPIO_PIN_3  :
                (pin == PC_10) ? GPIO_PIN_10 :
                (pin == PC_11) ? GPIO_PIN_11 : GPIO_PIN_12;
        } else {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),
                       "Unsupported pin requested for SPI AF");
        }

        GPIO_InitStruct.Pin       = p;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = (pin == m_NSS) ? GPIO_PULLUP : GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = af;
        HAL_GPIO_Init(port, &GPIO_InitStruct);
    };

    do_pin(m_SCK);
    do_pin(m_MISO);
    do_pin(m_MOSI);
    do_pin(m_NSS);

    // Select SPI instance for HAL
    m_hspi.Instance = (m_instance == Instance::SPI_1) ? SPI1 :
                      (m_instance == Instance::SPI_2) ? SPI2 : SPI3;

    // DMA mapping (STM32F446):
    //
    // SPI1 → DMA2: RX Stream0/Ch3, TX Stream3/Ch3
    // SPI2 → DMA1: RX Stream3/Ch0, TX Stream4/Ch0
    // SPI3 → DMA1: RX Stream0/Ch0, TX Stream5/Ch0
    if (m_instance == Instance::SPI_1) {
        // RX
        m_dma_rx.Instance                 = DMA2_Stream0;
        m_dma_rx.Init.Channel             = DMA_CHANNEL_3;
        m_dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        m_dma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_rx.Init.Mode                = DMA_NORMAL;
        m_dma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_rx);
        __HAL_LINKDMA(&m_hspi, hdmarx, m_dma_rx);

        // TX
        m_dma_tx.Instance                 = DMA2_Stream3;
        m_dma_tx.Init.Channel             = DMA_CHANNEL_3;
        m_dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        m_dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_tx.Init.Mode                = DMA_NORMAL;
        m_dma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_tx);
        __HAL_LINKDMA(&m_hspi, hdmatx, m_dma_tx);
    }
    else if (m_instance == Instance::SPI_2) {
        // RX
        m_dma_rx.Instance                 = DMA1_Stream3;
        m_dma_rx.Init.Channel             = DMA_CHANNEL_0;
        m_dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        m_dma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_rx.Init.Mode                = DMA_NORMAL;
        m_dma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_rx);
        __HAL_LINKDMA(&m_hspi, hdmarx, m_dma_rx);

        // TX
        m_dma_tx.Instance                 = DMA1_Stream4;
        m_dma_tx.Init.Channel             = DMA_CHANNEL_0;
        m_dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        m_dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_tx.Init.Mode                = DMA_NORMAL;
        m_dma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_tx);
        __HAL_LINKDMA(&m_hspi, hdmatx, m_dma_tx);
    }
    else { // SPI_3
        // RX
        m_dma_rx.Instance                 = DMA1_Stream0;
        m_dma_rx.Init.Channel             = DMA_CHANNEL_0;
        m_dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        m_dma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_rx.Init.Mode                = DMA_NORMAL;
        m_dma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_rx);
        __HAL_LINKDMA(&m_hspi, hdmarx, m_dma_rx);

        // TX
        m_dma_tx.Instance                 = DMA1_Stream5;
        m_dma_tx.Init.Channel             = DMA_CHANNEL_0;
        m_dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        m_dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_tx.Init.Mode                = DMA_NORMAL;
        m_dma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_tx);
        __HAL_LINKDMA(&m_hspi, hdmatx, m_dma_tx);
    }

    // Precompute flag masks for fast TC checks/clears
    const uint32_t te = te_flag_for(m_dma_rx.Instance);
    const uint32_t ht = ht_flag_for(m_dma_rx.Instance);
    const uint32_t fe = fe_flag_for(m_dma_rx.Instance);
    m_rx_tc_flag   = tc_flag_for(m_dma_rx.Instance);
    m_rx_all_flags = (m_rx_tc_flag | te | ht | fe);

    const uint32_t te2 = te_flag_for(m_dma_tx.Instance);
    const uint32_t ht2 = ht_flag_for(m_dma_tx.Instance);
    const uint32_t fe2 = fe_flag_for(m_dma_tx.Instance);
    m_tx_tc_flag   = tc_flag_for(m_dma_tx.Instance);
    m_tx_all_flags = (m_tx_tc_flag | te2 | ht2 | fe2);
}

bool SpiSlaveDMA::validatePins() const {
    auto in = [](PinName p, std::initializer_list<PinName> set) {
        for (auto q : set)
            if (p == q)
                return true;
        return false;
    };

    bool sck_ok=false, miso_ok=false, mosi_ok=false, nss_ok=false;

    if (m_instance == Instance::SPI_1) {
        sck_ok  = in(m_SCK , {PA_5, PB_3});
        miso_ok = in(m_MISO, {PA_6, PB_4});
        mosi_ok = in(m_MOSI, {PA_7, PB_5});
        nss_ok  = in(m_NSS , {PA_4, PA_15});
    } else if (m_instance == Instance::SPI_2) {
        sck_ok  = in(m_SCK , {PB_10, PB_13});
        miso_ok = in(m_MISO, {PB_14, PC_2});
        mosi_ok = in(m_MOSI, {PB_15, PC_3});
        nss_ok  = in(m_NSS , {PB_9, PB_12});
    } else { // SPI_3
        sck_ok  = in(m_SCK , {PC_10, PB_3});
        miso_ok = in(m_MISO, {PC_11, PB_4});
        mosi_ok = in(m_MOSI, {PC_12, PB_5});
        nss_ok  = in(m_NSS , {PA_4, PA_15});
    }

    if (!(sck_ok && miso_ok && mosi_ok && nss_ok)) return false;

    if ((m_SCK == m_MISO) || (m_SCK == m_MOSI) || (m_SCK == m_NSS) ||
        (m_MISO == m_MOSI) || (m_MISO == m_NSS) ||
        (m_MOSI == m_NSS)) return false;

    return true;
}

void SpiSlaveDMA::resetSPIPeripheral() {
    printf("[%s] Resetting SPI peripheral due to %lu consecutive failures\n",
           instanceName(), (unsigned long)m_consecutive_failures);

    HAL_SPI_DMAStop(&m_hspi);
    HAL_SPI_DeInit(&m_hspi);
    HAL_DMA_DeInit(&m_dma_rx);
    HAL_DMA_DeInit(&m_dma_tx);
    ThisThread::sleep_for(2ms);

    configureGPIOandDMA();

    if (HAL_SPI_Init(&m_hspi) != HAL_OK) {
        printf("[%s] ERROR: Failed to reinitialize SPI after reset\n", instanceName());
        return;
    }

    m_consecutive_failures = 0;
    printf("[%s] SPI peripheral reset complete - ready\n", instanceName());
}

const char* SpiSlaveDMA::instanceName() const {
    switch (m_instance) {
        case Instance::SPI_1: return "SPI1";
        case Instance::SPI_2: return "SPI2";
        case Instance::SPI_3: return "SPI3";
    }
    return "?";
}
