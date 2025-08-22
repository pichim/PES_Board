#include "mbed.h"
#include <cstring>

extern "C" {
#include "stm32f4xx_hal.h"  // We use the STM32 HAL directly (raw SPI + DMA)
}

using namespace std::chrono;

// -----------------------------
// Protocol constants
// -----------------------------
#define SPI_HEADER_MASTER 0x55
#define SPI_HEADER_SLAVE  0x45
#define SPI_NUM_FLOATS    3
#define SPI_MSG_SIZE      (1 + SPI_NUM_FLOATS * 4 + 1) // header + 3 floats (3*4) + crc

// -----------------------------
// CRC8 table (poly 0x07, init 0x00)
// -----------------------------
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

// Table-driven CRC8 for speed (same as your previous code)
static inline uint8_t calculate_crc8(const uint8_t *buffer, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) crc = CRC8_TAB[crc ^ buffer[i]];
    return crc;
}

static bool verify_checksum(const uint8_t *buffer, size_t length, uint8_t expected_crc)
{
    return calculate_crc8(buffer, length) == expected_crc;
}

// -----------------------------
// Simple container for payloads + counters
// -----------------------------
class SPIData {
public:
    SPIData() { init(); }
    ~SPIData() = default;

    float    data[SPI_NUM_FLOATS];     // The 3 floats in our protocol
    uint32_t message_count;            // Count of valid frames received
    uint32_t failed_count;             // Count of CRC/header failures
    uint32_t last_delta_time_us;       // Time since previous valid frame
    uint32_t readout_time_us;          // (unused, available if needed)

    void init() {
        for (int i = 0; i < SPI_NUM_FLOATS; i++) data[i] = 0.0f;
        message_count = 0;
        failed_count = 0;
        last_delta_time_us = 0;
        readout_time_us = 0;
    }
};

// -----------------------------
// HAL handles for SPI2 + DMA
// -----------------------------
static SPI_HandleTypeDef  hspi2;
static DMA_HandleTypeDef  hdma_spi2_rx;
static DMA_HandleTypeDef  hdma_spi2_tx;

/*
 * MSP init callback for SPI2.
 * HAL_SPI_Init() calls this to let us:
 *  - Enable clocks
 *  - Configure GPIO pins to SPI alternate functions
 *  - Configure and link DMA streams to SPI2 RX/TX
 *
 * IMPORTANT: We are *not* enabling DMA interrupts here.
 * We poll the "Transfer Complete" flags instead (non-blocking, but no ISRs).
 */
extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance != SPI2) return;

    // --- enable peripheral clocks ---
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();        // SPI2 uses DMA1 on F446

    // --- GPIO alternate-function config for SPI2 pins ---
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PB12 -> NSS, PB10 -> SCK  (AF5)
    GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;            // push-pull AF
    GPIO_InitStruct.Pull      = GPIO_NOPULL;                // master drives NSS
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  // fast slew
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PC2 -> MISO, PC3 -> MOSI  (AF5)
    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // --- DMA mapping (STM32F446 typical choice) ---
    // RX: DMA1 Stream3, Channel 0  (SPI2_RX)
    hdma_spi2_rx.Instance                 = DMA1_Stream3;
    hdma_spi2_rx.Init.Channel             = DMA_CHANNEL_0;
    hdma_spi2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc              = DMA_MINC_ENABLE;       // write sequentially into buffer
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi2_rx.Init.Mode                = DMA_NORMAL;            // 1 shot per frame
    hdma_spi2_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_spi2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_spi2_rx);
    __HAL_LINKDMA(hspi, hdmarx, hdma_spi2_rx); // link to SPI

    // TX: DMA1 Stream4, Channel 0  (SPI2_TX)
    hdma_spi2_tx.Instance                 = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel             = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc              = DMA_MINC_ENABLE;       // read sequentially from buffer
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode                = DMA_NORMAL;            // 1 shot per frame
    hdma_spi2_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_spi2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_spi2_tx);
    __HAL_LINKDMA(hspi, hdmatx, hdma_spi2_tx);

    // NOTE: No NVIC IRQ enables here — we poll completion flags in main().
}

int main()
{
    // -----------------------------
    // Configure SPI2 peripheral (slave, mode 0, 8-bit, HW NSS)
    // -----------------------------
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_SLAVE;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;       // full-duplex
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;           // CPOL=0
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;            // CPHA=0  => mode 0
    hspi2.Init.NSS               = SPI_NSS_HARD_INPUT;         // hardware NSS on PB12
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE; // we do our own CRC8
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;    // ignored in slave
    MBED_ASSERT(HAL_SPI_Init(&hspi2) == HAL_OK);

    DigitalOut led(LED1);

    // These buffers are touched by DMA — keep them static (not on the stack).
    // SAFETY RULE (no ping–pong here):
    //   - Only read/write these buffers *after* the current DMA transfer is fully done.
    //   - We detect completion by BOTH RX and TX DMA "Transfer Complete" flags (TC).
    static uint8_t buffer_rx[SPI_MSG_SIZE];
    static uint8_t buffer_tx[SPI_MSG_SIZE];

    SPIData received_data;
    SPIData reply_data;

    // Prepare payload to send. You can update reply_data.data[] at any time,
    // but you must rebuild buffer_tx just before (re-)arming DMA.
    reply_data.data[0] = 11.11f;
    reply_data.data[1] = 22.22f;
    reply_data.data[2] = 33.33f;

    // For timing between valid frames (purely informational)
    Timer timer;
    timer.start();
    microseconds time_previous = timer.elapsed_time();

    // Build the first TX frame now, before arming DMA.
    buffer_tx[0] = SPI_HEADER_SLAVE;
    std::memcpy(&buffer_tx[1], reply_data.data, SPI_NUM_FLOATS * sizeof(float));
    buffer_tx[SPI_MSG_SIZE - 1] = calculate_crc8(buffer_tx, SPI_MSG_SIZE - 1);

    printf("SPI Communication (DMA polling, non-blocking) started. Waiting for master...\r\n");

    // -----------------------------
    // Arm first DMA full-frame transfer
    //
    // After this call returns:
    //  - SPI/DMA are "ready and waiting".
    //  - When the master (Pi) asserts CS low and clocks exactly SPI_MSG_SIZE bytes,
    //    DMA hardware moves them in/out without CPU involvement.
    //  - This call does NOT block; we simply proceed to main loop and poll for completion.
    // -----------------------------
    MBED_ASSERT(HAL_SPI_TransmitReceive_DMA(&hspi2, buffer_tx, buffer_rx, SPI_MSG_SIZE) == HAL_OK);

    while (true) {
        // ---------------------------------------------------------
        // NON-BLOCKING completion detection:
        // We poll the DMA "Transfer Complete" (TC) flags for both streams.
        // Why both?
        //   - RX TC means "SPI->Memory transfer finished" (buffer_rx valid).
        //   - TX TC means "Memory->SPI transfer finished" (buffer_tx fully sent).
        // We only consider the frame "done" when **both** are true.
        //
        // Flag groups on F4 (by stream index):
        //   - RX stream: DMA1 Stream3 -> TCIF3_7 group
        //   - TX stream: DMA1 Stream4 -> TCIF0_4 group
        //
        // NOTE: If you ever switch streams/channels in MSP, update these groups.
        // ---------------------------------------------------------
        const bool rx_done = (__HAL_DMA_GET_FLAG(&hdma_spi2_rx, DMA_FLAG_TCIF3_7) != 0U); // Stream3 TC flag
        const bool tx_done = (__HAL_DMA_GET_FLAG(&hdma_spi2_tx, DMA_FLAG_TCIF0_4) != 0U); // Stream4 TC flag

        if (rx_done && tx_done) {
            // Clear both TC flags to "acknowledge" completion.
            __HAL_DMA_CLEAR_FLAG(&hdma_spi2_rx, DMA_FLAG_TCIF3_7);
            __HAL_DMA_CLEAR_FLAG(&hdma_spi2_tx, DMA_FLAG_TCIF0_4);

            // Put HAL SPI back to READY and stop both DMA streams.
            // This is the point where it becomes SAFE to read buffer_rx and
            // rebuild buffer_tx, because DMA is no longer touching them.
            HAL_SPI_DMAStop(&hspi2);

            // -----------------------------
            // Process the just-received frame
            // -----------------------------
            const uint8_t received_checksum = buffer_rx[SPI_MSG_SIZE - 1];

            if (verify_checksum(buffer_rx, SPI_MSG_SIZE - 1, received_checksum)) {
                // Extract payload floats (STM32F4 is little-endian, matches Python "<f")
                std::memcpy(received_data.data, &buffer_rx[1], SPI_NUM_FLOATS * sizeof(float));
                received_data.message_count++;

                // Measure time since last valid frame (for info/debug)
                const auto now = timer.elapsed_time();
                received_data.last_delta_time_us = duration_cast<microseconds>(now - time_previous).count();
                time_previous = now;

                led = !led; // heartbeat on each valid frame
                printf("Message: %lu | Delta Time: %lu us | "
                       "Received: [%.2f, %.2f, %.2f] | "
                       "Header: 0x%02X | Failed: %lu\r\n",
                       received_data.message_count,
                       received_data.last_delta_time_us,
                       received_data.data[0], received_data.data[1], received_data.data[2],
                       SPI_HEADER_SLAVE,
                       received_data.failed_count);
            } else {
                // CRC mismatch — count and print for diagnostics
                received_data.failed_count++;
                const uint8_t expected = calculate_crc8(buffer_rx, SPI_MSG_SIZE - 1);
                printf("CRC failed! Expected: 0x%02X, Got: 0x%02X | Failed: %lu\r\n",
                       expected, received_checksum, received_data.failed_count);
            }

            // Optional safety: if an overrun occurred while we weren't armed,
            // clear it so the peripheral returns to a good state.
            // OVR happens if the master clocks data when the RX side isn't ready.
            if (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_OVR)) {
                volatile uint32_t dummy;
                dummy = hspi2.Instance->DR;
                dummy = hspi2.Instance->SR;
                (void)dummy;
            }

            // -----------------------------
            // Prepare the NEXT reply frame
            //
            // IMPORTANT: We rebuild buffer_tx BEFORE re-arming DMA.
            // If other threads update `reply_data` asynchronously, copy those values here.
            // With single buffering (no ping–pong), never modify buffer_tx while DMA is active.
            // -----------------------------
            buffer_tx[0] = SPI_HEADER_SLAVE;
            std::memcpy(&buffer_tx[1], reply_data.data, SPI_NUM_FLOATS * sizeof(float));
            buffer_tx[SPI_MSG_SIZE - 1] = calculate_crc8(buffer_tx, SPI_MSG_SIZE - 1);

            // -----------------------------
            // Re-arm SPI2 DMA immediately.
            // From now on, whenever the master asserts CS and clocks SPI_MSG_SIZE bytes,
            // the hardware will transfer them without blocking the CPU.
            //
            // If this call ever returns not-OK (rare), we force-stop and try once more.
            // -----------------------------
            HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(&hspi2, buffer_tx, buffer_rx, SPI_MSG_SIZE);
            if (st != HAL_OK) {
                HAL_SPI_DMAStop(&hspi2);
                (void)HAL_SPI_TransmitReceive_DMA(&hspi2, buffer_tx, buffer_rx, SPI_MSG_SIZE);
            }
        }

        // Do any other useful work; this loop is non-blocking.
        // ThisThread::yield(); // optional (lets other RTOS threads run)
    }
}
