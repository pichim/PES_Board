#include "SerialStream.h"

#include "PeripheralPins.h"
#include "mbed_assert.h"
#include "mbed_critical.h"
#include "pinmap.h"

#include <cstring>

using namespace std::chrono;

namespace {
static SerialStream *g_serialStream = nullptr; // single instance used by IRQ handlers
inline SerialStream& self() {
    MBED_ASSERT(g_serialStream != nullptr);
    return *g_serialStream;
}

constexpr uint32_t UART_IRQ_PRIORITY = 15; // lowest priority to avoid jitter
constexpr uint32_t DMA_IRQ_PRIORITY  = 15;
}

extern "C" void serialStreamDmaTxIrq(void);
extern "C" void serialStreamDmaRxIrq(void);
extern "C" void serialStreamUsartIrq(void);

SerialStream::SerialStream(PinName tx,
                           PinName rx,
                           uint8_t num_of_floats,
                           int baudrate) : _bufferSize(sizeof(float) * S_STREAM_CLAMP(num_of_floats))
{
    // One instance only (IRQs rely on a global pointer)
    MBED_ASSERT(g_serialStream == nullptr);
    g_serialStream = this;

    // Resolve UART peripheral from pins (expect USART1 on L432KC)
    const int tx_peripheral = pinmap_peripheral(tx, PinMap_UART_TX);
    const int rx_peripheral = pinmap_peripheral(rx, PinMap_UART_RX);
    MBED_ASSERT(tx_peripheral == rx_peripheral);
    MBED_ASSERT(tx_peripheral == (int)UART_1); // only USART1 supported here

    pinmap_pinout(tx, PinMap_UART_TX);
    pinmap_pinout(rx, PinMap_UART_RX);

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
#if defined(DMAMUX1)
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
#endif

    _uart.Instance = USART1;
    _uart.Init.BaudRate = baudrate;
    _uart.Init.WordLength = UART_WORDLENGTH_8B;
    _uart.Init.StopBits = UART_STOPBITS_1;
    _uart.Init.Parity = UART_PARITY_NONE;
    _uart.Init.Mode = UART_MODE_TX_RX;
    _uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    _uart.Init.OverSampling = UART_OVERSAMPLING_16;
#if defined(UART_ONE_BIT_SAMPLE_DISABLE)
    _uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
#endif
#if defined(UART_PRESCALER_DIV1)
    _uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
#endif
    MBED_ASSERT(HAL_UART_Init(&_uart) == HAL_OK);

    // Configure DMA channel for USART1_TX (L432KC: DMA1 Channel4, request 2)
    _txDma.Instance = DMA1_Channel4;
#if defined(DMA_REQUEST_USART1_TX)
    _txDma.Init.Request = DMA_REQUEST_USART1_TX;
#elif defined(DMA_REQUEST_2)
    _txDma.Init.Request = DMA_REQUEST_2;
#endif
    _txDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    _txDma.Init.PeriphInc = DMA_PINC_DISABLE;
    _txDma.Init.MemInc = DMA_MINC_ENABLE;
    _txDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    _txDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    _txDma.Init.Mode = DMA_NORMAL;
    _txDma.Init.Priority = DMA_PRIORITY_LOW;
    MBED_ASSERT(HAL_DMA_Init(&_txDma) == HAL_OK);

    __HAL_LINKDMA(&_uart, hdmatx, _txDma);

    // Configure DMA channel for USART1_RX (L432KC: DMA1 Channel5, request 2) in circular mode
    _rxDma.Instance = DMA1_Channel5;
#if defined(DMA_REQUEST_USART1_RX)
    _rxDma.Init.Request = DMA_REQUEST_USART1_RX;
#elif defined(DMA_REQUEST_2)
    _rxDma.Init.Request = DMA_REQUEST_2;
#endif
    _rxDma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    _rxDma.Init.PeriphInc = DMA_PINC_DISABLE;
    _rxDma.Init.MemInc = DMA_MINC_ENABLE;
    _rxDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    _rxDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    _rxDma.Init.Mode = DMA_CIRCULAR;
    _rxDma.Init.Priority = DMA_PRIORITY_LOW;
    MBED_ASSERT(HAL_DMA_Init(&_rxDma) == HAL_OK);

    __HAL_LINKDMA(&_uart, hdmarx, _rxDma);

    // NVIC priorities
    NVIC_SetVector(DMA1_Channel4_IRQn, reinterpret_cast<uint32_t>(serialStreamDmaTxIrq));
    NVIC_SetPriority(DMA1_Channel4_IRQn, DMA_IRQ_PRIORITY);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    NVIC_SetVector(DMA1_Channel5_IRQn, reinterpret_cast<uint32_t>(serialStreamDmaRxIrq));
    NVIC_SetPriority(DMA1_Channel5_IRQn, DMA_IRQ_PRIORITY);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    NVIC_SetVector(USART1_IRQn, reinterpret_cast<uint32_t>(serialStreamUsartIrq));
    NVIC_SetPriority(USART1_IRQn, UART_IRQ_PRIORITY);
    NVIC_EnableIRQ(USART1_IRQn);

    // Start continuous RX DMA so we can poll the ring without blocking
    startDmaRx();
}

void SerialStream::startTiming()
{
    if (_timerStarted) {
        return;
    }

    _timer.reset();
    _timer.start();
    _timePrev = _timer.elapsed_time();
    _timerStarted = true;
}

float SerialStream::measureDeltaUs()
{
    if (!_timerStarted) {
        startTiming();
        return 0.0f;
    }

    const microseconds now = _timer.elapsed_time();
    const float dtime_us = static_cast<float>(duration_cast<microseconds>(now - _timePrev).count());
    _timePrev = now;
    return dtime_us;
}

void SerialStream::write(const float val)
{
    startFrameIfNeeded();

    memcpy(&_buffer[_byteCount], &val, sizeof(float));
    _byteCount += sizeof(float);
    _floatsInFrame++;

    if (_byteCount == _bufferSize) {
        send();
    }
}

void SerialStream::send()
{
    if (_byteCount == 0) {
        return;
    }

    // Total bytes needed this send (header once per stream + payload)
    size_t needed = _byteCount;
    if (!_sentNumFloats) {
        needed += 1; // header byte
    }

    // If we cannot fit header+payload, drop this frame atomically
    if (!ensureSpace(needed)) {
        _byteCount = 0;
        _floatsInFrame = 0;
        return;
    }

    // Send header once per stream, only if it enqueues successfully
    if (!_sentNumFloats) {
        if (_floatsInFrame == 0) {
            _byteCount = 0;
            return; // nothing meaningful to send
        }
        const uint8_t num_of_floats = _floatsInFrame; // header matches actual floats in this frame
        const size_t pushed_hdr = enqueueBytes(&num_of_floats, 1);
        if (pushed_hdr != 1) {
            _byteCount = 0;
            _floatsInFrame = 0;
            return;
        }
        _sentNumFloats = true;
        // _sentNumFloats is cleared only by reset(), so header is once-per-stream
    }

    // Enqueue payload (dt already inserted when frame started)
    const size_t pushed = enqueueBytes(reinterpret_cast<const uint8_t *>(_buffer), _byteCount);
    (void)pushed; // ensureSpace guaranteed capacity
    _byteCount = 0;
    _floatsInFrame = 0;
}

bool SerialStream::startByteReceived()
{
    const uint16_t head = rxHeadFromDma();
    while (_rxTail != head) {
        const uint8_t val = _rxRing[_rxTail];
        _rxTail = static_cast<uint16_t>((_rxTail + 1) % RX_RING_SIZE);
        if (val == S_STREAM_START_BYTE) {
            // Fresh session: clear TX/RX state so the next send re-emits the header
            reset();
            return true;
        }
    }
    return false;
}

void SerialStream::reset()
{
    resetTiming();

    // Stop TX DMA before clearing indices to avoid sending stale bytes
    HAL_UART_AbortTransmit(&_uart);
    HAL_DMA_Abort(&_txDma);

    core_util_critical_section_enter();
    _txHead = 0;
    _txTail = 0;
    _txLenActive = 0;
    _txDmaActive = false;

    _rxTail = 0;
    memset(_rxRing, 0, sizeof(_rxRing));

    memset(&_buffer, 0, sizeof(_buffer));
    _byteCount = 0;
    _floatsInFrame = 0;
    _sentNumFloats = false;
    core_util_critical_section_exit();

    startDmaRx();
}

void SerialStream::resetTiming()
{
    _timer.stop();
    _timer.reset();
    _timePrev = microseconds{0};
    _timerStarted = false;
}

size_t SerialStream::enqueueBytes(const uint8_t *data, size_t length)
{
    size_t pushed = 0;

    core_util_critical_section_enter();

    while (pushed < length) {
        const uint16_t next_head = static_cast<uint16_t>((_txHead + 1) % TX_RING_SIZE);
        if (next_head == _txTail) {
            break; // ring full
        }
        _txRing[_txHead] = data[pushed];
        _txHead = next_head;
        pushed++;
    }

    if (!_txDmaActive && _txHead != _txTail) {
        startDmaTxUnsafe();
    }

    core_util_critical_section_exit();

    return pushed;
}

void SerialStream::startDmaTxUnsafe()
{
    if (_txDmaActive || _txHead == _txTail) {
        return;
    }

    // Send the largest contiguous block from tail to either head or end of ring
    if (_txHead > _txTail) {
        _txLenActive = _txHead - _txTail;
    } else {
        _txLenActive = TX_RING_SIZE - _txTail;
    }

    _txDmaActive = true;
    if (HAL_UART_Transmit_DMA(&_uart, &_txRing[_txTail], _txLenActive) != HAL_OK) {
        _txDmaActive = false;
        _txLenActive = 0;
    }
}

void SerialStream::onDmaTxComplete(size_t sentBytes)
{
    core_util_critical_section_enter();

    _txTail = static_cast<uint16_t>((_txTail + sentBytes) % TX_RING_SIZE);
    _txLenActive = 0;
    _txDmaActive = false;

    if (_txHead != _txTail) {
        startDmaTxUnsafe();
    }

    core_util_critical_section_exit();
}

extern "C" void serialStreamDmaTxIrq(void)
{
    HAL_DMA_IRQHandler(&self()._txDma);
}

extern "C" void serialStreamDmaRxIrq(void)
{
    HAL_DMA_IRQHandler(&self()._rxDma);
}

extern "C" void serialStreamUsartIrq(void)
{
    HAL_UART_IRQHandler(&self()._uart);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &self()._uart) {
        self().onDmaTxComplete(self()._txLenActive);
    }
}

void SerialStream::startDmaRx()
{
    // Stop any ongoing RX DMA so re-arming cannot return HAL_BUSY
    HAL_UART_AbortReceive(&_uart);
    HAL_DMA_Abort(&_rxDma);

    core_util_critical_section_enter();
    _rxTail = 0;
    core_util_critical_section_exit();
    // HAL will drive the circular ring; we poll the head via DMA counter
    const HAL_StatusTypeDef st = HAL_UART_Receive_DMA(&_uart, _rxRing, RX_RING_SIZE);
    if (st != HAL_OK) {
        // RX not armed; leave tail at 0 and let caller decide next steps
        return;
    }
}

uint16_t SerialStream::rxHeadFromDma() const
{
    const uint16_t remaining = __HAL_DMA_GET_COUNTER(const_cast<DMA_HandleTypeDef*>(&_rxDma));
    return static_cast<uint16_t>((RX_RING_SIZE - remaining) % RX_RING_SIZE);
}

bool SerialStream::ensureSpace(size_t needed)
{
    // Returns true if ring has at least 'needed' free slots
    core_util_critical_section_enter();
    const uint16_t head = _txHead;
    const uint16_t tail = _txTail;
    size_t used = (head >= tail) ? (head - tail) : (TX_RING_SIZE - (tail - head));
    size_t free = TX_RING_SIZE - 1 - used;
    const bool has_space = free >= needed;
    core_util_critical_section_exit();
    return has_space;
}

void SerialStream::startFrameIfNeeded()
{
    if (_byteCount != 0) {
        return;
    }

    const float dt_us = measureDeltaUs();
    memcpy(&_buffer[0], &dt_us, sizeof(float));
    _byteCount = sizeof(float);
    _floatsInFrame = 1; // dt
}
