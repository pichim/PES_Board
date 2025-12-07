/**
 * @file SerialStream.h
 * @brief This file defines the SerialStream class.
 * @author M. Peter / pmic / pichim
 */

#ifndef SERIAL_STREAM_H_
#define SERIAL_STREAM_H_

#include "mbed.h"

#include <chrono>

extern "C" {
void serialStreamDmaTxIrq(void);
void serialStreamDmaRxIrq(void);
void serialStreamUsartIrq(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
}

#define S_STREAM_DO_USE_SERIAL_PIPE 0 // legacy macro retained for compatibility only
#define S_STREAM_NUM_OF_FLOATS_MAX 30 // tested at 2 kHz 20 floats
#define S_STREAM_CLAMP(x) (x <= S_STREAM_NUM_OF_FLOATS_MAX ? x : S_STREAM_NUM_OF_FLOATS_MAX)
#define S_STREAM_START_BYTE 255

class SerialStream {
public:
    explicit SerialStream(PinName tx,
                          PinName rx,
                          uint8_t num_of_floats = S_STREAM_NUM_OF_FLOATS_MAX,
                          int baudrate = 2000000);
    virtual ~SerialStream() = default;

    void write(const float val);
    void send();
    bool startByteReceived();
    void reset();

    // Timing helpers so callers do not need their own timers
    void startTiming();
    float measureDeltaUs();
    void resetTiming();

    // Grant IRQ handlers access to internals without exposing them publicly
    friend void serialStreamDmaTxIrq(void);
    friend void serialStreamDmaRxIrq(void);
    friend void serialStreamUsartIrq(void);
    friend void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

private:
    char _buffer[sizeof(float) * S_STREAM_NUM_OF_FLOATS_MAX];
    uint8_t _bufferSize;
    uint8_t _byteCount{0};
    uint8_t _floatsInFrame{0};
    UART_HandleTypeDef _uart{};
    DMA_HandleTypeDef  _txDma{};
    DMA_HandleTypeDef  _rxDma{};
    size_t _txLenActive{0};

    // TX ring buffer for DMA
    static constexpr size_t TX_RING_SIZE = 13 * 1024;
    alignas(4) uint8_t _txRing[TX_RING_SIZE];
    volatile uint16_t _txHead{0};
    volatile uint16_t _txTail{0};
    volatile bool _txDmaActive{false};

    // RX ring buffer for DMA (circular)
    static constexpr size_t RX_RING_SIZE = 256;
    alignas(4) uint8_t _rxRing[RX_RING_SIZE];
    volatile uint16_t _rxTail{0};

    bool _sentNumFloats{false};

    // Timing state
    Timer _timer;
    std::chrono::microseconds _timePrev{0};
    bool _timerStarted{false};

    size_t enqueueBytes(const uint8_t *data, size_t length);
    void startDmaTxUnsafe();
    void onDmaTxComplete(size_t sentBytes);
    void startDmaRx();
    uint16_t rxHeadFromDma() const;
    bool ensureSpace(size_t needed);
    void startFrameIfNeeded();
};
#endif /* SERIAL_STREAM_H_ */
