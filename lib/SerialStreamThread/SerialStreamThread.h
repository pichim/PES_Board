#pragma once
// Simple, fast UART streamer in its own periodic thread.
// - Backend via S_STREAM_DO_USE_SERIAL_PIPE (1=SerialPipe, 0=BufferedSerial)
// - Static TX ring (no heap), batched writes
// - First header (1 byte) sent once: actual first frame length
//
// Usage:
//   #define S_STREAM_DO_USE_SERIAL_PIPE 1  // or 0
//   #include "SerialStreamThread.h"
//   SerialStreamThread ss(TX, RX, 30, 2000000, /*period_us*/1000,
//                         osPriorityAboveNormal, /*stack*/1536);
//   ss.enable();
//   ss.write(f1); ... ss.send();   // first send() sends header=frame length

#include "mbed.h"
#include "SerialStream.h"     // S_STREAM_* macros; selects SerialPipe vs BufferedSerial
#include "RealTimeThread.h"
#include <cstring>
#include <errno.h>

#ifndef SST_RING_SIZE
#define SST_RING_SIZE (8u * 1024u)   // 8 KiB fits well on small MCUs
#endif

class SerialStreamThread : public RealTimeThread {
public:
    explicit SerialStreamThread(PinName tx,
                                PinName rx,
                                uint8_t num_of_floats = S_STREAM_NUM_OF_FLOATS_MAX,
                                int baudrate = 2000000,
                                uint32_t period_us = 1000,
                                osPriority prio = osPriorityAboveNormal,
                                uint32_t stack = OS_STACK_SIZE)
        : RealTimeThread(period_us, prio, stack)
#if S_STREAM_DO_USE_SERIAL_PIPE
        , _pipe(tx, rx, baudrate,
                /*rx*/64,
                /*tx*/ (int)(sizeof(float) * S_STREAM_CLAMP(num_of_floats) + 256))
#else
        , _ser(tx, rx, baudrate)
#endif
        , _frame_cap_bytes((uint8_t)(sizeof(float) * S_STREAM_CLAMP(num_of_floats)))
    {
#if !S_STREAM_DO_USE_SERIAL_PIPE
        _ser.set_blocking(false);
#endif
        _start.byte = 0; _start.received = false;
        static_assert(sizeof(float) == 4, "Protocol assumes 32-bit float");
    }

    ~SerialStreamThread() override { disable(); }

    // --- Producer-side API (like SerialStream) ---
    inline void write(float v) {
        std::memcpy(&_frame[_frame_len], &v, sizeof(float));
        _frame_len += sizeof(float);
        if (_frame_len == _frame_cap_bytes) send(); // auto-flush => header=max
    }

    inline void send() {
        if (_frame_len == 0) return;
        sendNumFloatsOnce(_frame_len);           // header = actual first frame size
        enqueue(_frame, _frame_len);
        _frame_len = 0;
    }

    inline bool startByteReceived() {
        // cached
        if (_start.byte == S_STREAM_START_BYTE) return true;
#if S_STREAM_DO_USE_SERIAL_PIPE
        if (!_start.received && _pipe.readable()) {
            char b; if (_pipe.get(&b, 1, false) == 1) { _start.byte=(uint8_t)b; _start.received=true; }
        }
#else
        if (!_start.received && _ser.readable()) {
            uint8_t b; if (_ser.read(&b, 1) == 1) { _start.byte=b; _start.received=true; }
        }
#endif
        return _start.byte == S_STREAM_START_BYTE;
    }

    inline void reset() {
        std::memset(_frame, 0, sizeof(_frame));
        _frame_len = 0;
        _start = {0,false};
        _header_sent = false;
        core_util_critical_section_enter();
        _tx_head = _tx_tail = 0;
        core_util_critical_section_exit();
    }

protected:
    void executeTask() override {
        // 1) Drain TX ring to UART
        for (int loops = 0; loops < 8; ++loops) {
            size_t used = ring_used(); if (!used) break;
            size_t cont = contiguous_used(); if (!cont) break;

            uint8_t* p = &_tx[_tx_tail];
#if S_STREAM_DO_USE_SERIAL_PIPE
            // Blocking put: simplest and fastest for a dedicated worker
            int w = _pipe.put(reinterpret_cast<const char*>(p), (int)cont, /*blocking*/true);
            if (w > 0) { advance_tail((size_t)w); continue; }
#else
            ssize_t w = _ser.write(p, cont);
            if (w > 0) { advance_tail((size_t)w); continue; }
            if (w == -EAGAIN || w == -EWOULDBLOCK) break;
#endif
            break;
        }

        // 2) Opportunistic RX for start byte
        for (int i=0; i<4 && !_start.received; ++i) {
#if S_STREAM_DO_USE_SERIAL_PIPE
            if (!_pipe.readable()) break;
            char b; if (_pipe.get(&b,1,false)==1){ _start.byte=(uint8_t)b; _start.received=true; }
#else
            if (!_ser.readable()) break;
            uint8_t b; if (_ser.read(&b,1)==1){ _start.byte=b; _start.received=true; }
#endif
        }
    }

private:
#if S_STREAM_DO_USE_SERIAL_PIPE
    SerialPipe _pipe;
#else
    BufferedSerial _ser;
#endif

    // TX ring (single producer/consumer, simple volatile indices)
    static constexpr size_t RING = SST_RING_SIZE;
    uint8_t _tx[RING]{};                 // .bss
    volatile size_t _tx_head{0};         // producer
    volatile size_t _tx_tail{0};         // consumer

    // Staging buffer for one frame (max floats)
    char     _frame[sizeof(float) * S_STREAM_NUM_OF_FLOATS_MAX]{};
    uint8_t  _frame_cap_bytes;           // capacity (bytes)
    uint8_t  _frame_len{0};              // current length (bytes)

    struct { uint8_t byte; bool received; } _start{0,false};
    bool _header_sent{false};

    // --- Ring helpers ---
    inline size_t ring_used() const {
        size_t h=_tx_head, t=_tx_tail; return (h>=t)?(h-t):(RING-(t-h));
    }
    inline size_t ring_free() const { return RING - 1 - ring_used(); }
    inline size_t contiguous_used() const {
        size_t h=_tx_head, t=_tx_tail; return (h>=t)?(h-t):(RING-t);
    }
    inline void advance_head(size_t n) { _tx_head = (_tx_head + n) % RING; }
    inline void advance_tail(size_t n) { _tx_tail = (_tx_tail + n) % RING; }

    inline void enqueue(const void* src, size_t n) {
        const uint8_t* s = static_cast<const uint8_t*>(src);
        while (n) {
            size_t space = ring_free();
            if (space == 0) { ThisThread::yield(); continue; } // backpressure allowed
            size_t chunk = (n < space) ? n : space;

            size_t head_to_end = RING - _tx_head;
            size_t c1 = (chunk < head_to_end) ? chunk : head_to_end;
            std::memcpy(&_tx[_tx_head], s, c1);
            advance_head(c1);
            size_t rem = chunk - c1;
            if (rem) {
                std::memcpy(&_tx[_tx_head], s + c1, rem);
                advance_head(rem);
            }
            s += chunk; n -= chunk;
        }
    }

    inline void sendNumFloatsOnce(uint8_t bytes_first_frame) {
        if (_header_sent) return;
        _header_sent = true;
        uint8_t nf = bytes_first_frame / sizeof(float);
        uint8_t nf_max = _frame_cap_bytes / sizeof(float);
        if (nf > nf_max) nf = nf_max;
        enqueue(&nf, 1);
    }
};
