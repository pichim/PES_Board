/**
 * @file SerialStream.h
 * @brief This file defines the SerialStream class.
 * @author M. Peter / pmic / pichim
 */

#ifndef SERIAL_STREAM_H_
#define SERIAL_STREAM_H_

#define S_STREAM_DO_USE_SERIAL_PIPE true

#if S_STREAM_DO_USE_SERIAL_PIPE
    #include "serial_pipe.h"
#else
    #include "mbed.h"
#endif

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

private:
    char _buffer[sizeof(float) * S_STREAM_NUM_OF_FLOATS_MAX];
    uint8_t _buffer_size;
    uint8_t _byte_cntr{0};
#if S_STREAM_DO_USE_SERIAL_PIPE
    SerialPipe _SerialPipe;
#else
    BufferedSerial _BufferedSerial;
#endif

    typedef struct byte_msg_s {
        uint8_t byte{0};
        bool received{false};
    } byte_msg_t;

    byte_msg_t _start;
    bool _send_num_of_floats_once{false};

    bool checkByteReceived(byte_msg_t& byte_msg, const uint8_t byte_expected);
    void resetByteMsg(byte_msg_t& byte_msg);
    void sendNumOfFloatsOnce();
};
#endif /* SERIAL_STREAM_H_ */
