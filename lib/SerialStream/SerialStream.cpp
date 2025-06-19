#include "SerialStream.h"

SerialStream::SerialStream(PinName tx,
                           PinName rx,
                           uint8_t num_of_floats,
                           int baudrate) : _buffer_size(sizeof(float) * S_STREAM_CLAMP(num_of_floats))
#if S_STREAM_DO_USE_SERIAL_PIPE
                                         , _SerialPipe(tx, rx, baudrate, 1 + 1, // serial pipe extects 1 byte more
                                                       sizeof(float) * S_STREAM_CLAMP(num_of_floats) + 1)
{
#else
                                         , _BufferedSerial(tx, rx, baudrate)
{
    _BufferedSerial.set_blocking(false);
#endif
}

void SerialStream::write(const float val)
{
    memcpy(&_buffer[_byte_cntr], &val, sizeof(float));
    _byte_cntr += sizeof(float);

    // send the data if the buffer is full immediately
    if (_byte_cntr == _buffer_size)
        send();
}

void SerialStream::send()
{
    // return if there is no data to send
    if (_byte_cntr == 0)
        return;

    // blocking so that we guarantee that the number of floats is sent once
    // and it will not occupy the buffer for actual data to be sent
    sendNumOfFloatsOnce();

#if S_STREAM_DO_USE_SERIAL_PIPE
    // TODO: handle case where bytes_writeable < _byte_cntr
    const int bytes_writeable = _SerialPipe.writeable();
    if (bytes_writeable > 0)
        _SerialPipe.put(&_buffer, _byte_cntr, false);
#else
    // TODO: handle case where bytes_written < _byte_cntr
    if (_BufferedSerial.writable())
        ssize_t bytes_written = _BufferedSerial.write(_buffer, _byte_cntr);
#endif
    _byte_cntr = 0;
}

bool SerialStream::startByteReceived()
{   
    return checkByteReceived(_start, S_STREAM_START_BYTE);
}

void SerialStream::reset()
{
    memset(&_buffer, 0, sizeof(_buffer));
    _byte_cntr = 0;
    resetByteMsg(_start);
    _send_num_of_floats_once = false;
}

bool SerialStream::checkByteReceived(byte_msg_t& byte_msg, const uint8_t byte_expected)
{
    // logic so that we can read non-blocking
    if (byte_msg.byte == byte_expected)
        return true;

#if S_STREAM_DO_USE_SERIAL_PIPE
    const int bytes_readable = _SerialPipe.readable();
    if (!byte_msg.received && (bytes_readable > 0)) {
        byte_msg.received = true;
        _SerialPipe.get(&byte_msg.byte, 1, false);
    }
#else
    if (!byte_msg.received && _BufferedSerial.readable()) {
        byte_msg.received = true;
        _BufferedSerial.read(&byte_msg.byte, 1);
    }
#endif
    return false;
}

void SerialStream::resetByteMsg(byte_msg_t& byte_msg)
{
    byte_msg.byte = 0;
    byte_msg.received = false;
}

void SerialStream::sendNumOfFloatsOnce()
{
    if (_send_num_of_floats_once)
        return;
    else {
        _send_num_of_floats_once = true;
        const uint8_t num_of_floats = _byte_cntr / sizeof(float);
#if S_STREAM_DO_USE_SERIAL_PIPE
        _SerialPipe.put(&num_of_floats, 1, true);
#else
        _BufferedSerial.write(&num_of_floats, 1);
#endif
    }
}
