#include "Encoder.h"

Encoder::Encoder(PinName enc_a_pin,
                 PinName enc_b_pin,
                 float counts_per_turn,
                 float fcut,
                 float Ts) : m_EncoderCounter(enc_a_pin, enc_b_pin)
                           , m_counts_per_turn(counts_per_turn)
                           , m_Ts(Ts)
{
    m_lowPass1.lowPass1Init(fcut, Ts);
    reset();
}

void Encoder::reset()
{
    // m_Mutex.lock();
    m_lowPass1.reset(0.0f);
    m_EncoderCounter.reset();
    m_counts = m_count_previous = m_EncoderCounter.read();
    // m_Mutex.unlock();
}

Encoder::encoder_signals_t Encoder::update(float sign)
{
    // convert to rotations per second resp. rotations
    const float velocity_gain = 1.0f / m_Ts;
    const float rotation_gain = 1.0f / m_counts_per_turn;

    // m_Mutex.lock();
    // avoid overflow by using short for counts
    const short count = m_EncoderCounter.read();
    const short count_delta = count - m_count_previous;
    m_count_previous = count;

    // total counts
    m_counts += count_delta;

    // encoder signals
    encoder_signals_t encoder_signals;
    encoder_signals.counts = m_counts;
    const float rotation_increment = static_cast<float>(count_delta) / m_counts_per_turn;
    encoder_signals.velocity = m_lowPass1.apply(sign * velocity_gain * rotation_increment);
    encoder_signals.rotations = sign * rotation_gain * static_cast<float>(m_counts);
    // m_Mutex.unlock();

    return encoder_signals;
}
