#include "Encoder.h"

Encoder::Encoder(PinName enc_a_pin,
                 PinName enc_b_pin,
                 float counts_per_turn,
                 float fcut,
                 float D,
                 float Ts) : m_EncoderCounter(enc_a_pin, enc_b_pin)
                           , m_counts_per_turn(counts_per_turn)
                           , m_Ts(Ts)
{
    m_lowPass2.lowPass2Init(fcut, D, Ts);
    reset();
}

void Encoder::reset()
{
    m_Mutex.lock();
    m_EncoderCounter.reset();
    m_lowPass2.reset(0.0f);
    m_encoder_signals.counts = m_counts_previous = m_EncoderCounter.read();
    m_encoder_signals.velocity = 0.0f;
    m_encoder_signals.rotations = 0.0f;
    m_Mutex.unlock();
}

Encoder::encoder_signals_t Encoder::update(float sign)
{
    static float velocity_gain = 1.0f / m_Ts;
    static float rotation_gain = 1.0f / m_counts_per_turn;

    m_Mutex.lock();
    const float rotation_increment = updateEncoderAndReturnDeltaCounts();
    m_encoder_signals.velocity = m_lowPass2.apply(sign * velocity_gain * rotation_increment);
    m_encoder_signals.rotations = sign * rotation_gain * static_cast<float>(m_encoder_signals.counts);
    encoder_signals_t encoder_signals = m_encoder_signals;  // Copy under protection
    m_Mutex.unlock();
    
    return encoder_signals;
}

float Encoder::updateEncoderAndReturnDeltaCounts()
{  
    // avoid overflow
    const short counts = m_EncoderCounter.read();
    const short counts_delta = counts - m_counts_previous;
    m_counts_previous = counts;
    m_encoder_signals.counts += counts_delta;
    return static_cast<float>(counts_delta) / m_counts_per_turn;
}
