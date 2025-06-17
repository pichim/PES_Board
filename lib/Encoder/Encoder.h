#ifndef ENCODER_H_
#define ENCODER_H_

#include "EncoderCounter.h"
#include "IIRFilter.h"

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

class Encoder
{
public:
    explicit Encoder(PinName enc_a_pin,
                     PinName enc_b_pin,
                     float counts_per_turn,
                     float fcut,
                     float Ts);
    virtual ~Encoder() = default;

    typedef struct encoder_signals_s {
        long counts{0};
        float velocity{0.0f};
        // TODO: should rotations be a double?
        float rotations{0.0f};
    } encoder_signals_t;

    void reset();
    encoder_signals_t update(float sign = 1.0f);

private:
    EncoderCounter m_EncoderCounter;
    IIRFilter m_lowPass1;
    // Mutex m_Mutex; // not needed when only used in one thread

    long m_counts;
    short m_count_previous;
    float m_counts_per_turn;
    float m_Ts;
};
#endif /* ENCODER_H_ */
