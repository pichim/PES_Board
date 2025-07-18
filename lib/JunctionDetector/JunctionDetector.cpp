#include "JunctionDetector.h"

/* helpers ---------------------------------------------------------------- */
static inline bool pattern_left(const LineFollower& lf)  { return lf.isLedActive() && lf.getAngleDegrees() >  15.0f; }
static inline bool pattern_right(const LineFollower& lf) { return lf.isLedActive() && lf.getAngleDegrees() < -15.0f; }

JunctionDetector::JunctionDetector(LineFollower& lf, float debounce_ms)
: m_lf(lf), m_debounce_ms(debounce_ms)
{
    m_juncTimer.start();
    m_deadTimer.start();    // for dead-ends
    m_goalHoldTimer.start();  // black hold-time


}

bool JunctionDetector::isJunction()
{
    /* rule: ≥2 non-adjacent sensors dark → junction */
    bool left  = pattern_left(m_lf);
    bool right = pattern_right(m_lf);

    /* ignore cases where both edge sensors are *very* dark – that’s the goal line */
    bool goal_edges = (m_lf.getLeftEdgeIntensity()  > 0.95f) &&
                    (m_lf.getRightEdgeIntensity() > 0.95f);

    bool junc = left && right && !goal_edges;


    bool rising = junc && !m_prev_junc &&
                  m_juncTimer.elapsed_time().count() > m_debounce_ms*1000;

    if (rising) m_juncTimer.reset();
    m_prev_junc = junc;
    return rising;
}

bool JunctionDetector::isGoal()
{
    /* all sensors dark AND centred */
    bool all_black = m_lf.isLedActive() &&
                     fabsf(m_lf.getAngleRadians()) < 0.05f &&
                     (m_lf.getLeftEdgeIntensity()  > 0.95f) &&
                     (m_lf.getRightEdgeIntensity() > 0.95f);

    if (all_black) {
        /* start / keep running the hold-timer */
        if (m_goalHoldTimer.elapsed_time() >= 200ms) {
            return true;                     // black held for ≥200 ms ⇒ GOAL
        }
    } else {
        m_goalHoldTimer.reset();             // left black → reset
    }
    return false;                            // not yet a goal
}


bool JunctionDetector::isDeadEnd()
{
    bool white = !m_lf.isLedActive();                 // no black seen

    bool rising = white && !m_prev_dead &&
                  m_deadTimer.elapsed_time().count() > m_debounce_ms*1000;

    if (rising) m_deadTimer.reset();                  // debounce
    m_prev_dead = white;

    return rising;                                    // true once per dead-end
}




JunctionType JunctionDetector::classify() const
{
    bool left  = pattern_left(m_lf);
    bool right = pattern_right(m_lf);

    if (left && right) return JunctionType::X;             // crude but works
    if (left)          return JunctionType::L;
    if (right)         return JunctionType::T;
    return JunctionType::DEAD;
}
