#ifndef JUNCTION_DETECTOR_H_
#define JUNCTION_DETECTOR_H_

#include "LineFollower.h"
#include "MazeGraph.h"
#include "mbed.h"

/** small utility that inspects LineFollower’s sensor bar every 2 ms
 *  and decides if we just crossed a junction or the goal line          */
class JunctionDetector {
public:
    explicit JunctionDetector(LineFollower& lf,
                              float debounce_ms = 50.0f);

    /** returns a rising-edge flag: true only once per physical junction  */
    bool isJunction();

    /** true while the robot sits on the double-width goal line          */
    bool isGoal();

    /** classify current junction pattern                                */
    JunctionType classify() const;

        /** rising edge when the bar sees *no* black line (dead end) */
    bool isDeadEnd();


private:
    LineFollower& m_lf;
    Timer  m_juncTimer;       // already exists as m_timer

    Timer  m_deadTimer;          // independent debounce timer
    bool   m_prev_dead{false};
    Timer  m_goalHoldTimer;   // NEW – for 200 ms hold-time

    bool          m_prev_junc{false};
    float         m_debounce_ms;

};

#endif /* JUNCTION_DETECTOR_H_ */
