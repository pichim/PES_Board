/*
 * Motion.h
 * Copyright (c) 2022, ZHAW
 * All rights reserved.
 */

#ifndef MOTION_H_
#define MOTION_H_

#include <cstdlib>

/**
 * This class keeps the motion values <code>position</code> and <code>velocity</code>, and
 * offers methods to increment these values towards a desired target position or velocity.
 * <br/>
 * To increment the current motion values, this class uses a simple 2nd order motion planner.
 * This planner calculates the motion to the target position or velocity with the various motion
 * phases, based on given limits for the profile velocity, acceleration and deceleration.
 * <br/>
 * Note that the trajectory is calculated every time the motion state is incremented.
 * This allows to change the target position or velocity, as well as the limits for profile
 * velocity, acceleration and deceleration at any time.
 */
class Motion {
    
    public:
        
        double      position;       /**< The position value of this motion, given in [m] or [rad]. */
        float       velocity;       /**< The velocity value of this motion, given in [m/s] or [rad/s]. */
        
        explicit    Motion();
        explicit    Motion(double position, float velocity);
        explicit    Motion(const Motion& motion);
        virtual     ~Motion();
        void        set(double position, float velocity);
        void        set(const Motion& motion);
        void        setPosition(double position);
        double      getPosition();
        void        setVelocity(float velocity);
        float       getVelocity();
        void        setProfileVelocity(float profileVelocity);
        void        setProfileAcceleration(float profileAcceleration);
        void        setProfileDeceleration(float profileDeceleration);
        void        setLimits(float profileVelocity, float profileAcceleration, float profileDeceleration);
        float       getTimeToPosition(double targetPosition);
        void        incrementToVelocity(float targetVelocity, float period);
        void        incrementToPosition(double targetPosition, float period);
        
    private:
        
        static const float  DEFAULT_LIMIT;  // default value for limits
        static const float  MINIMUM_LIMIT;  // smallest value allowed for limits
        
        float       profileVelocity;
        float       profileAcceleration;
        float       profileDeceleration;
};

#endif /* MOTION_H_ */

