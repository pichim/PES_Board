/*
 * ThreadFlag.cpp
 * Copyright (c) 2020, ZHAW
 * All rights reserved.
 */

#include "ThreadFlag.h"

using namespace std;

unsigned int ThreadFlag::threadFlags = 0;
Mutex ThreadFlag::mutex;

/**
 * Creates a signal object and assigns a unique flag.
 */
ThreadFlag::ThreadFlag()
{
    mutex.lock();

    unsigned int n = 0;
    while ((((1 << n) & threadFlags) > 0) && (n < 30))
        n++;
    if (n >= 30) {
        // All flags are in use - this is a critical error
        printf("ThreadFlag: CRITICAL ERROR - All 30 thread flags are in use! System may become unstable.\n");
        threadFlag = 0; // Invalid flag to indicate failure
        mutex.unlock();
        return;
    }

    threadFlag = (1 << n);
    threadFlags |= threadFlag; // CRITICAL FIX: Mark flag as used

    mutex.unlock();
}

/**
 * Deletes the signal object and releases the assigned flag.
 */
ThreadFlag::~ThreadFlag()
{
    mutex.lock();

    threadFlags &= ~threadFlag;

    mutex.unlock();
}

/**
 * Gets the assigned thread flag.
 */
unsigned int ThreadFlag::read() const
{
    return threadFlag;
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
ThreadFlag::operator unsigned int() const
{
    return read();
}

/**
 * Static utility method to count how many flags are currently in use.
 * Useful for diagnostics and monitoring system resource usage.
 */
unsigned int ThreadFlag::getUsedFlagCount()
{
    unsigned int count = 0;
    unsigned int flags = threadFlags;

    // Count set bits using Brian Kernighan's algorithm
    while (flags) {
        count++;
        flags &= flags - 1; // Remove the lowest set bit
    }

    return count;
}
