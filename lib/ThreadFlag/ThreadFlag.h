/*
 * ThreadFlag.h
 * Copyright (c) 2020, ZHAW
 * All rights reserved.
 * @authors M. Honegger / honr and M. Peter / pmic / pichim
 */

#ifndef THREAD_FLAG_H_
#define THREAD_FLAG_H_

#include "mbed.h"

/**
 * This class manages the handling of unique thread flags to trigger rtos threads.
 *
 * The ThreadFlag class automatically allocates and manages unique thread flags
 * for RTOS thread synchronization. It ensures that each instance gets a unique
 * flag value from the available 30 flags (bit positions 0-29).
 *
 * IMPORTANT: Always check isValid() after creating a ThreadFlag instance to
 * ensure successful allocation. The system supports a maximum of 30 concurrent
 * ThreadFlag instances.
 *
 * Example usage:
 * ```cpp
 * ThreadFlag flag;
 * if (flag.isValid()) {
 *     Thread thread;
 *     thread.start(...);
 *     thread.flags_set(flag); // Use flag for synchronization
 * } else {
 *     // Handle allocation failure
 *     printf("ThreadFlag allocation failed\n");
 * }
 * ```
 *
 * Thread Safety: All operations are thread-safe and protected by mutex.
 * Resource Management: Flags are automatically released in the destructor.
 */
class ThreadFlag
{
public:
    explicit ThreadFlag();
    virtual ~ThreadFlag();
    virtual unsigned int read() const;
    operator unsigned int() const;

    // Validation method to check if flag allocation was successful
    bool isValid() const { return threadFlag != 0; }

    // Static utility methods for diagnostics
    static unsigned int getUsedFlagCount();
    static unsigned int getAvailableFlagCount() { return 30 - getUsedFlagCount(); }

private:
    static unsigned int threadFlags; // variable that holds all assigned thread flags
    unsigned int threadFlag;         // thread flag of this object
    static Mutex mutex;              // shared mutex for all ThreadFlag instances
};

#endif /* THREAD_FLAG_H_ */
