/**
 * Copyright (c) 2015 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
 * =======================================================================
 */

#if !defined(__DEBUG_H_)
#define __DEBUG_H_

#include "mbed_config.h"


#if defined(ENABLE_THREAD_SAFE_LOGGING)

#include "mbed.h"

extern EventQueue s_logging_event_queue;

#endif


#if defined(ENABLE_LOGGING)

#include "DigiLogger.h"

#if !(defined(ENABLE_THREAD_SAFE_LOGGING))

#define digi_log(...)  DigiLog::DigiLogger::log_format(__VA_ARGS__);

#else

#define digi_log(...)  s_logging_event_queue.call(&DigiLog::DigiLogger::log_format, __VA_ARGS__);

#endif

#else
#define digi_log(...)  do {} while(0)
#endif

#if defined(ENABLE_ASSERTIONS)
#include "mbed.h"
#if !(defined assert)

#if !(defined(ENABLE_THREAD_SAFE_LOGGING))

    #define assert(expr)        if (!(expr)) {                                      \
                                    DigiLog::DigiLogger::log_format(LogLevelNone, "Assertion failed: %s, file %s, line %d\n", \
                                         #expr, __FILE__, __LINE__);                \
                                    mbed_die();                                     \
                                }

#else

    #define assert(expr)        if (!(expr)) {                                      \
                                    s_logging_event_queue.call(&DigiLog::DigiLogger::log_format, LogLevelNone, "Assertion failed: %s, file %s, line %d\n", #expr, __FILE__, __LINE__ ); \
                                    mbed_die();                                     \
                                }

#endif

#endif
#else
#define assert(expr)
#endif

#endif /* __DEBUG_H_ */
