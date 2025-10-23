#ifndef ROI_DEBUG_H
#define ROI_DEBUG_H

#ifndef DEBUG_INFO
#define DEBUG_INFO 1
#endif  // DEBUG_INFO
#ifndef DEBUG_EVENT
#define DEBUG_EVENT 1
#endif  // DEBUG_EVENT
#ifndef DEBUG_ERROR
#define DEBUG_ERROR 1
#endif  // DEBUG_ERROR

namespace ROI_DEBUG {
// define common output format
#if defined(__AVR__)
#define ___debug_format(message) \
    _Generic((message), char*: (message), const char*: F(message), default: (message))

#define ___debug_out(message) Serial.println(___debug_format(message))  // INTERNAL USE ONLY
#define ___debug_out_val(message, value)    \
    Serial.print(___debug_format(message)); \
    Serial.println(___debug_format(value))  // INTERNAL USE ONLY
#else
#define ___debug_out(message) \
    static_assert(false, "Debug output not yet supported on this architecture")
#define ___debug_out_val(message, value) \
    static_assert(false, "Debug output with value not yet supported on this architecture")
#endif

// define common initialization function
#if defined(__AVR__)
#define __debug_init() Serial.begin(115200);
#else
#define __debug_init() \
    static_assert(false, "Debug initialization not yet supported on this architecture")
#endif

// define debug macros for use in codebase
#if DEBUG_INFO
#define __debug_info(message) ___debug_out(message)
#define __debug_info_val(message, value) ___debug_out_val(message, value)
#else
#define __debug_info(message) (void)0
#define __debug_info_val(message, value) (void)0
#endif  // DEBUG_INFO

#if DEBUG_EVENT
#define __debug_event(message) ___debug_out(message)
#define __debug_event_val(message, value) ___debug_out_val(message, value)
#else
#define __debug_event(message) (void)0
#define __debug_event_val(message, value) (void)0
#endif  // DEBUG_EVENT

#if DEBUG_ERROR
#define __debug_error(message) ___debug_out(message)
#define __debug_error_val(message, value) ___debug_out_val(message, value)
#else
#define __debug_error(message) (void)0
#define __debug_error_val(message, value) (void)0
#endif  // DEBUG_ERROR

}  // namespace ROI_DEBUG
#endif  // ROI_DEBUG_H
