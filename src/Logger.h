#pragma once

#include <Arduino.h>

// Log levels
#define LOG_LEVEL_NONE    0
#define LOG_LEVEL_ERROR   1
#define LOG_LEVEL_WARN    2
#define LOG_LEVEL_INFO    3
#define LOG_LEVEL_DEBUG   4

// Set the log level here
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

// Macro to print logs with a specific level
#define LOG_PRINT(level, levelStr, msg)  \
  if (level <= LOG_LEVEL) {              \
    Serial.print("[");                   \
    Serial.print(levelStr);              \
    Serial.print("] ");                  \
    Serial.println(msg);                 \
  }

// Macro to print formatted logs with a specific level
#define LOG_PRINTF(level, levelStr, format, ...)  \
  if (level <= LOG_LEVEL) {                       \
    Serial.print("[");                            \
    Serial.print(levelStr);                       \
    Serial.print("] ");                           \
    Serial.printf(format, ##__VA_ARGS__);         \
  }

// Logging macros
#define LOG_ERROR(msg)   LOG_PRINT(LOG_LEVEL_ERROR, "ERROR", msg)
#define LOG_WARN(msg)    LOG_PRINT(LOG_LEVEL_WARN, "WARN", msg)
#define LOG_INFO(msg)    LOG_PRINT(LOG_LEVEL_INFO, "INFO", msg)
#define LOG_DEBUG(msg)   LOG_PRINT(LOG_LEVEL_DEBUG, "DEBUG", msg)

#define LOG_ERRORF(format, ...)   LOG_PRINTF(LOG_LEVEL_ERROR, "ERROR", format, ##__VA_ARGS__)
#define LOG_WARNF(format, ...)    LOG_PRINTF(LOG_LEVEL_WARN, "WARN", format, ##__VA_ARGS__)
#define LOG_INFOF(format, ...)    LOG_PRINTF(LOG_LEVEL_INFO, "INFO", format, ##__VA_ARGS__)
#define LOG_DEBUGF(format, ...)   LOG_PRINTF(LOG_LEVEL_DEBUG, "DEBUG", format, ##__VA_ARGS__)