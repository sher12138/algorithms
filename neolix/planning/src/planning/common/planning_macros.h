#pragma once

#include "common/macros.h"
#include "planning_logger.h"

namespace neodrive {
namespace planning {

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN_PLANNING(classname) \
  classname(const classname &) = delete;             \
  classname &operator=(const classname &) = delete;

// quit if condition is met
#ifndef QUIT_IF_WARN
#define QUIT_IF_WARN(CONDITION, RET, WARN_CODE, MSG, ...) \
  do {                                                    \
    if (CONDITION) {                                      \
      LOG_WARN(WARN_CODE, MSG, ##__VA_ARGS__);            \
      return RET;                                         \
    }                                                     \
  } while (0)
#endif

#ifndef QUIT_IF_ERROR
#define QUIT_IF_ERROR(CONDITION, RET, ERROR_CODE, MSG, ...) \
  do {                                                      \
    if (CONDITION) {                                        \
      LOG_ERROR(ERROR_CODE, MSG, ##__VA_ARGS__);            \
      return RET;                                           \
    }                                                       \
  } while (0)
#endif

#ifndef QUIT_IF
#define QUIT_IF(CONDITION, RET, LEVEL, MSG, ...)    \
  do {                                              \
    if (CONDITION) {                                \
      LOG_DEBUG("quit", LEVEL, MSG, ##__VA_ARGS__); \
      return RET;                                   \
    }                                               \
  } while (0)
#endif

#ifndef QUIT_IF_VOID
#define QUIT_IF_VOID(CONDITION, LEVEL, MSG, ...)    \
  do {                                              \
    if (CONDITION) {                                \
      LOG_DEBUG("quit", LEVEL, MSG, ##__VA_ARGS__); \
      return;                                       \
    }                                               \
  } while (0);
#endif

#ifndef QUIT_IF_QUIET
#define QUIT_IF_QUIET(CONDITION, RET) \
  if (CONDITION) {                    \
    return RET;                       \
  }
#endif

#ifndef CONTINUE_IF_QUIET
#define CONTINUE_IF_QUIET(CONDITION) \
  if (CONDITION) {                   \
    continue;                        \
  }
#endif

#ifndef CONTINUE_IF
#define CONTINUE_IF(CONDITION, LEVEL, MSG, ...)       \
  if (CONDITION) {                                    \
    LOG_DEBUG("continue", LEVEL, MSG, ##__VA_ARGS__); \
    continue;                                         \
  }
#endif

#ifndef BREAK_IF
#define BREAK_IF(CONDITION) \
  if (CONDITION) {          \
    break;                  \
  }
#endif

#ifndef RETURN_IF
#define RETURN_IF(condition)                                 \
  if (condition) {                                           \
    LOG_ERROR("The condition: {}, is not met.", #condition); \
    return;                                                  \
  }
#endif

#ifndef RETURN_VAL_IF
#define RETURN_VAL_IF(condition, val)                        \
  if (condition) {                                           \
    LOG_ERROR("The condition: {}, is not met.", #condition); \
    return val;                                              \
  }
#endif

}  // namespace planning
}  // namespace neodrive
