#include "autonomy_core/clock.hpp"

#include <chrono>

namespace icarus::core {

std::int64_t SystemClock::NowUnixMs() const {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

}  // namespace icarus::core
