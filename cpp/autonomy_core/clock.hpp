#pragma once

#include <cstdint>

namespace icarus::core {

class Clock {
 public:
  virtual ~Clock() = default;
  [[nodiscard]] virtual std::int64_t NowUnixMs() const = 0;
};

class SystemClock final : public Clock {
 public:
  [[nodiscard]] std::int64_t NowUnixMs() const override;
};

}  // namespace icarus::core
