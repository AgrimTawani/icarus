#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "autonomy_core/clock.hpp"
#include "icarus/v1/drone_api.pb.h"

namespace icarus::drone_api {

class AuthorityManager {
 public:
  explicit AuthorityManager(const core::Clock& clock,
                            std::uint32_t maximum_lease_ms = 10'000);

  [[nodiscard]] v1::ControlLeaseResponse Acquire(
      const v1::AcquireControlRequest& request);
  [[nodiscard]] v1::ControlLeaseResponse Renew(
      const v1::RenewControlRequest& request);
  [[nodiscard]] bool Release(const v1::ReleaseControlRequest& request);
  [[nodiscard]] bool IsAuthorized(const v1::CommandContext& context) const;
  [[nodiscard]] std::optional<v1::ControlAuthority> Current() const;

 private:
  [[nodiscard]] static int Priority(v1::ControlRole role);
  [[nodiscard]] bool ExpiredLocked() const;
  [[nodiscard]] static std::string NewId();

  const core::Clock& clock_;
  const std::uint32_t maximum_lease_ms_;
  mutable std::mutex mutex_;
  std::optional<v1::ControlAuthority> current_;
  std::string session_id_;
};

}  // namespace icarus::drone_api
