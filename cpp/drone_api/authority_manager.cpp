#include "drone_api/authority_manager.hpp"

#include <algorithm>
#include <array>
#include <iomanip>
#include <random>
#include <sstream>

namespace icarus::drone_api {
namespace {

v1::ControlLeaseResponse Denied(v1::ReasonCode code, std::string message) {
  v1::ControlLeaseResponse response;
  response.set_granted(false);
  response.set_reason_code(code);
  response.set_message(std::move(message));
  return response;
}

}  // namespace

AuthorityManager::AuthorityManager(const core::Clock& clock,
                                   std::uint32_t maximum_lease_ms)
    : clock_(clock), maximum_lease_ms_(maximum_lease_ms) {}

v1::ControlLeaseResponse AuthorityManager::Acquire(
    const v1::AcquireControlRequest& request) {
  if (request.session_id().empty() || request.client_id().empty() ||
      request.vehicle_id().empty() ||
      request.requested_role() == v1::CONTROL_ROLE_UNSPECIFIED ||
      request.requested_role() == v1::CONTROL_ROLE_OBSERVER) {
    return Denied(v1::REASON_CODE_INVALID_ARGUMENT,
                  "session, client, vehicle and controlling role are required");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (current_ && !ExpiredLocked() && session_id_ != request.session_id() &&
      Priority(request.requested_role()) <= Priority(current_->role())) {
    return Denied(v1::REASON_CODE_CONTROL_CONFLICT,
                  "an equal or higher-priority controller owns the vehicle");
  }

  const auto duration = std::clamp(request.requested_lease_ms(), 250U,
                                   maximum_lease_ms_);
  v1::ControlAuthority authority;
  authority.set_lease_id(NewId());
  authority.set_client_id(request.client_id());
  authority.set_role(request.requested_role());
  authority.set_expires_at_unix_ms(clock_.NowUnixMs() + duration);
  current_ = authority;
  session_id_ = request.session_id();

  v1::ControlLeaseResponse response;
  response.set_granted(true);
  response.set_reason_code(v1::REASON_CODE_OK);
  response.set_message("control lease granted");
  *response.mutable_authority() = authority;
  return response;
}

v1::ControlLeaseResponse AuthorityManager::Renew(
    const v1::RenewControlRequest& request) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_ || ExpiredLocked() || session_id_ != request.session_id() ||
      current_->lease_id() != request.lease_id()) {
    current_.reset();
    session_id_.clear();
    return Denied(v1::REASON_CODE_CONTROL_LEASE_EXPIRED,
                  "control lease is absent, expired or not owned by session");
  }
  const auto duration = std::clamp(request.requested_lease_ms(), 250U,
                                   maximum_lease_ms_);
  current_->set_expires_at_unix_ms(clock_.NowUnixMs() + duration);

  v1::ControlLeaseResponse response;
  response.set_granted(true);
  response.set_reason_code(v1::REASON_CODE_OK);
  response.set_message("control lease renewed");
  *response.mutable_authority() = *current_;
  return response;
}

bool AuthorityManager::Release(const v1::ReleaseControlRequest& request) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_ || session_id_ != request.session_id() ||
      current_->lease_id() != request.lease_id()) {
    return false;
  }
  current_.reset();
  session_id_.clear();
  return true;
}

bool AuthorityManager::IsAuthorized(const v1::CommandContext& context) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_ && !ExpiredLocked() &&
         current_->lease_id() == context.control_lease_id() &&
         current_->client_id() == context.client_id() &&
         session_id_ == context.session_id();
}

std::optional<v1::ControlAuthority> AuthorityManager::Current() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_ || ExpiredLocked()) {
    return std::nullopt;
  }
  return current_;
}

int AuthorityManager::Priority(v1::ControlRole role) {
  switch (role) {
    case v1::CONTROL_ROLE_SAFETY_SUPERVISOR:
      return 50;
    case v1::CONTROL_ROLE_MANUAL_OPERATOR:
      return 40;
    case v1::CONTROL_ROLE_SCRIPTED_AUTONOMY:
      return 30;
    case v1::CONTROL_ROLE_DCM_AUTONOMY:
      return 20;
    case v1::CONTROL_ROLE_MAINTAINER:
      return 10;
    case v1::CONTROL_ROLE_OBSERVER:
    case v1::CONTROL_ROLE_UNSPECIFIED:
      return 0;
    default:
      return 0;
  }
}

bool AuthorityManager::ExpiredLocked() const {
  return current_ && current_->expires_at_unix_ms() <= clock_.NowUnixMs();
}

std::string AuthorityManager::NewId() {
  std::array<unsigned char, 16> bytes{};
  std::random_device device;
  for (auto& byte : bytes) {
    byte = static_cast<unsigned char>(device());
  }
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : bytes) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

}  // namespace icarus::drone_api
