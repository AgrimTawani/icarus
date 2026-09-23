#include "perception/lidar/airsim_lidar_adapter.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace icarus::perception {
namespace {

constexpr std::array<std::uint8_t, 4> kMagic{'I', 'C', 'A', 'R'};
constexpr std::uint16_t kVersion = 1;
constexpr std::size_t kHeaderSize = 24;
constexpr std::size_t kPointSize = 12;
constexpr std::uint16_t kMaximumPoints = 4096;

template <typename T>
T ReadLittleEndian(const std::uint8_t* data) {
  T result{};
  std::memcpy(&result, data, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(reinterpret_cast<std::uint8_t*>(&result),
               reinterpret_cast<std::uint8_t*>(&result) + sizeof(T));
#endif
  return result;
}

}  // namespace

AirSimLidarAdapter::AirSimLidarAdapter(std::string bind_address,
                                       std::uint16_t port)
    : bind_address_(std::move(bind_address)), port_(port) {}

AirSimLidarAdapter::~AirSimLidarAdapter() { Stop(); }

bool AirSimLidarAdapter::Start(Callback callback) {
  if (running_) return true;
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ < 0) return false;
  timeval timeout{};
  timeout.tv_usec = 200'000;
  if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                 sizeof(timeout)) != 0) {
    ::close(socket_);
    socket_ = -1;
    return false;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  if (inet_pton(AF_INET, bind_address_.c_str(), &address.sin_addr) != 1 ||
      bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) !=
          0) {
    ::close(socket_);
    socket_ = -1;
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(callback);
  }
  running_ = true;
  receive_thread_ = std::thread(&AirSimLidarAdapter::ReceiveLoop, this);
  return true;
}

void AirSimLidarAdapter::Stop() {
  if (!running_.exchange(false)) return;
  if (receive_thread_.joinable()) receive_thread_.join();
  if (socket_ >= 0) ::close(socket_);
  socket_ = -1;
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callback_ = nullptr;
}

bool AirSimLidarAdapter::Healthy(std::uint32_t maximum_age_ms) const {
  const auto last = last_sample_ms_.load();
  return running_ && last != 0 && NowMs() - last <= maximum_age_ms;
}

bool AirSimLidarAdapter::DecodePacket(const std::uint8_t* data,
                                      std::size_t size, RangeScan* scan) {
  if (data == nullptr || scan == nullptr || size < kHeaderSize ||
      !std::equal(kMagic.begin(), kMagic.end(), data)) {
    return false;
  }
  const auto version = ReadLittleEndian<std::uint16_t>(data + 4);
  const auto count = ReadLittleEndian<std::uint16_t>(data + 6);
  if (version != kVersion || count > kMaximumPoints ||
      size != kHeaderSize + static_cast<std::size_t>(count) * kPointSize) {
    return false;
  }
  const auto captured = ReadLittleEndian<std::int64_t>(data + 8);
  const auto sequence = ReadLittleEndian<std::uint64_t>(data + 16);
  if (captured <= 0 || sequence == 0) return false;
  RangeScan decoded;
  decoded.sensor_id = "airsim-lidar";
  decoded.captured_at_unix_ms = captured;
  decoded.sequence = sequence;
  decoded.frame = SensorFrame::kBodyFrd;
  decoded.minimum_range_m = 1.0;
  decoded.maximum_range_m = 100.0;
  decoded.points.reserve(count);
  for (std::uint16_t index = 0; index < count; ++index) {
    const auto* point = data + kHeaderSize + index * kPointSize;
    const float x = ReadLittleEndian<float>(point);
    const float y = ReadLittleEndian<float>(point + 4);
    const float z = ReadLittleEndian<float>(point + 8);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }
    decoded.points.push_back({x, y, z, 1.0});
  }
  *scan = std::move(decoded);
  return true;
}

void AirSimLidarAdapter::ReceiveLoop() {
  std::array<std::uint8_t, kHeaderSize + kMaximumPoints * kPointSize> buffer{};
  while (running_) {
    const auto received = recvfrom(socket_, buffer.data(), buffer.size(), 0,
                                   nullptr, nullptr);
    if (received <= 0) continue;
    RangeScan scan;
    if (!DecodePacket(buffer.data(), static_cast<std::size_t>(received),
                      &scan)) {
      continue;
    }
    scan.received_at_unix_ms = NowMs();
    // Reject replayed/future-dated samples before they enter the shared map.
    if (scan.captured_at_unix_ms > scan.received_at_unix_ms + 1000 ||
        scan.received_at_unix_ms - scan.captured_at_unix_ms > 5000) {
      continue;
    }
    last_sample_ms_ = scan.received_at_unix_ms;
    Callback callback;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback = callback_;
    }
    if (callback) callback(scan);
  }
}

std::int64_t AirSimLidarAdapter::NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace icarus::perception
