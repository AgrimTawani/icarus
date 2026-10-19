#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "mavlink_gateway/mavlink_gateway.hpp"

namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

class LoopbackPeer {
 public:
  LoopbackPeer() {
    listener_ = socket(AF_INET, SOCK_STREAM, 0);
    Require(listener_ >= 0, "create loopback listener");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    Require(bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
            "bind loopback listener");
    Require(listen(listener_, 1) == 0, "listen on loopback");
    socklen_t length = sizeof(address);
    Require(getsockname(listener_, reinterpret_cast<sockaddr*>(&address), &length) == 0,
            "read assigned port");
    port_ = ntohs(address.sin_port);
  }
  ~LoopbackPeer() {
    Close();
    if (listener_ >= 0) close(listener_);
  }
  std::uint16_t port() const { return port_; }
  void Accept() {
    peer_ = accept(listener_, nullptr, nullptr);
    Require(peer_ >= 0, "accept gateway connection");
  }
  void Close() {
    if (peer_ >= 0) {
      shutdown(peer_, SHUT_RDWR);
      close(peer_);
      peer_ = -1;
    }
  }
 private:
  int listener_ = -1;
  int peer_ = -1;
  std::uint16_t port_ = 0;
};

void AwaitPeerClose(icarus::mavlink_gateway::ArdupilotGateway& gateway) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (gateway.IsConnected() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Require(!gateway.IsConnected(), "reader notices peer close before cleanup");
}

void TestPeerCloseThenRepeatedCleanup() {
  std::atomic<int> callbacks{0};
  {
    LoopbackPeer peer;
    icarus::mavlink_gateway::ArdupilotGateway gateway;
    gateway.SetStateCallback([&](const auto&) { ++callbacks; });
    Require(gateway.Connect("127.0.0.1", peer.port()), "connect gateway");
    peer.Accept();
    peer.Close();
    AwaitPeerClose(gateway);
    gateway.Disconnect();
    const auto after_disconnect = callbacks.load();
    gateway.Disconnect();
    Require(callbacks == after_disconnect, "repeated cleanup must not invoke callbacks");
  }
  Require(callbacks == 0, "peer-closed cleanup and destructor do not republish state");
}

void TestReconnectReapsOldThreads() {
  std::atomic<int> callbacks{0};
  {
    LoopbackPeer peer;
    icarus::mavlink_gateway::ArdupilotGateway gateway;
    gateway.SetStateCallback([&](const auto&) { ++callbacks; });
    Require(gateway.Connect("127.0.0.1", peer.port()), "connect first session");
    peer.Accept();
    peer.Close();
    AwaitPeerClose(gateway);
    // No explicit Disconnect: Connect must reap the terminated reader and
    // heartbeat rather than assigning over joinable std::thread objects.
    Require(gateway.Connect("127.0.0.1", peer.port()), "reconnect dropped session");
    peer.Accept();
    Require(gateway.IsConnected(), "replacement connection is live");
    Require(!gateway.WaitForReady(10), "new connection needs fresh vehicle telemetry");
    gateway.Disconnect();
    Require(callbacks == 1, "live explicit disconnect publishes one final state");
    gateway.Disconnect();
    Require(callbacks == 1, "second disconnect is callback-free");
  }
  Require(callbacks == 1, "destruction after disconnect is callback-free");
}

void TestDestructorAfterPeerClose() {
  LoopbackPeer peer;
  {
    icarus::mavlink_gateway::ArdupilotGateway gateway;
    Require(gateway.Connect("127.0.0.1", peer.port()), "connect before destructor");
    peer.Accept();
    peer.Close();
    AwaitPeerClose(gateway);
  }
  // Reaching here demonstrates that neither joinable thread called terminate.
}
}  // namespace

int main() {
  try {
    TestPeerCloseThenRepeatedCleanup();
    TestReconnectReapsOldThreads();
    TestDestructorAfterPeerClose();
    std::cout << "MAVLink gateway lifecycle tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
