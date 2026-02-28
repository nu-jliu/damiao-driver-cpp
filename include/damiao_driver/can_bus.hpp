#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "damiao_driver/comm_bus.hpp"

namespace dm
{

  class CanBus : public CommBus
  {
  public:
    /// Private key type to allow make_shared while keeping constructor private.
    struct PrivateTag
    {
    private:
      friend class CanBus;
      PrivateTag() = default;
    };

    /// Get or create a CanBus instance for the given interface.
    /// If an instance already exists for this interface, returns the existing one.
    static std::shared_ptr<CanBus> create(const std::string &interface);

    /// @note Public only for make_shared; use create() instead.
    explicit CanBus(PrivateTag, const std::string &interface);

    ~CanBus() override;

    void send(const CanFrame &frame) override;
    bool receive(CanFrame &frame, const int timeout_ms = 10) override;

    /// Returns the interface name this bus is bound to
    const std::string &interfaceName() const;

  private:

    int socket_fd_ = -1;
    std::string interface_name_;

    static std::mutex registry_mutex_;
    static std::unordered_map<std::string, std::weak_ptr<CanBus>> registry_;
  };

} // namespace dm
