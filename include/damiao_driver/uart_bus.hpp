#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <boost/asio.hpp>

#include "damiao_driver/comm_bus.hpp"

namespace dm
{

  class UartBus : public CommBus
  {
  public:
    /// Private key type to allow make_shared while keeping constructor private.
    struct PrivateTag
    {
    private:
      friend class UartBus;
      PrivateTag() = default;
    };

    /// Get or create a UartBus instance for the given device.
    /// If an instance already exists for this device, returns the existing one.
    static std::shared_ptr<UartBus> create(const std::string &device,
                                           const unsigned int baud_rate = 921600);

    /// @note Public only for make_shared; use create() instead.
    UartBus(PrivateTag, const std::string &device, const unsigned int baud_rate);

    ~UartBus() override;

    void send(const CanFrame &frame) override;
    bool receive(CanFrame &frame, const int timeout_ms = 10) override;

    /// Returns the device path this bus is bound to
    const std::string &device_name() const;

  private:

    void send_raw_(const std::vector<uint8_t> &data);
    std::vector<uint8_t> receive_raw_(const size_t max_len, const int timeout_ms);

    boost::asio::io_context io_;
    boost::asio::serial_port port_;
    std::string device_name_;

    static std::mutex registry_mutex_;
    static std::unordered_map<std::string, std::weak_ptr<UartBus>> registry_;
  };

} // namespace dm
