#include <memory>
#include <stdexcept>

#include <catch2/catch_test_macros.hpp>

#include "damiao_driver/can_bus.hpp"
#include "damiao_driver/uart_bus.hpp"

// ---------------------------------------------------------------------------
// CanBus instantiation tests (require a live "vcan0" interface)
// ---------------------------------------------------------------------------

TEST_CASE("CanBus creation with vcan0", "[can_bus]")
{
  std::shared_ptr<dm::CanBus> bus;

  SECTION("create succeeds on available interface")
  {
    REQUIRE_NOTHROW(bus = dm::CanBus::create("vcan0"));
    REQUIRE(bus != nullptr);
    REQUIRE(bus->interfaceName() == "vcan0");
  }

  SECTION("create returns the same instance for the same interface")
  {
    auto bus1 = dm::CanBus::create("vcan0");
    auto bus2 = dm::CanBus::create("vcan0");
    REQUIRE(bus1.get() == bus2.get());
  }

  SECTION("instance is released when all shared_ptrs are destroyed")
  {
    auto bus1 = dm::CanBus::create("vcan0");
    auto *raw = bus1.get();
    bus1.reset();

    // A new create() should yield a fresh instance (different address)
    auto bus2 = dm::CanBus::create("vcan0");
    REQUIRE(bus2.get() != raw);
  }
}

TEST_CASE("CanBus creation fails on non-existent interface", "[can_bus]")
{
  REQUIRE_THROWS_AS(
      dm::CanBus::create("no_such_can_iface_xyz"),
      std::runtime_error);
}

// ---------------------------------------------------------------------------
// UartBus instantiation tests (require a device at "/tmp/ttyV0")
// ---------------------------------------------------------------------------

TEST_CASE("UartBus creation with /tmp/ttyV0", "[uart_bus]")
{
  std::shared_ptr<dm::UartBus> bus;

  SECTION("create succeeds on available device")
  {
    REQUIRE_NOTHROW(bus = dm::UartBus::create("/tmp/ttyV0"));
    REQUIRE(bus != nullptr);
    REQUIRE(bus->deviceName() == "/tmp/ttyV0");
  }

  SECTION("create returns the same instance for the same device")
  {
    auto bus1 = dm::UartBus::create("/tmp/ttyV0");
    auto bus2 = dm::UartBus::create("/tmp/ttyV0");
    REQUIRE(bus1.get() == bus2.get());
  }

  SECTION("instance is released when all shared_ptrs are destroyed")
  {
    auto bus1 = dm::UartBus::create("/tmp/ttyV0");
    auto *raw = bus1.get();
    bus1.reset();

    // A new create() should yield a fresh instance (different address)
    auto bus2 = dm::UartBus::create("/tmp/ttyV0");
    REQUIRE(bus2.get() != raw);
  }
}

TEST_CASE("UartBus creation fails on non-existent device", "[uart_bus]")
{
  REQUIRE_THROWS(dm::UartBus::create("/tmp/no_such_uart_device_xyz"));
}
