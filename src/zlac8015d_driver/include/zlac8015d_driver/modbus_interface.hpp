#pragma once

#include <cstdint>
#include <memory>
#include <string>

struct _modbus;

namespace zlac8015d_driver
{

class ModbusInterface
{
public:
  ModbusInterface() = default;
  ~ModbusInterface();

  ModbusInterface(const ModbusInterface &) = delete;
  ModbusInterface & operator=(const ModbusInterface &) = delete;

  bool connect(const std::string & serial_port, int baudrate, char parity, int data_bits,
    int stop_bits, int driver_id, int response_timeout_ms, std::string & error);
  void disconnect();
  bool connected() const;
  bool write_register(uint16_t address, uint16_t value, std::string & error);
  bool write_registers(uint16_t address, const uint16_t * values, int count, std::string & error);
  bool read_registers(uint16_t address, uint16_t * values, int count, std::string & error);

private:
  _modbus * context_{nullptr};
};

}  // namespace zlac8015d_driver
