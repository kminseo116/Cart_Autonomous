// 모터 통신코드

#include "zlac8015d_driver/modbus_interface.hpp"

#include <cerrno>
#include <cstring>

#include <modbus/modbus.h>

namespace zlac8015d_driver
{

ModbusInterface::~ModbusInterface()
{
  disconnect();
}

bool ModbusInterface::connect(const std::string & serial_port, const int baudrate, const char parity,
  const int data_bits, const int stop_bits, const int driver_id, const int response_timeout_ms,
  std::string & error)
{
  disconnect();
  context_ = modbus_new_rtu(serial_port.c_str(), baudrate, parity, data_bits, stop_bits);
  if (context_ == nullptr) {
    error = "modbus_new_rtu 실패";
    return false;
  }
  if (modbus_set_slave(context_, driver_id) == -1) {
    error = std::string("Modbus slave ID 설정 실패: ") + modbus_strerror(errno);
    disconnect();
    return false;
  }
  const uint32_t timeout_us = static_cast<uint32_t>(response_timeout_ms) * 1000U;
  if (modbus_set_response_timeout(context_, timeout_us / 1000000U, timeout_us % 1000000U) == -1 ||
    modbus_connect(context_) == -1)
  {
    error = std::string("RS485 연결 실패: ") + modbus_strerror(errno);
    disconnect();
    return false;
  }
  return true;
}

void ModbusInterface::disconnect()
{
  if (context_ != nullptr) {
    modbus_close(context_);
    modbus_free(context_);
    context_ = nullptr;
  }
}

bool ModbusInterface::connected() const
{
  return context_ != nullptr;
}

bool ModbusInterface::write_register(const uint16_t address, const uint16_t value, std::string & error)
{
  if (context_ == nullptr || modbus_write_register(context_, address, value) == -1) {
    error = std::string("Modbus 단일 레지스터 쓰기 실패: ") + modbus_strerror(errno);
    return false;
  }
  return true;
}

bool ModbusInterface::write_registers(const uint16_t address, const uint16_t * values, const int count,
  std::string & error)
{
  if (context_ == nullptr || modbus_write_registers(context_, address, count, values) == -1) {
    error = std::string("Modbus 다중 레지스터 쓰기 실패: ") + modbus_strerror(errno);
    return false;
  }
  return true;
}

bool ModbusInterface::read_registers(const uint16_t address, uint16_t * values, const int count,
  std::string & error)
{
  if (context_ == nullptr || modbus_read_registers(context_, address, count, values) == -1) {
    error = std::string("Modbus 레지스터 읽기 실패: ") + modbus_strerror(errno);
    return false;
  }
  return true;
}

}  // namespace zlac8015d_driver
