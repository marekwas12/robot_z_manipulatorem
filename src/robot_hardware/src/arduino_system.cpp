#include "robot_hardware/arduino_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sstream>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace robot_hardware
{


// =====================================================
// INIT
// =====================================================

hardware_interface::CallbackReturn ArduinoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  // ===================================================
  // LICZBA JOINTOW
  // ===================================================

  if (info_.joints.size() != JOINT_COUNT)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Oczekiwano 5 jointow, otrzymano %zu",
      info_.joints.size());

    return hardware_interface::CallbackReturn::ERROR;
  }


  // ===================================================
  // PARAMETRY SERIAL
  // ===================================================

  if (info_.hardware_parameters.count("serial_port"))
  {
    serial_port_ =
      info_.hardware_parameters.at("serial_port");
  }


  if (info_.hardware_parameters.count("baud_rate"))
  {
    baud_rate_ =
      std::stoi(
        info_.hardware_parameters.at("baud_rate")
      );
  }


  // ===================================================
  // BARDZO WAZNE
  //
  // Nie ustawiamy tutaj komend na 0.
  //
  // Zero jest prawdziwa pozycja i mogloby spowodowac
  // ruch robota natychmiast po aktywacji kontrolera.
  // ===================================================

  const double nan =
    std::numeric_limits<double>::quiet_NaN();


  hw_positions_.fill(nan);
  hw_commands_.fill(nan);
  last_commands_.fill(nan);


  homed_ = false;
  homing_requested_ = false;
  have_valid_position_ = false;
  first_write_ = true;


  RCLCPP_INFO(
    get_logger(),
    "ArduinoSystem zainicjalizowany");


  RCLCPP_INFO(
    get_logger(),
    "Serial: %s @ %d",
    serial_port_.c_str(),
    baud_rate_);


  return hardware_interface::CallbackReturn::SUCCESS;
}


// =====================================================
// OTWARCIE SERIAL
// =====================================================

bool ArduinoSystem::open_serial()
{
  serial_fd_ = open(
    serial_port_.c_str(),
    O_RDWR | O_NOCTTY | O_NONBLOCK
  );


  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Nie moge otworzyc %s: %s",
      serial_port_.c_str(),
      strerror(errno)
    );

    return false;
  }


  termios tty{};


  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "tcgetattr() failed: %s",
      strerror(errno)
    );

    close_serial();

    return false;
  }


  cfmakeraw(&tty);


  speed_t speed = B115200;


  if (baud_rate_ != 115200)
  {
    RCLCPP_WARN(
      get_logger(),
      "Ten driver aktualnie obsluguje 115200 baud. "
      "Uzywam B115200."
    );
  }


  cfsetispeed(
    &tty,
    speed
  );

  cfsetospeed(
    &tty,
    speed
  );


  tty.c_cflag |= CLOCAL | CREAD;

  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;

  tty.c_cflag |= CS8;


  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;


  if (
    tcsetattr(
      serial_fd_,
      TCSANOW,
      &tty
    ) != 0
  )
  {
    RCLCPP_ERROR(
      get_logger(),
      "tcsetattr() failed: %s",
      strerror(errno)
    );

    close_serial();

    return false;
  }


  tcflush(
    serial_fd_,
    TCIOFLUSH
  );


  RCLCPP_INFO(
    get_logger(),
    "Serial otwarty: %s",
    serial_port_.c_str()
  );


  return true;
}


// =====================================================
// ZAMKNIECIE SERIAL
// =====================================================

void ArduinoSystem::close_serial()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);

    serial_fd_ = -1;
  }
}


// =====================================================
// CONFIGURE
// =====================================================

hardware_interface::CallbackReturn ArduinoSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  // Arduino Mega resetuje sie po otwarciu portu.
  usleep(2000000);


  homed_ = false;

  homing_requested_ = false;

  have_valid_position_ = false;

  first_write_ = true;


  rx_buffer_.clear();


  const double nan =
    std::numeric_limits<double>::quiet_NaN();


  hw_positions_.fill(nan);

  hw_commands_.fill(nan);

  last_commands_.fill(nan);


  RCLCPP_INFO(
    get_logger(),
    "ArduinoSystem skonfigurowany"
  );


  return hardware_interface::CallbackReturn::SUCCESS;
}


// =====================================================
// ACTIVATE
// =====================================================

hardware_interface::CallbackReturn ArduinoSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(),
    "Aktywacja ArduinoSystem"
  );


  RCLCPP_INFO(
    get_logger(),
    "Ruch zablokowany do zakonczenia homingu "
    "i odebrania poprawnego @P"
  );


  return hardware_interface::CallbackReturn::SUCCESS;
}


// =====================================================
// DEACTIVATE
// =====================================================

hardware_interface::CallbackReturn ArduinoSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    get_logger(),
    "Dezaktywacja ArduinoSystem"
  );


  homed_ = false;

  have_valid_position_ = false;


  close_serial();


  return hardware_interface::CallbackReturn::SUCCESS;
}


// =====================================================
// STATE INTERFACES
// =====================================================

std::vector<hardware_interface::StateInterface>
ArduinoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface>
    interfaces;


  for (
    std::size_t i = 0;
    i < JOINT_COUNT;
    ++i
  )
  {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]
    );
  }


  return interfaces;
}


// =====================================================
// COMMAND INTERFACES
// =====================================================

std::vector<hardware_interface::CommandInterface>
ArduinoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface>
    interfaces;


  for (
    std::size_t i = 0;
    i < JOINT_COUNT;
    ++i
  )
  {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]
    );
  }


  return interfaces;
}


// =====================================================
// SERIAL TX
// =====================================================

bool ArduinoSystem::send_line(
  const std::string & line)
{
  if (serial_fd_ < 0)
  {
    return false;
  }


  const ssize_t result =
    ::write(
      serial_fd_,
      line.data(),
      line.size()
    );


  if (result < 0)
  {
    if (
      errno != EAGAIN &&
      errno != EWOULDBLOCK
    )
    {
      RCLCPP_ERROR(
        get_logger(),
        "Serial write error: %s",
        strerror(errno)
      );
    }


    return false;
  }


  return
    static_cast<std::size_t>(result) ==
    line.size();
}


// =====================================================
// PARSOWANIE @P
//
// Arduino:
// @P:-1000,1000,1000,180,90
// =====================================================

bool ArduinoSystem::parse_position_report(
  const std::string & line)
{
  if (
    line.rfind(
      "@P:",
      0
    ) != 0
  )
  {
    return false;
  }


  std::string payload =
    line.substr(3);


  std::replace(
    payload.begin(),
    payload.end(),
    ',',
    ' '
  );


  std::stringstream ss(
    payload
  );


  long j1_steps = 0;
  long j2_steps = 0;
  long j3_steps = 0;

  double j4_deg = 0.0;
  double j5_deg = 0.0;


  if (
    !(
      ss >>
      j1_steps >>
      j2_steps >>
      j3_steps >>
      j4_deg >>
      j5_deg
    )
  )
  {
    RCLCPP_WARN(
      get_logger(),
      "Niepoprawny @P: %s",
      line.c_str()
    );

    return false;
  }


  // ===================================================
  // KROKI -> RAD
  // ===================================================

  hw_positions_[0] =
    static_cast<double>(j1_steps) /
    STEPS_PER_RAD;


  hw_positions_[1] =
    static_cast<double>(j2_steps) /
    STEPS_PER_RAD;


  hw_positions_[2] =
    static_cast<double>(j3_steps) /
    STEPS_PER_RAD;


  // ===================================================
  // STOPNIE -> RAD
  // ===================================================

  hw_positions_[3] =
    j4_deg *
    M_PI /
    180.0;


  hw_positions_[4] =
    j5_deg *
    M_PI /
    180.0;


  // ===================================================
  // PIERWSZY POPRAWNY @P PO HOMINGU
  //
  // To jest najwazniejsza blokada.
  //
  // Command interface dostaje DOKLADNIE aktualna
  // fizyczna pozycje robota.
  // ===================================================

  if (
    homed_ &&
    !have_valid_position_
  )
  {
    hw_commands_ =
      hw_positions_;


    last_commands_ =
      hw_positions_;


    have_valid_position_ =
      true;


    first_write_ =
      false;


    RCLCPP_INFO(
      get_logger(),
      "Stan command interface zsynchronizowany "
      "z aktualna pozycja Arduino"
    );


    RCLCPP_INFO(
      get_logger(),
      "J1 %.6f rad",
      hw_positions_[0]
    );


    RCLCPP_INFO(
      get_logger(),
      "J2 %.6f rad",
      hw_positions_[1]
    );


    RCLCPP_INFO(
      get_logger(),
      "J3 %.6f rad",
      hw_positions_[2]
    );


    RCLCPP_INFO(
      get_logger(),
      "J4 %.6f rad",
      hw_positions_[3]
    );


    RCLCPP_INFO(
      get_logger(),
      "J5 %.6f rad",
      hw_positions_[4]
    );
  }


  return true;
}


// =====================================================
// SERIAL RX
// =====================================================

void ArduinoSystem::process_serial()
{
  if (serial_fd_ < 0)
  {
    return;
  }


  char buffer[256];


  while (true)
  {
    const ssize_t count =
      ::read(
        serial_fd_,
        buffer,
        sizeof(buffer)
      );


    if (count > 0)
    {
      rx_buffer_.append(
        buffer,
        static_cast<std::size_t>(count)
      );
    }

    else
    {
      break;
    }
  }


  std::size_t newline_pos;


  while (
    (
      newline_pos =
      rx_buffer_.find('\n')
    ) !=
    std::string::npos
  )
  {
    std::string line =
      rx_buffer_.substr(
        0,
        newline_pos
      );


    rx_buffer_.erase(
      0,
      newline_pos + 1
    );


    if (
      !line.empty() &&
      line.back() == '\r'
    )
    {
      line.pop_back();
    }


    if (line.empty())
    {
      continue;
    }


    // =================================================
    // @P
    // =================================================

    if (
      parse_position_report(
        line
      )
    )
    {
      continue;
    }


    // =================================================
    // ARDUINO GOTOWE
    // =================================================

    if (
      line.find(
        "Manipulator gotowy"
      ) !=
      std::string::npos
    )
    {
      if (!homing_requested_)
      {
        RCLCPP_INFO(
          get_logger(),
          "Arduino gotowe. "
          "Rozpoczynam homing."
        );


        if (
          send_line(
            "H\n"
          )
        )
        {
          homing_requested_ =
            true;
        }
      }


      continue;
    }


    // =================================================
    // HOMING GOTOWY
    // =================================================

    if (
      line.find(
        "HOMING GOTOWY"
      ) !=
      std::string::npos
    )
    {
      if (!homed_)
      {
        homed_ = true;


        /*
         * Bardzo wazne:
         *
         * Nie kopiujemy tutaj hw_positions_
         * do hw_commands_.
         *
         * Czekamy na PIERWSZY poprawny raport @P
         * wyslany juz po zakonczeniu homingu.
         */

        have_valid_position_ =
          false;


        first_write_ =
          true;


        RCLCPP_INFO(
          get_logger(),
          "=============================="
        );


        RCLCPP_INFO(
          get_logger(),
          "HOMING GOTOWY"
        );


        RCLCPP_INFO(
          get_logger(),
          "Czekam na pierwszy poprawny "
          "raport @P po homingu"
        );


        RCLCPP_INFO(
          get_logger(),
          "=============================="
        );
      }


      continue;
    }


    // =================================================
    // BLAD HOMINGU
    // =================================================

    if (
      line.find(
        "HOMING ERROR"
      ) !=
      std::string::npos ||

      line.find(
        "HOMING PRZERWANY"
      ) !=
      std::string::npos ||

      line.find(
        "\"error\":\"not_homed\""
      ) !=
      std::string::npos
    )
    {
      homed_ = false;

      homing_requested_ = false;

      have_valid_position_ = false;

      first_write_ = true;


      const double nan =
        std::numeric_limits<double>::quiet_NaN();


      hw_commands_.fill(nan);

      last_commands_.fill(nan);


      RCLCPP_ERROR(
        get_logger(),
        "Arduino utracilo stan homingu"
      );


      continue;
    }
  }
}


// =====================================================
// READ
// =====================================================

hardware_interface::return_type ArduinoSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  process_serial();


  return hardware_interface::return_type::OK;
}


// =====================================================
// WRITE
// =====================================================

hardware_interface::return_type ArduinoSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // ===================================================
  // BLOKADA 1:
  // HOMING
  // ===================================================

  if (!homed_)
  {
    return hardware_interface::return_type::OK;
  }


  // ===================================================
  // BLOKADA 2:
  // MUSIMY MIEC PIERWSZY POPRAWNY @P
  // ===================================================

  if (!have_valid_position_)
  {
    return hardware_interface::return_type::OK;
  }


  // ===================================================
  // BLOKADA 3:
  // ZADNA KOMENDA NIE MOZE BYC NaN / INF
  // ===================================================

  for (
    const double command :
    hw_commands_
  )
  {
    if (!std::isfinite(command))
    {
      return hardware_interface::return_type::OK;
    }
  }


  // ===================================================
  // J1-J3
  // RAD -> KROKI
  // ===================================================

  long j1_steps =
    std::lround(
      hw_commands_[0] *
      STEPS_PER_RAD
    );


  long j2_steps =
    std::lround(
      hw_commands_[1] *
      STEPS_PER_RAD
    );


  long j3_steps =
    std::lround(
      hw_commands_[2] *
      STEPS_PER_RAD
    );


  // ===================================================
  // J4-J5
  // RAD -> STOPNIE
  // ===================================================

  double j4_deg =
    hw_commands_[3] *
    180.0 /
    M_PI;


  double j5_deg =
    hw_commands_[4] *
    180.0 /
    M_PI;


  // ===================================================
  // TWARDY LIMIT J1
  // ===================================================

  j1_steps =
    std::clamp(
      j1_steps,
      -15557L,
      0L
    );


  // ===================================================
  // TWARDY LIMIT J2
  // ===================================================

  j2_steps =
    std::clamp(
      j2_steps,
      0L,
      10100L
    );


  // ===================================================
  // TWARDY LIMIT J3
  // ===================================================

  j3_steps =
    std::clamp(
      j3_steps,
      0L,
      11835L
    );


  // ===================================================
  // TWARDY LIMIT J4
  // ===================================================

  j4_deg =
    std::clamp(
      j4_deg,
      5.0,
      180.0
    );


  // ===================================================
  // TWARDY LIMIT J5
  // ===================================================

  j5_deg =
    std::clamp(
      j5_deg,
      0.0,
      180.0
    );


  // ===================================================
  // CZY KOMENDA SIE ZMIENILA
  // ===================================================

  bool changed =
    first_write_;


  if (!changed)
  {
    changed =

      std::abs(
        hw_commands_[0] -
        last_commands_[0]
      ) >
      (2.0 / STEPS_PER_RAD)

      ||

      std::abs(
        hw_commands_[1] -
        last_commands_[1]
      ) >
      (2.0 / STEPS_PER_RAD)

      ||

      std::abs(
        hw_commands_[2] -
        last_commands_[2]
      ) >
      (2.0 / STEPS_PER_RAD)

      ||

      std::abs(
        hw_commands_[3] -
        last_commands_[3]
      ) >
      (0.2 * M_PI / 180.0)

      ||

      std::abs(
        hw_commands_[4] -
        last_commands_[4]
      ) >
      (0.2 * M_PI / 180.0);
  }


  if (!changed)
  {
    return hardware_interface::return_type::OK;
  }


  // ===================================================
  // TWORZENIE @J
  // ===================================================

  std::ostringstream command;


  command
    << "@J:"
    << j1_steps
    << ","
    << j2_steps
    << ","
    << j3_steps
    << ","
    << j4_deg
    << ","
    << j5_deg
    << "\n";


  // ===================================================
  // SERIAL TX
  // ===================================================

  if (
    !send_line(
      command.str()
    )
  )
  {
    return hardware_interface::return_type::ERROR;
  }


  last_commands_ =
    hw_commands_;


  first_write_ =
    false;


  return hardware_interface::return_type::OK;
}


}  // namespace robot_hardware


PLUGINLIB_EXPORT_CLASS(
  robot_hardware::ArduinoSystem,
  hardware_interface::SystemInterface
)