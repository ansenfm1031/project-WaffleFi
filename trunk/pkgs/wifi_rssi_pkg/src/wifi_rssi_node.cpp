#include <chrono>
#include <string>
#include <array>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "wifi_interface/msg/wifi_rssi.hpp"

using namespace std::chrono_literals;

static constexpr const char* SERIAL_PORT = "/dev/serial0";
static constexpr int SERIAL_BAUD = 115200;
static constexpr const char* TOPIC_WIFI_RAW = "/wifi/raw";

static constexpr int TIMER_PERIOD_MS = 20;
static constexpr size_t READ_CHUNK = 4096;

static speed_t to_speed_t(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

class WifiRssiNode : public rclcpp::Node
{
public:
  WifiRssiNode()
  : Node("wifi_rssi_node"), serial_fd_(-1), in_frame_(false)
  {
    pub_ = create_publisher<wifi_interface::msg::WifiRssi>(
      TOPIC_WIFI_RAW, 10);

    open_serial();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(TIMER_PERIOD_MS),
      std::bind(&WifiRssiNode::tick, this)
    );

    RCLCPP_INFO(get_logger(), "wifi_rssi_node started");
  }

  ~WifiRssiNode() override
  {
    if (serial_fd_ >= 0)
      ::close(serial_fd_);
  }

private:
  void open_serial()
  {
    serial_fd_ = ::open(SERIAL_PORT, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(),
        "serial open failed: %s", std::strerror(errno));
      return;
    }

    struct termios tio{};
    tcgetattr(serial_fd_, &tio);
    cfmakeraw(&tio);

    speed_t spd = to_speed_t(SERIAL_BAUD);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    tcsetattr(serial_fd_, TCSANOW, &tio);
  }

  void tick()
  {
    std::array<char, READ_CHUNK> buf{};
    ssize_t n = ::read(serial_fd_, buf.data(), buf.size());

    if (n <= 0)
      return;

    process_stream(buf.data(), n);
  }

  // ===== 스트림 처리 핵심 =====
  void process_stream(const char* data, size_t len)
  {
    for (size_t i = 0; i < len; ++i) {
      char c = data[i];

      if (c == '\n') {
        handle_line(line_buf_);
        line_buf_.clear();
      }
      else if (c != '\r') {
        line_buf_ += c;
      }
    }
  }

  void handle_line(const std::string& line)
  {
    if (line.empty())
      return;

    // BEGIN
    if (line.find("BEGIN") != std::string::npos) {
      in_frame_ = true;
      RCLCPP_DEBUG(get_logger(), "BEGIN detected");
      return;
    }

    // END
    if (line.find("END") != std::string::npos) {
      in_frame_ = false;
      RCLCPP_DEBUG(get_logger(), "END detected");
      return;
    }

    if (!in_frame_)
      return;

    // 기대 포맷: SSID:xxx,RSSI:-45
    size_t ssid_pos = line.find("SSID:");
    size_t rssi_pos = line.find("RSSI:");

    if (ssid_pos == std::string::npos ||
        rssi_pos == std::string::npos)
      return;

    std::string ssid = line.substr(
      ssid_pos + 5,
      rssi_pos - (ssid_pos + 5)
    );

    int rssi = std::stoi(
      line.substr(rssi_pos + 5)
    );

    wifi_interface::msg::WifiRssi msg;
    msg.header.stamp = now();
    msg.header.frame_id = "wifi";
    msg.ssid = ssid;
    msg.rssi = rssi;

    pub_->publish(msg);
  }

private:
  int serial_fd_;
  bool in_frame_;
  std::string line_buf_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<wifi_interface::msg::WifiRssi>::SharedPtr pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WifiRssiNode>());
  rclcpp::shutdown();
  return 0;
}
