#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "gimbal_bridge/sp_protocol.hpp"

namespace gimbal_bridge
{

class GimbalBridge : public rclcpp::Node
{
public:
  GimbalBridge()
  : Node("gimbal_bridge")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);
    tx_rate_ = declare_parameter<double>("tx_rate_hz", 100.0);
    default_mode_ = declare_parameter<int>("default_mode", 0);
    print_tx_ = declare_parameter<bool>("print_tx", false);
    mode_ = static_cast<uint8_t>(default_mode_);

    if (!openSerial()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s @ %d. Node stays up but will not talk.",
                   port_.c_str(), baud_);
    } else {
      RCLCPP_INFO(get_logger(), "Opened %s @ %d (frame %zu/%zu bytes)",
                  port_.c_str(), baud_, sizeof(VisionToGimbal), sizeof(GimbalToVision));
    }

    // chassis velocity: linear.x/y -> vx/vy, angular.z -> vw
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        vx_ = static_cast<float>(msg->linear.x);
        vy_ = static_cast<float>(msg->linear.y);
        vw_ = static_cast<float>(msg->angular.z);
      });

    // gimbal aim: x -> yaw, y -> pitch, z -> mode
    sub_aim_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/gimbal_aim", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        yaw_ = static_cast<float>(msg->x);
        pitch_ = static_cast<float>(msg->y);
        mode_ = static_cast<uint8_t>(msg->z);
      });

    // lifter target: 0 top, 1 bottom, 2 middle, 255 no change
    sub_lifter_ = create_subscription<std_msgs::msg::UInt8>(
      "/lifter_cmd", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        lifter_cmd_ = msg->data;
      });

    // tunnel heading alignment: x -> target angle (deg), y -> enable
    sub_align_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/tunnel_align", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        align_angle_deg_ = static_cast<float>(msg->x);
        align_active_ = (msg->y != 0.0) ? 1U : 0U;
      });

    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      "/auto_aim_enable", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        enable_ = msg->data;
      });

    pub_state_ = create_publisher<sensor_msgs::msg::Imu>("/gimbal_state", 10);
    pub_rate_ = create_publisher<std_msgs::msg::Float32>("/gimbal_rx_hz", 10);
    // [lifter_current_pos, is_transformable, transform_state_centi, tunnel_yaw_aligned]
    pub_lifter_ = create_publisher<std_msgs::msg::Int32MultiArray>("/lifter_state", 10);

    if (fd_ >= 0) {
      read_thread_ = std::thread(&GimbalBridge::readLoop, this);
      period_ms_ = static_cast<int>(1000.0 / std::max(1.0, tx_rate_));
      tx_thread_ = std::thread(&GimbalBridge::txLoop, this);
    }
  }

  ~GimbalBridge() override
  {
    quit_ = true;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (tx_thread_.joinable()) {
      tx_thread_.join();
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  bool openSerial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      return false;
    }
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    speed_t sp = B115200;
    switch (baud_) {
      case 9600: sp = B9600; break;
      case 57600: sp = B57600; break;
      case 115200: sp = B115200; break;
      case 230400: sp = B230400; break;
      case 460800: sp = B460800; break;
      case 921600: sp = B921600; break;
      default:
        RCLCPP_WARN(get_logger(), "Unsupported baud %d, falling back to 115200", baud_);
        sp = B115200;
        break;
    }
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void readLoop()
  {
    std::vector<uint8_t> acc;
    acc.reserve(4096);
    uint8_t chunk[1024];
    auto last_stat = std::chrono::steady_clock::now();
    int frames_since = 0;

    while (!quit_) {
      const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
      if (n <= 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      } else {
        acc.insert(acc.end(), chunk, chunk + n);
      }

      std::size_t i = 0;
      while (i + sizeof(GimbalToVision) <= acc.size()) {
        if (acc[i] != 'S' || acc[i + 1] != 'P') {
          ++i;
          continue;
        }
        GimbalToVision frame;
        std::memcpy(&frame, acc.data() + i, sizeof(frame));
        if (checkRx(frame)) {
          publishState(frame);
          ++frames_since;
          i += sizeof(frame);
        } else {
          ++i;
        }
      }
      if (i > 0) {
        acc.erase(acc.begin(), acc.begin() + static_cast<long>(i));
      }
      if (acc.size() > 8192) {
        acc.clear();
      }

      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - last_stat).count();
      if (dt >= 1.0) {
        std_msgs::msg::Float32 m;
        m.data = static_cast<float>(frames_since / dt);
        pub_rate_->publish(m);
        frames_since = 0;
        last_stat = now;
      }
    }
  }

  void publishState(const GimbalToVision & f)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = "gimbal";
    imu.orientation.w = f.q[0];
    imu.orientation.x = f.q[1];
    imu.orientation.y = f.q[2];
    imu.orientation.z = f.q[3];
    imu.angular_velocity.x = f.yaw_vel;
    imu.angular_velocity.y = f.pitch_vel;
    imu.angular_velocity.z = static_cast<double>(f.mode);
    pub_state_->publish(imu);

    std_msgs::msg::Int32MultiArray lift;
    lift.data.resize(4);
    lift.data[0] = static_cast<int32_t>(f.lifter_current_pos);
    lift.data[1] = static_cast<int32_t>(f.is_transformable);
    lift.data[2] = static_cast<int32_t>(f.transform_state * 100.0F);
    lift.data[3] = static_cast<int32_t>(f.tunnel_yaw_aligned);
    pub_lifter_->publish(lift);
  }

  void txLoop()
  {
    int logged = 0;
    while (!quit_) {
      VisionToGimbal tx;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        tx.mode = enable_ ? mode_ : 0;
        tx.yaw = yaw_;
        tx.pitch = pitch_;
        tx.vx = vx_;
        tx.vy = vy_;
        tx.vw = vw_;
        tx.desire_lifter_pos = lifter_cmd_;
        tx.tunnel_align_active = align_active_;
        tx.tunnel_align_angle_deg = align_angle_deg_;
      }
      finalizeTx(tx);
      const ssize_t w = ::write(fd_, &tx, sizeof(tx));
      if (w != static_cast<ssize_t>(sizeof(tx))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "serial write short: %zd/%zu", w, sizeof(tx));
      }
      if (print_tx_ && logged < 5) {
        RCLCPP_INFO(get_logger(),
                    "tx mode=%u yaw=%.3f pitch=%.3f vx=%.3f vy=%.3f vw=%.3f "
                    "lifter=%u align=%u/%.1fdeg crc=0x%04X",
                    tx.mode, tx.yaw, tx.pitch, tx.vx, tx.vy, tx.vw,
                    tx.desire_lifter_pos, tx.tunnel_align_active,
                    tx.tunnel_align_angle_deg, tx.crc16);
        ++logged;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(period_ms_));
    }
    // leave a real board in a safe state
    VisionToGimbal tx;
    tx.mode = 0;
    tx.desire_lifter_pos = LIFTER_NO_CHANGE;
    finalizeTx(tx);
    if (::write(fd_, &tx, sizeof(tx)) < 0) {
      // best effort on shutdown
    }
  }

  std::string port_;
  int baud_ = 115200;
  double tx_rate_ = 100.0;
  int default_mode_ = 0;
  bool print_tx_ = false;
  int period_ms_ = 10;
  int fd_ = -1;
  std::atomic<bool> quit_{false};
  std::thread read_thread_;
  std::thread tx_thread_;

  std::mutex mtx_;
  float vx_ = 0.0F;
  float vy_ = 0.0F;
  float vw_ = 0.0F;
  float yaw_ = 0.0F;
  float pitch_ = 0.0F;
  uint8_t mode_ = 0;
  uint8_t lifter_cmd_ = LIFTER_NO_CHANGE;
  uint8_t align_active_ = 0;
  float align_angle_deg_ = 0.0F;
  bool enable_ = true;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_aim_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_align_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_lifter_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_rate_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_lifter_;
};

}  // namespace gimbal_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gimbal_bridge::GimbalBridge>());
  rclcpp::shutdown();
  return 0;
}
