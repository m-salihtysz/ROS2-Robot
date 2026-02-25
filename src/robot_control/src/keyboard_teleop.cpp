#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop() : Node("keyboard_teleop")
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    linear_speed_ = 0.5;
    angular_speed_ = 1.0;
    
    // Mevcut komutu başlat
    current_cmd_.linear.x = 0.0;
    current_cmd_.linear.y = 0.0;
    current_cmd_.linear.z = 0.0;
    current_cmd_.angular.x = 0.0;
    current_cmd_.angular.y = 0.0;
    current_cmd_.angular.z = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "Keyboard Teleop başlatıldı");
    RCLCPP_INFO(this->get_logger(), "Kontroller:");
    RCLCPP_INFO(this->get_logger(), "  W/A/S/D - Hareket");
    RCLCPP_INFO(this->get_logger(), "  Q - Çıkış");
    
    // Terminal ayarları
    tcgetattr(STDIN_FILENO, &old_terminal_);
    new_terminal_ = old_terminal_;
    new_terminal_.c_lflag &= ~(ICANON | ECHO);
    new_terminal_.c_cc[VMIN] = 0;
    new_terminal_.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal_);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&KeyboardTeleop::timer_callback, this));
  }
  
  ~KeyboardTeleop()
  {
    // Terminal ayarlarını geri yükle
    tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
    
    // Robotu durdur
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }
  
private:
  void timer_callback()
  {
    char key;
    bool key_pressed = false;
    
    // Tüm tuşları oku (buffer'da kalan tuşları temizle)
    while (read(STDIN_FILENO, &key, 1) > 0)
    {
      key_pressed = true;
      switch (key)
      {
        case 'w':
        case 'W':
          current_cmd_.linear.x = linear_speed_;
          current_cmd_.angular.z = 0.0;
          RCLCPP_INFO(this->get_logger(), "İleri");
          break;
        case 's':
        case 'S':
          current_cmd_.linear.x = -linear_speed_;
          current_cmd_.angular.z = 0.0;
          RCLCPP_INFO(this->get_logger(), "Geri");
          break;
        case 'a':
        case 'A':
          current_cmd_.angular.z = angular_speed_;
          current_cmd_.linear.x = 0.0;
          RCLCPP_INFO(this->get_logger(), "Sola dön");
          break;
        case 'd':
        case 'D':
          current_cmd_.angular.z = -angular_speed_;
          current_cmd_.linear.x = 0.0;
          RCLCPP_INFO(this->get_logger(), "Sağa dön");
          break;
        case 'q':
        case 'Q':
          RCLCPP_INFO(this->get_logger(), "Çıkılıyor...");
          // Robotu durdur
          current_cmd_.linear.x = 0.0;
          current_cmd_.angular.z = 0.0;
          cmd_vel_pub_->publish(current_cmd_);
          rclcpp::shutdown();
          return;
        case ' ':  // Space tuşu - dur
          current_cmd_.linear.x = 0.0;
          current_cmd_.angular.z = 0.0;
          RCLCPP_INFO(this->get_logger(), "Dur");
          break;
        default:
          break;
      }
    }
    
    // Komutu sürekli yayınla (tuş basılmadığında da önceki komut korunur)
    cmd_vel_pub_->publish(current_cmd_);
  }
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist current_cmd_;
  double linear_speed_;
  double angular_speed_;
  struct termios old_terminal_, new_terminal_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}
