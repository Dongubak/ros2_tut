#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node {
public:
   MyNode(): Node("cpp_test") {
      RCLCPP_INFO(this->get_logger(), "Hello Cpp class Node");

      timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                       std::bind(&MyNode::timerCallback, this));
   }
private:
   void timerCallback() {
      RCLCPP_INFO(this->get_logger(), "Hello");
   }
   rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
   rclcpp::init(argc, argv); /// 로스 커뮤니케이션 초기화

   auto node = std::make_shared<MyNode>();
   rclcpp::spin(node); /// 노드를 스핀함 (기다림) 노드가 살이있게 유지해준다.
   rclcpp::shutdown(); /// 노드가 멈추면 로스 커뮤니케이션 종료

   return 0;
}