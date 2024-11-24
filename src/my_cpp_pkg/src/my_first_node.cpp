#include "rclcpp/rclcpp.hpp"

using namespace rclcpp;
using namespace std;
class MyNode: public Node {
public:
   MyNode() : Node("cpp_node"), counter(0) {
      RCLCPP_INFO(this->get_logger(), "hello cpp class node");

      timer_ = this->create_wall_timer(chrono::seconds(1),
         bind(&MyNode::timerCallback, this));
   }

   private:
   void timerCallback() {
      counter++;
      RCLCPP_INFO(this->get_logger(), "hello %d", counter);
   }
   TimerBase::SharedPtr timer_;
   int counter;
};

int main(int argc, char** argv) {
   init(argc, argv);

   auto node = std::make_shared<MyNode>(); // 노드 생성
   spin(node);
   shutdown();

   return 0;
}