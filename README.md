# turtle_control
对小海龟订阅pose实现闭环控制
---
1. **打开海龟模拟器**  
  在Ubuntu桌面Ctrl+Alt+T打开终端输入`ros2 run turtlesim turtlesim_node`,出现小海龟模拟器。  
2. **创建turtle_control.cpp的文件**  
  在src目录下创建turtle_control.cpp的文件，然后在其中书写代码，如下所示：
```C++
#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"chrono"
#include"turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class TurtleControlNode:public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  // 发布者的共享智能指针
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;   // 订阅者的共享智能指针

    double target_x_{1.0};
    double target_y_{1.0};
    double k_{1.0};     // 比例系数
    double max_speed_{3.0};  // 最大速度

public:
    explicit TurtleControlNode(const std::string& node_name):Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msga::msg::Twist>("turtle1/cmd_vel",10)
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose",10,std::bind(&
        TurtleControlNode::on_pose_received,this,std::placeholders::_1))

    }

    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose)   // 参数收到共享指针
    {
        // 1.获取当前位置
        auto current_x = pose->x;
        auto current_y = pose->y;
        RCLCPP_INFO(get_logger(), "当前:x=%f, y=%f", current_x, current_y);

        // 2.计算当前海龟的位置与目标位置之间的距离之差和角度之差
        auto distance = std::sqrt(
            (target_x_ - current_x) * (target_x_ - current_x) + 
            (target_y_ - current_y) * (target_y_ - current_y)
        );
        auto angle = std::atan2((target_y_ - current_y) , (target_x_ - current_x))- pose -> theta;

        // 3.控制策略
        auto msg = geometry_msgs::msg::Twist();
        if(distance > 0.1)
        {
            if(fabs(angle) > 0.2)
            {
                msg.angular.z = fabs(angle);
            }
            else
            {
                msg.angular.x = k_ * distance;
            }
        }

        // 4.限制最大速度值
        if(msg.linear.x > max_speed_)
        {
            msg.linear.y = max_speed_;
        }

        publisher_->publish(msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init::(argc, argv)
    auto node = std::make_shared<TurtleControlNode>("turtle_control")
    rclcpp::spin(node);
    rclcpp::shutdown();
}

```
3. **添加可执行文件和依赖**  
    在CmakeList.txt文件中添加`add_executable(turtle_control src/turtle_control.cpp)`,  
    `ament_cmake(turtle_control rclcpp geometry_msgs turtlesim)`,  
    `install(TARGETS turtle_control DESTINATION lib/${PROJECT_NAME})`  
4. **运行**  
   （1）在回到工作空间目录打开终端，输入`colcon build`构造可执行文件  
   （2）利用`source install/setup.bash`添加环境变量  
   （3）利用命令`ros2 run demo_cpp_topic turtle_control`运行  
   最后，打开海龟模拟器，就可以看见小海龟到达指定位置。  
