#include "rclcpp/rclcpp.hpp"        // ros2의 c++ 라이브러리
#include "std_msgs/msg/string.hpp"   // Int32 타입 메시지 사용
#include <memory>                   // 스마트 포인터를 사용하기 위한 헤더파일
#include <chrono>                   // 시간 관련 헤더파일
#include <functional>               // 함수 헤더파일

using namespace std::chrono_literals;  // 시간 리터럴

// callback 함수
void mysub_callback(rclcpp::Node::SharedPtr node, const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Received message: %s", msg->data.c_str());  //로그 출력
}

// main 함수
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // ROS2 시스템 초기화 함수
    auto node = std::make_shared<rclcpp::Node>("node_sub1");    // 노드 객체 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));       // qos 객체 생성

    // create_subscription 함수는 반환형 void, MessageT만 받을 수 있음
    // 그래서 bind 함수를 이용해 매개변수가 있는 callback 함수를 올바른 형태로 바꿔줘야함
    std::function<void(const std_msgs::msg::String::SharedPtr)>fn=std::bind(mysub_callback,node,
    std::placeholders::_1);
    auto timer=node->create_subscription<std_msgs::msg::String>("topic_pub1",qos_profile,fn);
    rclcpp::spin(node);  // 프로그램 계속 실행
    rclcpp::shutdown();  // ros2 종료
    return 0;
}