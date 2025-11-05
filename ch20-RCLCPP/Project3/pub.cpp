#include "rclcpp/rclcpp.hpp"        // ros2의 c++ 라이브러리
#include "std_msgs/msg/string.hpp"   // string 타입 메시지 사용
#include "rclcpp/time_source.hpp"
#include <memory>                   // 스마트 포인터를 사용하기 위한 헤더파일
#include <chrono>                   // 시간 관련 헤더파일
#include <functional>               // 함수 헤더파일

using namespace std::chrono_literals;  // 시간 리터럴

// callback 함수
void callback(rclcpp::Node::SharedPtr node, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mypub,
    std_msgs::msg::String message)
{
    RCLCPP_INFO(node->get_logger(), "Publish: %s", message.data.c_str());  //로그 출력
    mypub->publish(message);   // 토픽 발행
}

// main 함수
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // ROS2 시스템 초기화 함수
    auto node = std::make_shared<rclcpp::Node>("node_pub1");    // 노드 객체 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));       // qos 객체 생성
    // publisher 생성
    auto mypub = node->create_publisher<std_msgs::msg::String >("topic_pub1",qos_profile);
    std_msgs::msg::String  message;  // int32 메시지 객체 생성
    message.data = "Hello world!";   // 메시지 초기화
    // create_wall_timer 함수는 void 즉, 매개변수 없는 함수만 받을 수 있음
    // 그래서 bind 함수를 이용해 매개변수가 있는 callback 함수를 void 형태로 바꿔줘야함
    std::function<void()>fn=std::bind(callback,node,mypub,message);
    auto timer=node->create_wall_timer(1s,fn);  // 1초마다 callback 함수 호출
    rclcpp::spin(node);  // 프로그램 계속 실행
    rclcpp::shutdown();  // ros2 종료
    return 0;
}