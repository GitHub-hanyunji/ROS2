#include "rclcpp/rclcpp.hpp"        // ros2의 c++ 라이브러리
#include "geometry_msgs/msg/vector3.hpp"   // geometry_msgs 패키지의 Vector3 메시지 타입
#include <iostream>                 // 표준 입출력 라이브러리
#include <memory>                   // 스마트 포인터를 사용하기 위한 헤더파일
#include <chrono>                   // 시간 관련 헤더파일
#include <functional>

using namespace std::chrono_literals;  // 시간 리터럴

// callback 함수
void callback(rclcpp::Node::SharedPtr node, rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mypub,
    geometry_msgs::msg::Vector3 message)
{
    // 로그 출력
    RCLCPP_INFO(node->get_logger(), "Publish: %f %f %f", message.x,message.y,message.z);
    mypub->publish(message);  // 토픽 발행
}
// main 함수
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // ROS2 시스템 초기화 함수
    auto node = std::make_shared<rclcpp::Node>("node_pub2");    // 노드 객체 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));       // qos 객체 생성
    // publisher 생성
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub2",qos_profile );
    geometry_msgs::msg::Vector3 message;    // Vector3 메시지 객체 생성
    // Vector3의 x,y,z 사용자로부터 입력 받기
    std::cin>>message.x>>message.y>>message.z;
    // bind 함수로 callback 함수를 매개변수 없는 void()함수형태로 만들기
    std::function<void()>fn=std::bind(callback,node,mypub,message);
    auto timer=node->create_wall_timer(1s,fn);  // 1초마다 callback 함수 호출
    rclcpp::spin(node);  // 프로그램 계속 실행
    rclcpp::shutdown();   // ros2 종료
    return 0;  
}
