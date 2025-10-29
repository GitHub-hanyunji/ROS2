#include "rclcpp/rclcpp.hpp"        // ros2의 c++ 라이브러리
#include "geometry_msgs/msg/twist.hpp"  // geometry_msgs 패키지의 Twist 메시지 타입
#include <iostream>                 // 표준 입출력 라이브러리
#include <memory>                   // 스마트 포인터를 사용하기 위한 헤더파일
#include <chrono>                   // 시간 관련 헤더파일

// main 함수
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // ROS2 시스템 초기화 함수
    auto node = std::make_shared<rclcpp::Node>("node_pub3");    // 노드 객체 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));       // qos 객체 생성
    //publisher 생성 turtle1/cmd_vel 로 보냄
    auto mypub = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",qos_profile );
    geometry_msgs::msg::Twist message;

    char c;
    rclcpp::WallRate loop_rate(1.0); //반복주파수를 저장하는 객체(단위 Hz)  1초에 보내고 싶은 메시지 수
    while(rclcpp::ok())
    {
        std::cin>>c;   // 사용자로부터 문자 입력 받음
        // 'f' 입력시 거북이 속도 2로 앞으로 전진
        if(c=='f'){
            message.linear.x=2.0;
            message.linear.y=0.0;
            message.angular.z=0.0;
        }
        // 'b' 입력시 거북이 속도 2로 뒤로 후진
        else if(c=='b'){
            message.linear.x=-2.0;
            message.linear.y=0.0;
            message.angular.z=0.0;
        }
        // 'l' 입력시 거북이 속도 2로 좌회전
        else if(c=='l'){
            message.linear.x=0.0;
            message.linear.y=0.0;
            message.angular.z=2.0;
        }
        // 'r' 입력시 거북이 속도 2로 우회전
        else if(c=='r'){
            message.linear.x=0.0;
            message.linear.y=0.0;
            message.angular.z=-2.0;
        }
        // 로그 출력
        RCLCPP_INFO(node->get_logger(), "Publish linear: %f %f %f, angular: %f %f %f", message.linear.x,message.linear.y,message.linear.z,
                    message.angular.x,message.angular.y,message.angular.z);
        mypub->publish(message);  // 토픽 발행
        //rclcpp::spin_some(node);
        loop_rate.sleep(); //반복주파수에서 남은 시간 만큼 sleep
    }
    rclcpp::shutdown();  // ros2 종료
    return 0; 
}
