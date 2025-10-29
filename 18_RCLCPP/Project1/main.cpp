#include "rclcpp/rclcpp.hpp"        // ros2의 c++ 라이브러리
#include "std_msgs/msg/int32.hpp"   // Int32 타입 메시지 사용
#include <memory>                   // 스마트 포인터를 사용하기 위한 헤더파일
#include <chrono>                   // 시간 관련 헤더파일

// main 함수
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // ROS2 시스템 초기화 함수
    auto node = std::make_shared<rclcpp::Node>("node_pub1");    // 노드 객체 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));       // qos 객체 생성
    // publisher 생성
    auto mypub = node->create_publisher<std_msgs::msg::Int32 >("topic_pub1",qos_profile );
    std_msgs::msg::Int32  message;  // int32 메시지 객체 생성
    message.data = 0;   // 0으로 초기화
    rclcpp::WallRate loop_rate(1.0); //반복주파수를 저장하는 객체(단위 Hz)
    // 무한루프 -> Ctrl+c 입력 받으면 종료
    while(rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data);  //로그 출력
        mypub->publish(message);   // 토픽 발행
        message.data+=1;   // 데이터 값 1씩 증가
        //rclcpp::spin_some(node);
        loop_rate.sleep(); //반복주파수에서 남은 시간 만큼 sleep
    }
    rclcpp::shutdown();  // ros2 종료
    return 0;
}



