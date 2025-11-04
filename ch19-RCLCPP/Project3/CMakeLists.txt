cmake_minimum_required(VERSION 3.5)    # CMake 최소 버전 지정
project(pub3)                          # 프로젝트 이름 설정

# Default to C++14 -> C++ 표준 버전 지정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 컴파일러 경고 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)     # 빌드 시스템 패키지
find_package(rclcpp REQUIRED)          # ros2의 C++ 클라이언트 라이브러리
find_package(geometry_msgs REQUIRED)   # geometry_msgs 패키지 -> 메시지 인터페이스

#build
add_executable(pub3 src/main.cpp)   # 실행파일 생성
#pub2 실행파일이 rclcpp와 geometry_msgs를 사용할 수 있도록 링크
ament_target_dependencies(pub3 rclcpp geometry_msgs)

# Install  -> 빌드 결과물 설치 경로 지정
install(TARGETS
  pub3
  DESTINATION lib/${PROJECT_NAME})

ament_package()
