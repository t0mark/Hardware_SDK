cmake_minimum_required(VERSION 3.8)
project(service_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# rosidl_generate_interfaces를 사용하여 서비스와 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Getcurrentact.srv"
  "srv/Geterror.srv"
  "srv/Getposset.srv"
  "srv/Getposact.srv"
  "srv/Getspeedset.srv"
  "srv/Getforceact.srv"
  "srv/Getforceset.srv"
  "srv/Getangleset.srv"
  "srv/Getangleact.srv"
  "srv/Gettemp.srv"
  "srv/Getstatus.srv"
  
  "srv/Setid.srv"
  "srv/Setreduratio.srv"
  "srv/Setpos.srv"
  "srv/Setclearerror.srv"
  "srv/Setresetpara.srv"
  "srv/Setdefaultspeed.srv"
  "srv/Setspeed.srv"
  "srv/Setangle.srv"
  "srv/Setsaveflash.srv"
  "srv/Setforce.srv"
  "srv/Setdefaultforce.srv"
  "srv/Setforceclb.srv"
  "srv/Setcurrentlimit.srv"
  "srv/Setgestureno.srv"
  
  "msg/GetAngleAct1.msg"
  "msg/GetForceAct1.msg"
  "msg/GetTouchAct1.msg"
  "msg/GetTemp1.msg"
  "msg/SetAngle1.msg"
  "msg/SetForce1.msg"
  "msg/SetSpeed1.msg"
  DEPENDENCIES std_msgs
)

# 테스트 및 코드 검사
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

