cmake_minimum_required(VERSION 3.0.2)
project(stm32_dcmotor_serial)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(LibSerial)

add_message_files(
  FILES
  encoder.msg
)

add_service_files(
  FILES
  dutycontrol.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stm32_dcmotor_serial
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LibSerial_INCLUDE_DIRS}
)

add_executable(STM32_DCMotor src/stm32_dcmotor_serial_main.cpp src/stm32_dcmotor_serial.cpp)
target_link_libraries(STM32_DCMotor ${catkin_LIBRARIES} serial)