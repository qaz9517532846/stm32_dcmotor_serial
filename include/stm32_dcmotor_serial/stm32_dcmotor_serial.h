#include <ros/ros.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <stm32_dcmotor_serial/encoder.h>
#include <stm32_dcmotor_serial/dutycontrol.h>

using namespace LibSerial;

class STM32_DCMotor
{
public:
	STM32_DCMotor(std::string name);
	~STM32_DCMotor();

	void spin();

private:
    ros::NodeHandle nh_;

    ros::ServiceServer duty_motor;
    ros::Time current_time_, last_time_;

    SerialPort serial_port_;
    std::string serial_port_name_;
    int parity_;
    int flowcontrol_;
    int baund_rate_;
    int character_size_;
    int stop_bits_;
    int frequency_;

    int pulse_resoution_;
    double move_resolution_ = 0;
    double last_move_resolution_ = 0;
    double motor_pos_;

    std::string serial_read;

    std::string read_string;

    void serial_baund_rate(int baund_rate);
    void serial_character_size(int character_size);
    void serial_stop_bits(int stop_bits);
    void serial_parity(int parity);
    void serial_flowcontrol(int flowcontrol);

    bool stm32_dutycontrol(stm32_dcmotor_serial::dutycontrol::Request  &req,
                           stm32_dcmotor_serial::dutycontrol::Response &res);

    int encoder_data_process(std::string data);
    int charToint_Number(char data);
};