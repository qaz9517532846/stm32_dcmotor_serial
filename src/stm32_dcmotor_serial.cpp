#include <stm32_dcmotor_serial/stm32_dcmotor_serial.h>

#define SERIAL_SEND_LEN         (3)
#define SERIAL_REV_LEN          (2)

STM32_DCMotor::STM32_DCMotor(std::string name)
{
    ros::NodeHandle private_nh_("~");
    private_nh_.param<std::string>("Serial_Port", serial_port_name_, "/dev/ttyACM0");
    private_nh_.param<int>("Baund_rate", baund_rate_, 115200);
    private_nh_.param<int>("character_size", character_size_, 8);
    private_nh_.param<int>("stop_bits", stop_bits_, 1);
    private_nh_.param<int>("parity", parity_, 3);
    private_nh_.param<int>("flow_control", flowcontrol_, 0);
    private_nh_.param<int>("pulse_resoution", pulse_resoution_, 512);

    duty_motor = private_nh_.advertiseService("duty_control", &STM32_DCMotor::stm32_dutycontrol, this);
    encoder_motor = private_nh_.advertise<stm32_dcmotor_serial::encoder>("encoder", 1000);

    serial_port_.Open(serial_port_name_.c_str());
    if(!serial_port_.IsOpen())
    {
        ROS_INFO("Find not device: %s", serial_port_name_.c_str());
        return;
    }

    serial_baund_rate(baund_rate_);
    serial_character_size(character_size_);
    serial_stop_bits(stop_bits_);
    serial_parity(parity_);
    serial_flowcontrol(flowcontrol_);

    sendData.resize(SERIAL_SEND_LEN);
    revData.resize(SERIAL_REV_LEN);
}

STM32_DCMotor::~STM32_DCMotor()
{

}

void STM32_DCMotor::serial_baund_rate(int baund_rate)
{
    switch(baund_rate)
    { 
        case 300:
            serial_port_.SetBaudRate(BaudRate::BAUD_300);
            break;
        case 1200:
            serial_port_.SetBaudRate(BaudRate::BAUD_1200);
            break; 
        case 2400:
            serial_port_.SetBaudRate(BaudRate::BAUD_2400);
            break;
        case 9600:
            serial_port_.SetBaudRate(BaudRate::BAUD_9600);
            break; 
        case 19200: 
            serial_port_.SetBaudRate(BaudRate::BAUD_19200);
            break; 
        case 38400: 
            serial_port_.SetBaudRate(BaudRate::BAUD_38400);
            break;
        case 57600: 
            serial_port_.SetBaudRate(BaudRate::BAUD_57600);
            break; 
        case 115200: 
            serial_port_.SetBaudRate(BaudRate::BAUD_115200);
            break; 
        default: 
            serial_port_.SetBaudRate(BaudRate::BAUD_DEFAULT);
            break;
    }
}

void STM32_DCMotor::serial_character_size(int character_size)
{
    switch(character_size)
    { 
        case 5:
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_5);
            break;
        case 6:
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_6);
            break; 
        case 7:
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_7);
            break;
        case 8:
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            break; 
        default: 
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_DEFAULT);
            break;
    }
}

void STM32_DCMotor::serial_stop_bits(int stop_bits)
{
    switch(stop_bits)
    { 
        case 1:
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            break;
        case 2:
            serial_port_.SetStopBits(StopBits::STOP_BITS_2);
            break; 
        default: 
            serial_port_.SetStopBits(StopBits::STOP_BITS_DEFAULT);
            break;
    }
}

void STM32_DCMotor::serial_parity(int parity)
{
    switch(parity)
    { 
        case 1:
            serial_port_.SetParity(Parity::PARITY_EVEN);
            break;
        case 2:
            serial_port_.SetParity(Parity::PARITY_ODD);
            break; 
        case 3:
            serial_port_.SetParity(Parity::PARITY_NONE);
            break; 
        default: 
            serial_port_.SetParity(Parity::PARITY_DEFAULT);
            break;
    }
}

void STM32_DCMotor::serial_flowcontrol(int flowcontrol)
{
    switch(flowcontrol)
    { 
        case 0:
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            break; 
        case 1:
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
            break;
        default: 
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_DEFAULT);
            break;
    }
}

bool STM32_DCMotor::stm32_dutycontrol(stm32_dcmotor_serial::dutycontrol::Request  &req, 
                                      stm32_dcmotor_serial::dutycontrol::Response &res)
{
    sendData[0] = req.duty >= 0 ? 1 : 0;
    sendData[1] = static_cast<uint8_t>(fabs(req.duty));
    sendData[2] = static_cast<uint8_t>((fabs(req.duty) - sendData[1]) * 100);

    ROS_INFO("Duty = %f, Sent data 0x%x 0x%x 0x%x", req.duty, sendData[0], sendData[1], sendData[2]);

    res.result = true;
    ROS_INFO("Sent data to STM32 Result = %d", res.result);
    return true;
}

void STM32_DCMotor::spin()
{
    uint16_t encoderCnt = 0;
    uint16_t encoderCntOld = 0;
    int mtrDist;
    double mtrPos;

    ros::Rate loop_rate(20);

    while(nh_.ok())
    {
        ros::spinOnce();

        stm32_dcmotor_serial::encoder encoder_;

        current_time_ = ros::Time::now();
        double dt = (current_time_- last_time_).toSec();
        int deltaCNT = 0; 
        double deltaDist = 0;

        // Read the appropriate number of bytes from each serial port.

		serial_port_.Write(sendData);

        serial_port_.Read(revData, SERIAL_REV_LEN);
        if(revData.size() > 0)
        {
            encoderCnt = revData[0] << 8 | revData[1];

            //ROS_INFO("Recieve data 0x%x 0x%x, Encoder CNT = %d", revData[0], revData[1], encoderCnt);

            deltaCNT = (int)encoderCnt - (int)encoderCntOld;
            if(fabs(deltaCNT) > 65535 * 0.5) deltaCNT = (int16_t)encoderCnt - (int16_t)encoderCntOld;

            mtrDist += deltaCNT;
            mtrPos = (mtrDist % pulse_resoution_) / (double)pulse_resoution_ * 2 * M_PI;
            deltaDist = (double)deltaCNT / (double)pulse_resoution_ * 2 * M_PI;

            //ROS_INFO("DC Motor position = %.2f", mtrPos);
        }


        double dc_motor_vel = deltaDist / dt;
        //ROS_INFO("DC Motor velocity = %.2f rad/s", dc_motor_vel);

        encoder_.encoder_pulse = encoderCnt;
        encoder_.motor_vel = dc_motor_vel;

        dcmotor_tf_publisher(mtrPos);

        encoder_motor.publish(encoder_);

        // Print to the terminal what was sent and what was received.
        //std::cout << "\tSerial Port received:\t" << read_string << std::endl;

        //loop_rate.sleep();
        encoderCntOld = encoderCnt;
        last_time_ = current_time_;
    }    
    
    // Close the serial ports and end the program.
    serial_port_.Close() ;
}

void STM32_DCMotor::dcmotor_tf_publisher(double position)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base";
    transformStamped.child_frame_id = "motor_link";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0.3;

    tf2::Quaternion q;
    q.setRPY(0, 0, position);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}