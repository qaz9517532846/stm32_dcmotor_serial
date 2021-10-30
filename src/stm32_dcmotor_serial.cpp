#include <stm32_dcmotor_serial/stm32_dcmotor_serial.h>

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
    serial_port_.Write(req.duty.c_str());

    // Wait until the data has actually been transmitted.
    serial_port_.DrainWriteBuffer() ;

    ROS_INFO("Sent data to STM32 = %s", req.duty.c_str());

    res.result = true;
    ROS_INFO("Sent data to STM32 Result = %d", res.result);
    return true;
}

void STM32_DCMotor::spin()
{
    while(nh_.ok())
    {
        ros::spinOnce();

        current_time_ = ros::Time::now();
        double dt = (current_time_- last_time_).toSec();

        try
        {
            // Read the appropriate number of bytes from each serial port.
            serial_port_.Read(read_string, 10, 1500);
            ROS_INFO("Recieve data from STM32 = %s", read_string.c_str());
            ROS_INFO("DC Motor Encoder count = %d", encoder_data_process(read_string));

            move_resolution_ = (float)encoder_data_process(read_string) / pulse_resoution_ * 2 * 3.1415926;
            motor_pos_ = (float)(encoder_data_process(read_string) % pulse_resoution_) / pulse_resoution_ * 2 * 3.1415926;

            ROS_INFO("DC Motor Move rad = %.2f", move_resolution_);
            ROS_INFO("DC Motor position = %.2f", motor_pos_);
        }
        catch(const ReadTimeout&)
        {
            std::cerr << "The Read() call has timed out." << std::endl;
        }

        double dc_motor_vel = (move_resolution_ - last_move_resolution_) / dt;
        last_move_resolution_ = move_resolution_;
        ROS_INFO("DC Motor velocity = %.2f rad/s", dc_motor_vel);
        last_time_ = current_time_;

        // Print to the terminal what was sent and what was received.
        //std::cout << "\tSerial Port received:\t" << read_string << std::endl;
    }    
    
    // Close the serial ports and end the program.
    serial_port_.Close() ;
}

int STM32_DCMotor::encoder_data_process(std::string data)
{
    char data_array[10];
    strcpy(data_array, data.c_str());

    int counter_;
    for(int i = 2; i <= 9; i++)
    {
        counter_ += charToint_Number(data_array[i]) * pow(10, 9 - i);
    }
    
    if(data_array[0] == '0')
    {
        return 1 * counter_;
    }
    else
    {
        return -1 * counter_;;
    }
}

int STM32_DCMotor::charToint_Number(char data)
{
    if(data == '1')
    {
        return 1;
    }
    else if(data == '2')
    {
        return 2;
    }
    else if(data == '3')
    {
        return 3;
    }
    else if(data == '4')
    {
        return 4;
    }
    else if(data == '5')
    {
        return 5;
    }
    else if(data == '6')
    {
        return 6;
    }
    else if(data == '7')
    {
        return 7;
    }
    else if(data == '8')
    {
        return 8;
    }
    else if(data == '9')
    {
        return 9;
    }
    else
    {
        return 0;
    }
}