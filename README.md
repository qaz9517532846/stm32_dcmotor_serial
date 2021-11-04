# stm32_dcmotor_serial
Control a Motor and read a AB Encoder using STM32 MCU.

- Software: Robot Operating System and LibSerial.

- Version: kinetic, melodic, noetic.

------

###  Install LibSerial.

  ``` $ sudo apt-get install -y libserial-dev ```
  
------

### Clone stm32_dcmotor_serial

``` bash
$ cd <catkin_workspace>/src
```

``` bash
$ git clone https://github.com/qaz9517532846/stm32_dcmotor_serial.git
```

``` bash
$ cd ..
```

``` bash
$ catkin_make
```

------

### Run serial_ros server

``` bash
$ roslaunch stm32_dcmotor_serial stm32_dcmotor.launch
```

### Run serial_ros client

``` bash
$ rosrun stm32_dcmotor_serial STM32_DCMotor_client <duty>
```

------

### ROS Parameter

- Serial_Port(type: string, default: "/dev/ttyACM0") : Serial Comunication with device(ex. Arduino, STM32, Renesas, Linkit board and MCU).

- Baund_rate(type: int, default: 115200): setting device baund rate.

- character_size(type: int, default: 8): setting device character size.

- stop_bits(type: int, default: 1): setting device stop bits.

- parity(type: int, default: 0): setting device parity.

   - 1 is even.

   - 2 is odd.

   - 3 is none.

- flow_control(type: int, default: 1): setting device flow control.

   - 0 is none.

   - 1 is hardware.

- pulse_resoution(type: int, default: 512): 512p/r

------

### Demo

[Demo Video](https://www.youtube.com/watch?v=4B3UtQTfNhs)

------

### Reference

[1]. LibSerial. https://github.com/crayzeewulf/libserial

[2]. STM_DCMotor_Encoder. https://github.com/qaz9517532846/STM_DCMotor_Encoder

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
