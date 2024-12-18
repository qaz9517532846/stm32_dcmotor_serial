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

| Item           | Type                       | Default                    | illustrate                    |
| ---            | ---                        | ---                        | ---                           |
| Serial_Port    | String                     |  "/dev/ttyACM0"            | Serial Comunication with device(ex. Arduino, STM32, Renesas, Linkit board and MCU). |
| Baund_rat      | Int                        | 115200                     | Setting device baund rate.     |
| character_size | Int                        | 8                          | Setting device character size. |
| stop_bits      | Int                        | 1                          | Setting device stop bits.      |
| parity         | Int                        | 0                          | Setting device parity.         |
| flow_control   | Int                        | 1                          | setting device flow control.   |
| pulse_resoution| Int                        | 512                        | Setting pulse of rev.          |

------

### Send data using serial communication.

- Duty

| Address        | Address 0                  | Address 1                  | Address 2|
| ---            | ---                        | ---                        | ---                           |
| data type      | UINT8                      | UINT8                      | UINT8                         |
| data           | direction(0 or 1)          | duty hightbyte (0 - 100)   | duty lowbyte (0 - 99) * 0.01  |

------

### Received data using serial communication.

- Encoder counter

| Address        | Address 0                  | Address 1                  |
| ---            | ---                        | ---                        |
| data type      | UINT8                      | UINT8                      |
| data           | encoder cnt hightbyte      | encoder cnt lowbyte        |

------

### Demo

[Demo Video](https://www.youtube.com/watch?v=ZO4yH0yluY4)

------

### Reference

[1]. LibSerial. https://github.com/crayzeewulf/libserial

[2]. STM_DCMotor_Encoder. https://github.com/qaz9517532846/STM_DCMotor_Encoder

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
