# Bugbase
Bugbase is a ROS package with Arduino driver for low cost indoor robot base. The base consists of stepper motors, operates in open loop, and kinematics is differential drive.

[Bugbase large outdoor test](https://youtu.be/SEN7IrsHLQw)
![alt text](https://github.com/tranqkhue/bugbase/blob/master/media/bugbase_large.jpeg)
![alt text](https://github.com/tranqkhue/bugbase/blob/master/media/bugbase_small.jpeg)

## Why?
A lot of people need a indoor robotics platform for experimenting with algorithm such as GMapping, AMCL; or some people need to build a robot from scratch for their own applications. In the meanwhile, a complete Turtlebot kit may be expensive, or building a robot with good odometry (which is important for SLAM) is complex with DC motors, encoders and controllers.

So why don't make robots with stepper motors? Stepper motors and drivers (such as DRV8825) are affordable everywhere. And with stepper motors, the harass with encoders and controllers is gone, just open loop :D 

However, stepper motors have their own disadvantages. That's why I recommend this for mini robots with low mass (and hence low inertia) that stepper motors' open loop is sufficient.

## Components
- Arduino Uno, Mega, Leonardo... (I thought I would choose STM32 boards, but the need for people to modify my codes is important so I just chose Arduino anyway)
- Two stepper motors
- Two stepper motor drivers
- Two wheels, motor brackets and castor wheels
- (Optional) One Arduino CNC Shield or screw shield (for easy and reliable connection; I do not trust breadboards :D ). Make sure that pin 9 and 10 on Arduino Uno is used to create pulse controlling the motors

## Arduino pinouts:

You can define the pinouts in `/arduino/stepper_diff_wheel/stepper_diff_wheel.ino`, then compile and upload ;)
```
const int STEP_A_PIN = 10;
const int STEP_B_PIN = 9; 
const int DIR_A_PIN  = 12; 
const int DIR_B_PIN  = 11; 
const int STEP_EN    = 13;
```
`STEP_A_PIN` AND `STEP_B_PIN` must **NOT** be changed for Arduino Uno due to limitation of timers. For more information, please refer [FastAccelStepper library](https://github.com/gin66/FastAccelStepper) 

## Software installation
- Compile and upload Arduino sketch. Arduino sketch depends on [**FastAccelStepper**](https://github.com/gin66/FastAccelStepper) library
- `catkin_make`, `source` the workspace and run `roslaunch` as usual ROS things

## How to use this package
- Wiring electronics and Arduino; modify the PIN (if any), compile and upload the code to Arduino
- Modify the port and baudrate in roslaunch file.
- Launch the roslaunch file. Use test script to change these roslaunch params for good odometry and velocity control

## ROS parameters:
`wheel_base: 0.36` distance between left and right wheels
`steps_per_meter: 4800` number of steps (may be multiplied by microstepping) that required for one meter 
`left_inversed: true` inverse left motor direction
`right_inversed: false` inverse right motor direction

[Differential-drive robot kinematics reference](http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)

## TODO
- [x] Modify Arduino sketch to use **timer interrupt** than **AccelStepper**: Now use FastAccelStepper library

- [x] Design a simple base (laser acrylic cutting)

- [ ] Arduino Parameters can be changed in roslaunch file
