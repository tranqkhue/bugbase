# Bugbase
Bugbase is a ROS package with Arduino driver for low cost indoor robot base. The base consists of stepper motors, operating in open loop, and kinematics is differential drive.

## Why?
A lot of people need a indoor robotics platform for experimenting with algorithm such as GMapping, AMCL; or some people need to build a robot from scratch for their own applications. In the meanwhile, a complete Turtlebot kit may be expensive, or building a robot with good odometry (which is important for SLAM) is complex with DC motors, encoders and controllers.

So why don't make robots with stepper motors? Stepper motors and drivers (such as DRV8825) are affordable everywhere. And with stepper motors, the harass with encoders and controllers is gone, just open loop :D 

However, stepper motors have their own disadvantages. That's why I recommend this for mini robots with low mass (and hence low inertia) that stepper motors' open loop is sufficient.

## Components
- Arduino Uno, Mega, Leonardo... (I thought I would choose STM32 boards, but the need for people to modify my codes is important so I just chose Arduino anyway)
- Two stepper motors
- Two stepper motor drivers
- Two wheels, motor brackets and castor wheels
- (Optional) One Arduino CNC Shield (for easy and reliable connection; I do not trust breadboards :D )
![alt text](https://github.com/tranqkhue/bugbase/blob/master/media/cnc_shield.jpg?raw=true)

## Software installation
- Compile and upload Arduino sketch. Arduino sketch depends on **AccelStepper** library
- `catkin_make`, `source` the workspace and run `roslaunch` as usual ROS things

## TODO
- [ ] Modify Arduino sketch to use **timer interrupt** than **AccelStepper
- [ ] Design a simple base (laser acrylic cutting)
