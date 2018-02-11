# System for quadruped robot

# General

This system serves our robot as data transfer, offers the joint, touchdown and IMU data for control algorithm, and transfer the joint command to the real robot. The framework of our system as follow,

<center>
![](images/framework.jpg)
</center>

As we can seen from the upper figure, our system, namely `SOFTWARE PLATFORM`, was locationed between the `ROS SUPPORT` and `HARDWARE`, was composed of the three mainly modules, `System Platform`, `Resource Repository`, and `Robot`. The mainly tasks of this system, we are listing as follow,
 1. The robot data from different communication styles should be collecting by the proper ways.
 2. The joint commands from someone were delivered to the real robot.
 3. The ROS environment should be supported.
 4. The control framework should be convenient, high-performance and simple.
 5. The control algorithm could test and debug in simulation environment, and doesn't need anyy other extra-work

# Ideas

Our design ideas include but not limited that,
 1. Wish to create the most instances through a plain file, such as `.txt`, `.xml`, etc. In this case, we choose the `xml` file format as our configure file.
 2. Wish to design a general framework that could hold the change in the real robot maybe occur in the future, such as, we maybe want to add a extra sensor, we maybe change the way of communication.
 3. Wish to our code could cross-platform. Perhaps, it's just a beautiful wish.


# Install Ubuntu mate linux system on raspberry PI3
 More detail about ubuntu mate, please reference to this [webset](http://ubuntu-mate.org/raspberry-pi/).
 1. Go to the seafile webset, [http://192.168.1.20](http://192.168.1.20), download the system image in the path `Shared with all/Tools/SystemISO/ubuntu/ubuntu-mate-16.04.2-desktop-armhf-raspberry-pi.img.xz`
 2. Extract this image at some place you like.
 3. Open a termial, and type the follow command, more details about installing images, please reference to the [install guide](https://www.raspberrypi.org/documentation/installation/installing-images/README.md) and the [linux guide](https://www.raspberrypi.org/documentation/installation/installing-images/linux.md):
 ```sh
 sudo dd bs=4M if=ubuntu-mate-16.04.2-desktop-armhf-raspberry-pi.img of=/dev/sdX conv=fsync
 ```
 Note, the `of=/dev/sdX` needs to be replaced by the real device file name. This device file name can be found by the command `df -h`.

如上图所示, 软件平台位于用户接口及算法等高级应用之间. 为用户提供硬件抽象以及低级别控制任务接口.主要功能如下所示:

1. 硬件机构通讯及数据协议实现
2. 所有硬件数据进入ROS环境的通道
3. 提供硬件机构控制接口
4. ROS Control机制实现
5. 机器人ROS运行环境准备

## 实现框架

恐龙软件实现, 主体分为双层, Based-Ubuntu 层, Based-ROS层.设计框架图如下所示:

<center>
![](../dragon_robot/img/driver_2.png)
</center>

- Based-Ubuntu 层: 仅依赖于Ubuntu, 完成通信及协议解析, 数据抽象, 线程管理, 用户接口实现等.
- Based-ROS层: 依赖于ROS, 管理Middleware对象, 将该对象所保存的数据引入ROS, 以及提供机器人ROS运行环境, 完成ROS Control机制实现等.

### Based-Ubuntu Layer
Middleware以单例模式实现, 管理HwUnit和Propagate对象, 分别完成数据抽象和通讯实现. 两者均以组合模式出现. 另外, 在实现中, HwUnit 和 Propagate分别编译成库, 以class_loader库进行动态加载, 具体类别信息以及组合方式等, 均以XML文件定义, 通过解析XML文件来完成Middleware对象的初始化.
HwUnit, 数据抽象基类. 具备两个抽象数据类型 -- HwCommand(命令数据) 和 HwState(状态数据), 和一个name(名称). 任何继承于HwUnit的子类, 均需要自定义这两种数据内容. 
针对恐龙系统当前情况, 该类别仅实现了两种硬件抽象 -- Motor 和 Encoder. 每个关节对应一个Motor(电机) 和 Encoder(编码器). 电机是关节的驱动设备, 编码器是关节的角度信息反馈设备. 电机包含电机当前状态信息(位置, 速度, 力矩), 以及命令信息(位置, 速度, 力矩). 当然, 每个电机, 单次只能在某一种模式下进行控制. 例如, 在位置模式下, 只能使用位置命令. 编码器仅包含状态信息.
Propagate, 通信方式实现基类. 接收HwUnit命令数据和状态数据的注册, 获取这两种数据的实际句柄, 并提供实际数据读写的实现, 在读的过程中, 包括对读取数据的协议解析并更新HwUnit状态数据.针对当前恐龙硬件机构, 已实现PCAN通讯方式.
Middleware管理一个读写线程, 实例化Middleware并初始化成功后, 开启该线程. 该线程循环读写操作, 保证数据的实时性及命令的及时下发.

### Based-ROS Layer
该层包含一个ROSAdapter类, 该类完成通过Middleware对象获取实际数据内容, 将所有数据以ROS的形式上传到ROS环境中来.提供两种分支共选择使用, 通过使用use_ros_control参数来指定.

1. ROS Control机制
use_ros_control为真时, 使用ROS Control机制. 所有控制器通过请求Controller Manager服务来进行加载, 切换及卸载. 默认将会加载joint_state_controller控制器, 该控制器是ROS提供的默认控制器之一, 将会在ROS Control中, 通过控制器的方式完成/joint_states话题数据的发布.
该方式是推荐使用方式, 可以与Gazebo仿真平台进行无缝对接, 所有控制算法均可以在Gazebo上进行仿真调试, 当调试完成后, 不需更改任何内容就可以在恐龙软件平台上进行加载.

2. ActionService服务
该类型控制方式, 将会开启一个线程, 专门用作发布实时话题/joint_states, 并且, 开启一个ActionService, 完成FollowJointTrajectory的任务. 该服务可以被MoveIt!所使用, 并且, 也为用户提供一种可选控制方式.

## ROS包
上述内容, 均实现于dragon_driver包中, 该包是catkin所管理的一个ROS包. 依赖dragon_description包. 在dragon_driver包中, 提供一个launch文件, 恐龙软件平台通过该launch文件进行发起. 在launch文件中, 将会发起机器人ROS运行环境所需要的必要组件. 例如ROS Master, 机器人模型文件, robot_state_publisher等. 并且大量的参数可直接在命令行中进行设定.