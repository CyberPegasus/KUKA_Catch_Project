**目录**

[toc]

# 0 运行环境

操作系统：Ubuntu 18.04

```python
http://releases.ubuntu.com/18.04/  # 镜像文件地址
```

机械臂：KUKA R700 sixx

```
ROS Industrial的软硬件驱动：
http://wiki.ros.org/kuka_experimental?distro=kinetic
```

控制柜：KUKA KR C4

~~~
驱动程序：WorkVisual 4.0
下载地址：https://kuka.sharefile.eu/share/view/sd554971b8f944159
通讯插件：EthernetKRL
~~~

# 1 KUKA KR6 R700 Sixx机械臂方面

## 1.1 设备安装

***桌面安装：***铝合金型槽开口6mm，深度5mm；梯形截面，底面宽度4mm，上方宽度11mm。

***配件选择：***欧标20型M4螺母+M4螺丝

***相机安装方案：***通过门型铝合金型材来架设Kinect v2相机

***EFG-20夹爪的安装与调试：***

1. 夹爪接线说明

   | 端口 | 说明                                 | 颜色     | 与M12口的配置（对应四芯线）     |
   | ---- | ------------------------------------ | -------- | ------------------------------- |
   | 1    | 夹爪电源正                           | 黑色     | 1（红）                         |
   | 2    | 未知                                 | 灰色     | 空                              |
   | 3    | 未知                                 | 黄绿相间 | 2（黑）——空置（也可能是控制线） |
   | 4    | 状态控制线（高电平闭合，低电平张开） | 棕色     | 3（绿）                         |
   | 5    | 夹爪地线                             | 蓝色     | 4（黄）                         |

   注意:当机械臂方向控制端口输入为高电平则打开，输入为负电平则夹合。

2.机械臂控制柜

​     在WorkVisual中插入总线模块——SYS-X44，添加IO Board，并将其输出与KRC的输入输出变量相链接。

![](https://www.cyberpegasus.cn/wp-content/uploads/2020/07/Annotation-2020-07-06-163617.png)

​    经过测试，X12接口中，上方中央输出对应IO板的output 0，也就是OUT[10]；而下方中央输出对应IO板的output 1，也就是OUT[11]. 

​	配置成功：OUT[10]持续输出高电平，当OUT[11]输出高电平时夹爪闭合，输出低电平时夹爪张开。

​	Notes：夹爪闭合较慢，实际使用时可以wait 5s，等待夹爪闭合。


## 1.2 基于ROS KUKA包的机械臂与PC通讯测试

### 1.2.1 EthernetKRL包安装

在`WorkVisual4.0`安装`EthernetKRL`，并部署到机械臂控制柜，安装文件与安装方法具体见杨学长毕设文档

### 1.2.2 EthernetKRL与ROS的初步通讯

注意！！！此步骤完全参考ROS Industrial的KUKA包——kuka_experimental

总体基于ROS-i的KUKA包：`https://github.com/ros-industrial/kuka_experimental/`

通讯基于KUKA包的eki：

`https://github.com/ros-industrial/kuka_experimental/tree/indigo-devel/kuka_eki_hw_interface/`

配置步骤：

- 确保机械臂上位机的IP地址已定

- 将`ROS kuka_experimental`包中的`kuka_eki_hw_interface/krl`下的`EkiHwInterface.xml`进行修改，确保IP地址与端口正确，并在机械臂上位机中登陆专家用户，将文件复制到`C:\KRC\ROBOTER\Config\User\Common\EthernetKRL\`下。

  修改如下所示：

```xml
<ETHERNETKRL>
   <CONFIGURATION>
      <EXTERNAL>
         <TYPE>Client</TYPE>                  <!-- Users connect as clients -->
      </EXTERNAL>
      <INTERNAL>
         <ENVIRONMENT>Program</ENVIRONMENT>  
          <!--删除本次连接与操作相结合，Program：在机械臂运动后删除连接-->
          <!-- Server run via robot interpreter -->
         <BUFFERING Limit="512" /> 
          <!--缓存区域设为最大值-->
          <!-- Allow buffering of up to 512 messages (system max) -->
         <ALIVE Set_Flag="1" />
         <!--<TIMEOUT Connect="2000">默认的断开重连时间为2000ms-->
          <!-- Use $flag[1] to indicate alive/good connection status -->
         <IP>address.of.robot.controller</IP>
          <!--需要自己填入？此处设为机械臂控制柜的IP地址-->
          <!-- IP address for EKI interface on robot controller (KRC) -->
         <PORT>54600</PORT>                   
          <!-- Port of EKI interface on robot controller (in [54600, 54615]) -->
         <PROTOCOL>UDP</PROTOCOL>             
          <!-- Use UDP protocol -->
      </INTERNAL>
   </CONFIGURATION>

   <!-- Configured XML structure for data reception (external client to robot)
        <RobotCommand>
           <Pos A1="..." A2="..." A3="..." A4="..." A5="..." A6="...">
           </Pos>
        </RobotCommand>
   -->
   <RECEIVE><!--机械臂控制柜接收的命令-->
      <XML>
         <!-- Joint position command -->
         <ELEMENT Tag="RobotCommand/Pos/@A1" Type="REAL" />
         <ELEMENT Tag="RobotCommand/Pos/@A2" Type="REAL" />
         <ELEMENT Tag="RobotCommand/Pos/@A3" Type="REAL" />
         <ELEMENT Tag="RobotCommand/Pos/@A4" Type="REAL" />
         <ELEMENT Tag="RobotCommand/Pos/@A5" Type="REAL" />
         <ELEMENT Tag="RobotCommand/Pos/@A6" Type="REAL" />
      </XML>
   </RECEIVE>

   <!-- Configured XML structure for data transmission (robot to external client)
        <RobotState>
           <Pos A1="..." A2="..." A3="..." A4="..." A5="..." A6="...">
           </Pos>
           <Vel A1="..." A2="..." A3="..." A4="..." A5="..." A6="...">
           </Vel>
           <Eff A1="..." A2="..." A3="..." A4="..." A5="..." A6="...">
           </Eff>
        </RobotState>
   -->
   <SEND><!--机械臂控制柜向外发送的命令-->
      <XML>
         <!-- Joint state positions -->
         <ELEMENT Tag="RobotState/Pos/@A1"/>
         <ELEMENT Tag="RobotState/Pos/@A2"/>
         <ELEMENT Tag="RobotState/Pos/@A3"/>
         <ELEMENT Tag="RobotState/Pos/@A4"/>
         <ELEMENT Tag="RobotState/Pos/@A5"/>
         <ELEMENT Tag="RobotState/Pos/@A6"/>
         <!-- Joint state velocities -->
         <ELEMENT Tag="RobotState/Vel/@A1"/>
         <ELEMENT Tag="RobotState/Vel/@A2"/>
         <ELEMENT Tag="RobotState/Vel/@A3"/>
         <ELEMENT Tag="RobotState/Vel/@A4"/>
         <ELEMENT Tag="RobotState/Vel/@A5"/>
         <ELEMENT Tag="RobotState/Vel/@A6"/>
         <!-- Joint state efforts (torques) -->
         <ELEMENT Tag="RobotState/Eff/@A1"/>
         <ELEMENT Tag="RobotState/Eff/@A2"/>
         <ELEMENT Tag="RobotState/Eff/@A3"/>
         <ELEMENT Tag="RobotState/Eff/@A4"/>
         <ELEMENT Tag="RobotState/Eff/@A5"/>
         <ELEMENT Tag="RobotState/Eff/@A6"/>
         <!-- Interface state -->
         <ELEMENT Tag="RobotState/RobotCommand/@Size"/>  
          <!-- Number of elements currently in command buffer -->
      </XML>
   </SEND>
</ETHERNETKRL>
```

- 将文件`kuka_eki_hw_interface.dat`，`kuke_eki_hw_interface.src`复制到`KRC:\R1\Program`

- 修改`ROS kuka_experimental`内部的`/kuka_eki_hw_interface/test`和`/kuka_eki_hw_interface/config`两个文件夹下的3个yaml文件，并修改`test_params.yaml`中机械手臂的IP地址与端口。

- 基于`/kuka_eki_hw_interface/test`下的编写launch文件，如下所示：

  ```xml
  <?xml version="1.0" encoding="utf-8"?>
  <launch>
      <param name="robot_description" command="$(find xacro)/xacro.py '$(find kuka_kr6_support)/urdf/kr6r700sixx.xacro'"/>
      <rosparam file="$(find kuka_eki_hw_interface)/test/test_params.yaml" command="load" />
  
      <!-- Start EKI interface -->
      <node name="kuka_eki_hardware_interface" pkg="kuka_eki_hw_interface"
        type="kuka_eki_hw_interface_node" respawn="false"
        output="screen"
        required="true"/>
  
      <!-- Load joint controller configurations from YAML file to parameter server -->
      <rosparam file="$(find kuka_eki_hw_interface)/config/hardware_controllers.yaml" command="load"/>
      <!-- Load standard kuka controller joint names from YAML file to parameter server -->
      <rosparam file="$(find kuka_eki_hw_interface)/config/controller_joint_names.yaml" command="load"/>
  
      <!-- Load controllers -->
      <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"
          args="position_trajectory_controller joint_state_controller --shutdown-timeout 1"/>
      <!-- Load robot state publisher -->
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  </launch>
  ```

- 在手动测试前应确保：

  - 各处(`机械臂网络设置，机械臂xml文件，ros eki包内test_params.yaml`)IP地址与端口均设置正确
  - You have a launch-file loading the network parameters, robot description, kuka_hw_interface, hardware controller and controller joint names.

- 在机械臂上启动服务器

  - 在示教板上，进入模式**T1**以进行测试
  - 导航至，`KRC:\R1\Program`然后选择`kuka_eki_hw_interface.src`
  - 选中文件并进入编辑器，按住灰色解锁开关再按住绿色运行开关，机械臂解除刹车并显示BCO消息

- 在ROS新建三个终端：

  - 第一个：运行roscore，`$ roscore`
- 第二个：先`source devel/setup.bash`，运行启动文件`$ roslaunch kuka_eki_hw_interface test_hardware_interface.launch`
  - 第三个：运行RQT可视化角度发布器` rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller`
  - 选择**控制器管理器ns**和**控制器**，此时可在可视化界面移动机械臂的每个关节。

![rqt_joint_controller](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/rqt_joint_api-e1591759600968.png)

### 1.2.3 本阶段问题汇总

1. 如何使得机械臂在自动运行的状态下可以连续接收来自PC的指令？

2. 如何通过编程实现发布关节角空间坐标来使机械臂运行到指定位姿？



## 1.3 机械臂与PC的连续通讯与关节角空间控制客户端开发

  ### 1.3.1 基于ROS KUKA_EKI包的连续通讯尝试

*问题：之前测试时机械臂使用的T1模式手动投运，需要一直按着按键，则是否机械臂工作在自动模式下也可以正常接受关节角命令呢？*

**准备工作：**阅读KUKA官方的编程指南，了解控制器如何投入自动运行，资料如下	`https://www.jianguoyun.com/p/Dd6uRpsQ8PmrCBjglqED (访问密码：q5D8US)`

**自动投运问题解决：**具体为无法进入AUTO模式来投运，错误代码有两个：1.`SYS-X44`,详细错误为No Network Responce，初步考虑为内部某模块未连接;2.ECat Stack Initialization Failed，可能EtherCAT出错。进入WorkVisual软件调取当前机械臂的激活项目，对总线下EtherCAT进行拓扑扫描，发现项目原有的EtherCAD模块——输入输出板（IOB-16-16B）不存在，则将其删去（但估计实体还在控制柜内未接线），并部署到控制柜。

TIPS：控制柜的IP地址只有在T1手动模式下才能更改，在投入运行的网络配置中;

**基于 ROS KUKA_EKI包与ROS rqt可视化界面的连续通讯：**

解决好自动投运问题后，使用AUTO模式运行，则按照1.2.2的步骤可以进入自动运行，并按rqt_joint_trajectory_controller节点发布的关节角数据来运行

### 1.3.2 发布关节角空间坐标的程序节点开发

*问题：如何通过编程实现发布关节角空间坐标来使机械臂运行到指定位姿？*

KUKA_EKI通讯驱动实际上和机械臂进行UDP通讯，收发的是XML文件。PC端向机械臂端发送关节角空间坐标的命令，机械臂向KUKA_EKI反馈关节角度、角速度、受力以及机械臂状态。具体见1.2.2提到的`EkiHwInterface.xml`

KUKA_EKI包向外部发布了关节角控制的节点，而启动`rqt_joint_trajectory_controller`节点后对6个关节角进行控制。所以编程的目的是代替 `rqt_joint_trajectory_controller`节点来对机械臂进行基于关节角空间坐标的运动控制。

**准备工作—Topic与分析：** 

运行节点，执行`rostopic list`，得到以下节点：

```
/joint_states
/position_trajectory_controller/command
/position_trajectory_controller/follow_joint_trajectory/cancel
/position_trajectory_controller/follow_joint_trajectory/feedback
/position_trajectory_controller/follow_joint_trajectory/goal
/position_trajectory_controller/follow_joint_trajectory/result
/position_trajectory_controller/follow_joint_trajectory/status
/position_trajectory_controller/state
/rosout
/rosout_agg
/tf
/tf_static
```

其中发布与订阅关节角空间坐标的主要是position_trajectory_controller包的follow_joint_trajectory节点，则考虑编写程序，通过action通讯机制来通讯。

![action](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/action.png)

**基于关节角空间坐标的控制程序开发：**

程序编写如下，通过action通讯定义发送指令的Client，类型为FollowJointTrajectoryAction，通讯对象server为/position_trajectory_controller/follow_joint_trajectory

```python
#!/usr/bin/env python
# -*- coding: UTF-8 -*- #由此能中文注释
from __future__ import print_function
import time
import roslib; roslib.load_manifest('kuka_eki_hw_interface')#导入KUKA_EKI节点的工作环境
import rospy
import actionlib #action通讯功能包
from control_msgs.msg import * #订阅控制器的消息
from trajectory_msgs.msg import *#订阅轨迹规划器的消息
JOINT_NAMES = ['joint_a1', 'joint_a2', 'joint_a3','joint_a4', 'joint_a5', 'joint_a6']#对应KUKA的yaml定义文件
Q_catch = [0.30,-1.14,0.84,-0.79,0.00,0.17]
client = None # 定义客户端变量并赋空值

#运行到指定抓取位姿
#提供关节角空间坐标
def move_to_catch():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
    #定义（经过的）目标关节角空间坐标，时间设为6s
	JointTrajectoryPoint(positions=Q_catch, velocities=[0]*6, time_from_start=rospy.Duration(6.0))
	]
    client.send_goal(g)#发送指令
    try:
        client.wait_for_result()#等待机械臂运行到指定位姿
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client #定义客户端为全局变量
    try:
        rospy.init_node("kuka_control", anonymous=True, disable_signals=True)#将该程序节点初始化
        #给Client赋值，设类型为FollowJointTrajectoryAction，通讯对象server为/position_trajectory_controller/follow_joint_trajectory
        client = actionlib.SimpleActionClient('/position_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()#等待服务连接
        print("Successfully Connected to server\nMove to catch..")
        move_to_catch()
	print("action completed!Equit...")
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()

```



**程序测试：**

机械臂投入自动运行，PC执行`roscore`和`roslaunch kuka_eki_hw_interface test_hardware_interface.launch`，然后运行程序进行测试。

测试结果正确。

### 1.3.3 笛卡尔空间坐标向关节角空间坐标的转换

*问题：上一节程序实现了基于关节角空间坐标的控制，那么笛卡尔空间坐标下如何进行控制？*

*由于ROS自带的OMPL开源轨迹规划库对于机械臂轨迹规划来说不太适用于长距离轨迹规划（毕设经验），所以直接发布关节角空间轨迹规划数据给机械臂比较靠谱。那么问题来了，为了实现对已知世界坐标系下工件的抓取，需要自己编程或者调用ROS的TF包来完成笛卡尔空间坐标到关节角空间坐标的转换。即对KUKA KR6 R700 SIXX的逆运动学求解。*

**正运动学验证与逆运动学求解：**

***！！！此处参考文献：陈恳. 基于机器视觉的物料分拣工业机器人关键技术研究[D].深圳大学,2017.***

KUKA KR6 R700 SIXX的官网尺寸图：

![KUKA-Parameters](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/KUKA-DH.png)

KUKA KR6 R700的转轴正负方向示意图：

![KUKA_xyz](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/kuka-xyz.bmp)

首先对机械臂进行建系：

**注意！由于逆运动学需要使用Pieper法，且KUKA商用机械臂符合Pieper法准则，故建系时需要将后三个关节的坐标系建在同一个原点**

![KUKA_XYZ_1](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/kuka_xyz1.bmp)

可以得到机械臂的D-H参数为：

| Kinematics | theta[度]  | a[m]  | d[m]  | alpha[度] | 关节限制 |
| ---------- | ---------- | ----- | ----- | --------- | ---------- |
| joint_a1   | $-\theta_1$ | 0     | 0.400 | 0      | $\pm170$ |
| joint_a2   | $\theta_2$ | 0.025 | 0     | -90       | -190~45 |
| joint_a3   | $\theta_3-90$ | 0.315 | 0     | 0         | -120~156 |
| joint_a4   | $-\theta_4$ | 0.035 | 0.365 | -90       | $\pm185$ |
| joint_a5   | $\theta_5$ | 0     | 0     | 90        | $\pm120$ |
| joint_a6   | $-\theta_6-180$ | 0     | 0.080 | -90       | $\pm350$ |

TIPS：经研究发现，经典DH建模的模型和实际机械臂的某些关节的零位不同，这可能和机械臂的零位设置有关。实际由于零位的不同，需要对角度进行修正：

(左边为角度理论值，右边为机械臂示教器实际值)

$$\theta_1=-\theta_{r1}$$

$$\theta_2=\theta_{r2}$$

$$\theta_3=\theta_{r3}-90$$

$$\theta_4=-\theta_{r4}$$

$$\theta_5=\theta_{r5}$$

$$\theta_6=-(\theta_{r1}+180)$$

进入MATLAB使用Robotic Toolbox进行测试，测试代码如下：

```matlab
clear;
clc;
%建立机器人模型
%       theta    d        a        alpha     offset
L1=Link([0       0.4      0     0     0     ],'modified'); 
L2=Link([0       0        0.025     -pi/2         0     ],'modified');
L3=Link([-pi/2       0        0.315     0      0     ],'modified');
L4=Link([0       0.365    0.035     -pi/2     0     ],'modified');
L5=Link([0       0        0         pi/2      0     ],'modified');
L6=Link([-pi       0.08     0        -pi/2         0     ],'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','kuka');
robot.teach();
```



**正运动学DH参数表验证：**

经过验证，此DH模型除零位与真实机械臂不同外，其他均正常。



![KUKA_DH_Simulate](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/kuka_DHsimulate.png)

![KUKA_Robot_p1](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/IMG_4075-scaled-e1591789086913.jpg)





*下一步计划：根据DH参数建立的正运动学变换矩阵去求逆运动学解析解，每组解应对应8个不同的解*

**KUKA机械臂逆运动学求解：**

基于Pieper建DH参数

| Kinematics | theta[度]       | a[m]  | d[m]  | alpha[度] | 关节限制 |
| ---------- | --------------- | ----- | ----- | --------- | -------- |
| joint_a1   | $-\theta_1$     | 0     | 0.400 | 0         | $\pm170$ |
| joint_a2   | $\theta_2$      | 0.025 | 0     | -90       | -190~45  |
| joint_a3   | $\theta_3-90$   | 0.315 | 0     | 0         | -120~156 |
| joint_a4   | $-\theta_4$     | 0.035 | 0.445 | -90       | $\pm185$ |
| joint_a5   | $\theta_5$      | 0     | 0     | 90        | $\pm120$ |
| joint_a6   | $-\theta_6-180$ | 0     | 0     | -90       | $\pm350$ |

使用MATLAB符号运算推导$T^6_0$

（未完待续）

（求逆解程序先放一下，后续再补充；目前先尝试通过EthernetKRL进行收发XML文件的通讯。）



## 1.4 EthernetKRL通讯研究与笛卡尔空间控制客户端开发

突然发现，要实现在笛卡尔空间下对机械臂进行控制，可以通过EthernetKRL API来直接向机械臂控制柜发送笛卡尔空间下的XYZABC位姿，以控制机械臂达到指定位姿，那么平台结构就可以得到简化，用不到KUKA_Experimental包了。

首先定义通讯配置，以及与机械臂收发的XML信息，命名为EkiCtrl.xml：

```xml
<ETHERNETKRL>
    <CONFIGURATION>
      <EXTERNAL>
         <TYPE>Client</TYPE>              
      </EXTERNAL>
      <INTERNAL>
         <ENVIRONMENT>Program</ENVIRONMENT>   
         <BUFFERING Limit="512" />            
         <ALIVE Set_Flag="1" />              
         <IP>192.168.1.234</IP>               
         <PORT>54600</PORT>                  
         <PROTOCOL>TCP</PROTOCOL>             
      </INTERNAL>
    </CONFIGURATION>
    <RECEIVE>
      <XML>
         <ELEMENT Tag="Robots/Command" Type="INT" /> 
		 <ELEMENT Tag="Robots/Pos/@X" Type="REAL" /> 
		 <ELEMENT Tag="Robots/Pos/@Y" Type="REAL" /> 
		 <ELEMENT Tag="Robots/Pos/@Z" Type="REAL" /> 
		 <ELEMENT Tag="Robots/Pos/@A" Type="REAL" /> 
		 <ELEMENT Tag="Robots/Pos/@B" Type="REAL" /> 
		 <ELEMENT Tag="Robots/Pos/@C" Type="REAL" /> 
      </XML>
   </RECEIVE>
   <SEND>
       <XML>
         <ELEMENT Tag="RobotState/Joint/@A1" Type="REAL"/>
         <ELEMENT Tag="RobotState/Joint/@A2" Type="REAL"/>
         <ELEMENT Tag="RobotState/Joint/@A3" Type="REAL"/>
         <ELEMENT Tag="RobotState/Joint/@A4" Type="REAL"/>
         <ELEMENT Tag="RobotState/Joint/@A5" Type="REAL"/>
         <ELEMENT Tag="RobotState/Joint/@A6" Type="REAL"/>
       </XML>
   </SEND>
</ETHERNETKRL>

```



基于Python编写笛卡尔空间控制客户端：

```python
# -*- coding: UTF-8 -*-
from socket import *
from xml.dom import minidom

# socket配置
host = 'localhost'
port = 54600
buffer_size = 1024
# 机械臂状态
joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def write_xml(filename, cmd, x, y, z, a, b, c):
    print("Writing Command to XML")
    doc = minidom.Document()
    # 主节点Robots
    robots = doc.createElement("Robots")
    doc.appendChild(robots)
    # 命令节点Command，一级节点
    command = doc.createElement("Command")
    # 写入Command的值
    command_value = doc.createTextNode(str(cmd))
    command.appendChild(command_value)
    robots.appendChild(command)
    # 姿态节点Pos，一级节点
    pos = doc.createElement("Pos")
    x_value = pos.setAttribute("X", str(x))
    y_value = pos.setAttribute("Y", str(y))
    z_value = pos.setAttribute("Z", str(z))
    a_value = pos.setAttribute("A", str(a))
    b_value = pos.setAttribute("B", str(b))
    c_value = pos.setAttribute("C", str(c))
    robots.appendChild(pos)
    # 从XML转为Bytes类型
    write_data = doc.toprettyxml(indent="\t", newl="\n", encoding="utf-8")
    # f = open(filename, "wb")
    # f.write()
    # f.close()
    return write_data


def read_xml(recv_message):
    print("reading message from XML")
    doc = minidom.parseString(str(recv_message, encoding='utf8'))
    # doc = minidom.parse("test_recv.xml")
    root = doc.getElementsByTagName("Joint")[0]
    joint_state[0] = float(root.getAttribute("A1"))
    joint_state[1] = float(root.getAttribute("A2"))
    joint_state[2] = float(root.getAttribute("A3"))
    joint_state[3] = float(root.getAttribute("A4"))
    joint_state[4] = float(root.getAttribute("A5"))
    joint_state[5] = float(root.getAttribute("A6"))
    print("The current joints of robotic arm are as follow:")
    print(joint_state)


def socket_client(ctrl_command):
    addr = (host, port)
    while True:
        ctrl_client = socket(AF_INET, SOCK_STREAM)
        # 连接机械臂服务端
        print('Begin to connect...')
        ctrl_client.connect(addr)
        print('Connection Established')
        # 发送控制命令
       #  send_data = bytes(ctrl_command, encoding="utf8")
        ctrl_client.send(ctrl_command)
        print('Send Successfully')
        recv_data = ctrl_client.recv(buffer_size)
        print('Receive Successfully....Begin to parse')
        read_xml(recv_data)
        ctrl_client.close()


def main():
    # 写入XML格式控制命令
    ctrl_command = write_xml("EkiCtrl.xml", 1, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    # 发送控制命令并读取回传信息
    socket_client(ctrl_command)

    # write_xml_test("test_recv.xml", 1, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    # read_xml(0)


if __name__ == "__main__":
    main()
```



编写KUKA机械臂控制服务端程序，放入Program中运行：

```
&ACCESS RV1
&REL 12
DEF connect1()
DECL POS Posv
DECL POS Cent
DECL EKI_STATUS RET
DECL INT F
DECL INT RobStatus

EXT BAS (BAS_COMMAND:IN,REAL:IN)
DECL AXIS HOME
BAS(#INITMOV,0)
HOME={AXIS:A1 0,A2 -90,A3 90,A4 0,A5 0,A6 0}
PTP HOME

$BASE=BASE_DATA[1]
$TOOL=TOOL_DATA[1]

Posv = {X 0.0, Y 0.0, Z 0.0, A 0.0, B 0.0, C 0.0}
Cent = {X 0.0, Y 0.0, Z 0.0, A 0.0, B 0.0, C 0.0}
Posv = $POS_ACT
F =0
$FLAG[1] = FALSE
RobStatus = 0

RET=EKI_Init("EkiCtrl")



WHILE (F <> 1)
RET=EKI_Open("EkiCtrl")
EKI_CHECK(RET,#QUIT)
;WAIT FOR 2
;WAIT SEC 1
;FOLD WAIT Time=1 sec;%{PE}%R 8.3.31,%MKUKATPBASIS,%CWAIT,%VWAIT,%P 3:1
;WAIT SEC 1
;ENDFOLD

WAIT FOR $FLAG[1]
$FLAG[1] = FALSE

RET = EKI_GetInt("EkiCtrl", "Robots/Command",RobStatus)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@X", Posv.X)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@Y", Posv.Y)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@Z", Posv.Z)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@A", Posv.A)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@B", Posv.B)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@C", Posv.C)


SWITCH RobStatus
	CASE 1
		LIN Posv
	CASE 2
		F = 1
ENDSWITCH

RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A1", $AXIS_ACT.A1) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A2", $AXIS_ACT.A2) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A3", $AXIS_ACT.A3) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A4", $AXIS_ACT.A4) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A5", $AXIS_ACT.A5) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A6", $AXIS_ACT.A6) 
RET = EKI_Send("EkiCtrl","RobotState")
RET=EKI_Close("EkiCtrl")

ENDWHILE

RET=EKI_Clear("EkiCtrl")

END
```



调试及测试结果：

**测试Bug调试**

1. Pycharm编辑器界面显示xml下不不存在dom，对`from xml.dom import minidom`报错

2. 发送时KUKA服务端等不到连接，客户端显示Winerror 10600：

   **原因：**Defender防火墙了拦截服务端的握手信号

   **解决方案：**由于网线直接连接机械臂是设置不了专用网络的，所以先将机械臂和PC连接上交换机，然后在PC的网络设置中将以太网设置为专用网络，并关闭Windows及杀毒软件的防火墙

3. 连接成功后，发送运动指令，KUKA服务端显示指令被禁

   **原因：**注意，由于XYZABC姿态下存在多个解，所以需要明确具体是哪个解

   **解决方案：**有两种方案：第一种是在运行到指定坐标前，先运行到一个完整的PTP指令，然后使用LIN运行到指定坐标；第二种是指定POS坐标中的S状态与T转角方向使用PTP运行到指定坐标，具体见KUKA Guide编程手册2的第50页`https://www.jianguoyun.com/p/Dd6uRpsQ8PmrCBjglqED (访问密码：q5D8US)`。

4. KUKA客户端第一次连接都可以正常工作，但是当第二次连接时，PC的Client会报错，显示`[WinError 10061] 由于目标计算机积极拒绝，无法连接。`

   **解决方案：**每次连接需要使用EKI_Close来关闭连接释放端口才行！所以要将`RET=EKI_Open("EkiCtrl")`与`RET=EKI_Close("EkiCtrl")`放在连接的循环中。

5. （等待新Bug出现）

**测试结果：**

程序可以实现对机械臂的笛卡尔空间坐标下的控制，并读取机械臂当前六个关节角。



TIPS：！！！但是经过观察发现，机械臂似乎零位设置得有点问题，比如A2角-90度、A3角为90度时，机械臂小臂并不水平！！后续要精确抓取的话需要对机械臂的零位及基坐标系、工具坐标系进行重新设置。

# 2 Kinect V2深度相机方面

## 2.1 深度相机驱动配置

### 2.1.1 显卡驱动安装

​		由于之后可能会使用CUDA，而ubuntu并不自带nvidia的显卡驱动，所以先要安装显卡驱动。`笔记本显卡配置：NVIDIA GTX 1060`





### 2.1.2 Libfreenect2安装

libfreenect2具体源码包见`https://github.com/OpenKinect/libfreenect2`，该节内容引用自： [![DOI](https://camo.githubusercontent.com/4aa9cbea04982b9f965ce985b2c74dff72d99bf5/68747470733a2f2f7a656e6f646f2e6f72672f62616467652f444f492f31302e353238312f7a656e6f646f2e35303634312e737667)](https://doi.org/10.5281/zenodo.50641)，该API的具体调用见`https://openkinect.github.io/libfreenect2/`

- Download libfreenect2 source

  ```
  git clone https://github.com/OpenKinect/libfreenect2.git
  cd libfreenect2
  ```

- 安装构建工具

  ```
  sudo apt-get install build-essential cmake pkg-config
  ```

- 安装libusb（usb设备访问接口库），版本必须为> = 1.0.20

  ```
  sudo apt-get install libusb-1.0-0-dev
  ```

- 安装TurboJPEG（JPEG库）`如果遇到无法下载的问题可以试试apt-get update`

  ```
  sudo apt-get install libturbojpeg0-dev
  ```

- 安装OpenGL（开源图形库）

  ```
  sudo apt-get install libglfw3-dev
  ```

- 安装OpenCL（intel集成显卡开源并行计算库，用于后续的iai_kinect2）（可选）

  ```
  sudo apt-get install beignet-dev
  ```

- ~~安装CUDA（NVIDIA显卡的并行计算库）~~（可选）

  ```
  apt-get install cuda
  sudo prime-select intel
  ```

  直接安装了NVIDIA显卡驱动，就涵盖了CUDA
  
  TIPS:但是我在cmake中发现CUDA并不包含其中，而此时又使用的是Nidia的显卡，可能之后会存在问题。

- 安装OpenNI2（深度相机的开源库）（可选）

  ```
  sudo apt-get install libopenni2-dev
  ```

- 构建

  ```
  mkdir build && cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
  make
  make install
  ```

You need to specify `cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2` for CMake based third-party application to find libfreenect2.

- 为硬件设置udev rules

  ```
  sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
  ```

- 运行测试程序(在libfreenect2/build文件夹下打开终端并输入)

  ```
  ./bin/Protonect
  ```

  测试结果：四个方框均有图像显示，RGBD信号良好

![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-15-14-40-49-的屏幕截图-e1592205887729.png)

### 2.1.3  安装iai_kinect2——ROS kinect库

**引用自：**

```
T. Wiedemeyer, “IAI Kinect2,” https://github.com/code-iai/iai_kinect2,
Institute for Artificial Intelligence, University Bremen, 2014 – 2015,
accessed June 12, 2015.
```

**安装：**

```
 cd ~/<your_catkin_ws>/src/
 git clone https://github.com/code-iai/iai_kinect2.git
 cd iai_kinect2
 rosdep install -r --from-paths .
 cd ~/<your_catkin_ws>
 catkin_make -DCMAKE_BUILD_TYPE="Release"
```

*Note: `rosdep` will output errors on not being able to locate `[kinect2_bridge]` and `[depth_registration]`. That is fine because they are all part of the iai_kinect2 package and `rosdep` does not know these packages.*

以下错误不用在意：

```
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
kinect2_viewer: Cannot locate rosdep definition for [kinect2_bridge]
iai_kinect2: Cannot locate rosdep definition for [kinect2_registration]
kinect2_calibration: Cannot locate rosdep definition for [kinect2_bridge]
kinect2_bridge: Cannot locate rosdep definition for [kinect2_registration]
Continuing to install resolvable dependencies...
#All required rosdeps installed successfully
```



**测试：**

TIPS：终端需先在catkin_ws根目录`source devel/setup.bash`

TIPS:或者`gedit ~/.bashrc`并在最后一行加上`source /home/<your user name>/catkin_ws/devel/setup.bash`

运行测试

```
roslaunch kinect2_bridge kinect2_bridge.launch
rosrun kinect2_viewer kinect2_viewer sd cloud
```

![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-15-15-15-20-的屏幕截图-e1592205753888.png)



**使用库进行相机矫正补偿：**

具体见`https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration#calibrating-the-kinect-one`



**库发布的ROS信号以及矫正补偿文件的载入：**

具体见`https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge#first-steps`



***至此Kinect v2的驱动基本配置完成***



### 2.1.4 iai_kinect2 包的发布消息及API分析

具体Topic说明文档见

`https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge`

由于机械臂控制部分没有使用ROS而是直接TCP通讯控制，所以相机方面也打算绕过ROS来实现控制。


### 2.1.5 Libfreenect2包的API分析

API说明文档具体见`https://openkinect.github.io/libfreenect2/`



## 2.2 基于RGBD图像的目标位姿估计(6D Pose Estimation)

​	仔细看了杨学长毕设中机器视觉的部分，发现其第一个方法是将RGB和D结合起来进行图像分割，但这种方法有个缺点，当相机处于倾斜角度时就无法进行，只适用于垂直的情况。而学长第二个基于CNN的方法效果不佳，须参与第一种方法对目标位姿的投票。由于毕设搞的就是  标定参照物定位》图像分割》边缘检测》特征点识别》四边形匹配》工件位姿估计  的方案，所以这方面暂时先不重复搞了，打算尝试一下机器学习的方案。

​	谷歌知网检索一下，机器学习的位姿估计主要有模板匹配的（Linemod）、大型卷积神经网络的（感觉不太好，调参是个技术活）等等。记录一个网站：`http://rkouskou.gitlab.io/research/6D_Object.html`

**记录他人的常用方法：**

1.采用Semantic segmentation network在彩色图像上进行物体分割，然后将分割出的部分点云与物体三维模型进行ICP匹配。比`https://arxiv.org/pdf/1703.01661.pdf`，对于遮挡效果好

2.由于非遮挡情况下使用Linemod鲁棒性很好，而实际操作环境感觉非遮挡就够了，所以打算先搞Linemod试一试。



### 2.2.1 深度相机RGBD图像信息编程获取

Libfreenect2使用的为C++编写的API，则先使用C++尝试图像信息的获取。

官方API说明文档：`https://openkinect.github.io/libfreenect2/`



Frame结构下对RGBD的存储，其中Depth是512*424的，单位为mm

| Enumerator |                                                              |
| :--------- | ------------------------------------------------------------ |
| Color      | 1920x1080. BGRX or RGBX.                                     |
| Ir         | 512x424 float. Range is [0.0, 65535.0].                      |
| Depth      | 512x424 float, unit: millimeter. Non-positive, NaN, and infinity are invalid or missing data. |

通过registration的apply函数的输出bigdepth，将深度信息插补扩充到rgb的分辨率1920*1080，注意插补后像素为1920 * 1082，第一行和最后一行为空白行。

在Clion中cmakelists中加上链接libfreenect2和OpenCV的命令，***`CMakeLists`:***

```cmake
cmake_minimum_required(VERSION 3.16)
project(K2_OD)

set(CMAKE_CXX_STANDARD 14)
set(INC_DIR /home/wxj/freenect2/include)
set(LINK_DIR /home/wxj/freenect2/lib)

include_directories(${INC_DIR})
link_directories(${LINK_DIR})
link_libraries(freenect2)

find_package(OpenCV REQUIRED)

add_executable(K2_OD main.cpp)

target_link_libraries(K2_OD freenect2)
target_link_libraries(K2_OD ${OpenCV_LIBS})

```

***编写程序如下：***

```c++
#include <iostream>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

const bool enable_rgb = true;
const bool enable_depth = true;

using namespace std;
using namespace cv;
int main(){
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::PacketPipeline *pipeline = nullptr;

    // 检测是否有设备连接并列举所有设备--------------
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    string serial = freenect2.getDefaultDeviceSerialNumber();
    if (serial.empty()){
        std::cout<<"Can't get the serial."<<endl;
        return -1;
    }
    std::cout<<"Begin to connect the kinect v2 camera,The serial number is :"<<serial<<endl;
    //-----------------------------------------

    //定义处理pipeline串流的方式-----------------------------
    //pipeline = new libfreenect2::CpuPacketPipeline();//使用cpu
    //pipeline = new libfreenect2::OpenGLPacketPipeline();//使用开源图形库
    pipeline = new libfreenect2::OpenCLPacketPipeline();//使用开源并行计算库，比较快
    //---------------------------------------------------

    //打开并设置设备-------------------------------------------
    dev = freenect2.openDevice(serial, pipeline);
    if (dev == 0){
        std::cout<<"Open the device failed"<<endl;
        return -1;
    }
    int types = 0;
    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    //---------------------------------------------------

    //设备开始采集深度图像----------------------------------
    if (enable_rgb && enable_depth)
    {
        if (!dev->start()){
            std::cout<<"device can't start"<<endl;
            return -1;
        }
    }
    else
    {
        if (!dev->startStreams(enable_rgb, enable_depth)){
            std::cout<<"device can't start"<<endl;
            return -1;
        }
    }
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;


    //setIrCameraParams()//深度相机补偿
    //setColorCameraParams()//彩色相机补偿


    //Registration Setup--------------------------------
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    //depth2rgb的上下两行为空白行
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),depth2rgb(1920, 1080 + 2, 4);
    //--------------------------------------------------

    //使用OpenCV处理相机数据--------------------------------
    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

    listener.waitForNewFrame(frames);
    //获取Frame对应数据
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    //转为CV MAT格式
    cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
    //cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
    //cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
    cv::imshow("rgb", rgbmat);
    // cv::imshow("ir", irmat / 4500.0f);//灰度除以4500.0来获取较好的效果
    //cv::imshow("depth", depthmat / 4500.0f);

    //registration
    registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

    //cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
    //cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
    cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
    //cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
    //cv::imshow("registered", rgbd);
    //将深度图像映射到彩色图像，且Kinect2的测量距离为4500.0mm
    cv::imshow("depth2RGB", rgbd2 / 4500.0f);

    //---------------------------------------------------

    //结束--------------------------------------------------
    listener.release(frames);
    dev->stop();
    dev->close();
    delete registration;

    waitKey(0);
    std::cout<<"Quit..."<<endl;
    //------------------------------------------------------

    return 0;
}
```

Depth插补到1920 * 1080的测试结果：

![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-17-12-34-19-的屏幕截图-e1592369252156.png)

### 2.2.2 深度相机的红外深度图像校准

使用`setIrCameraParams()`和`setColorCameraParams()`分别进行红外相机参数补偿与RGB相机参数补偿。

(未完待续，先不自行进行校准，使用杨学长采用的校正参数)

校准可通过iai_kinect2的calibration功能包，通过可视化GUI轻松校准。

### 2.2.3  基于深度阈值分割的 5-Pose 目标姿态估计（假设目标保持垂直向上）

**基本假设：**目标平躺在水平桌面上，且机械臂可通过垂直方向来抓取目标。

**具体思路：**先使用深度信息作阈值分割，并同时对RGB图像作相应的分割；再在RGB图像基础上识别目标的二维位置和转角（共 4 Pose），并通过深度信息得到目标高度（5 Pose）。

1. ~~因为后续需要开发smart malware对ROS的message进行截获分析并伪装发送，所以算法需要包装成ROS Package的形式。首先在工作区`catkin_create_pkg`一下kuka控制端和相机处理端~~

   （还未包装成package，未完待续）

2. **深度阈值分割**

   先尝试简单的基于深度的二元阈值分割，由于扩展后的深度格式为`CV_32FC1`，使用`float`形式访问，并提取特定深度范围的彩色像素

   ```
   for(int i=0;i<1080;i++)
           for(int j=0;j<1920;j++)
               //注意深度图像为1920*1082，其中第一行和最后一行为无效行
               if(rgbd2.at<float>(i+1,j)<1000||rgbd2.at<float>(i+1,j)>1100){
                   img_color.at<Vec3b>(i,j)[0]=0;
                   img_color.at<Vec3b>(i,j)[1]=0;
                   img_color.at<Vec3b>(i,j)[2]=0;
               }
   ```

   ![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-22-15-04-57-的屏幕截图-e1592810644161.png)

   ![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-22-15-05-06-的屏幕截图-e1592810751107.png)

3. **可以通过参考圆来定位感兴趣的区域，从而减少处理量**

   通过霍夫变换识别参考圆，并提取参考圆组成的矩形区域。

   ![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-23-10-01-11-的屏幕截图-e1592877774636.png)

   如图通过参考圆将图像进行仿射变换来正畸。

4. **对矩形物体进行定位**

   - 通过灰度阈值分割和边缘检测的方式快速找到物块边缘

   ![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-23-11-00-49-的屏幕截图-e1592881389838.png)

   - 通过convexhull，基于角点检测寻找凸包，定位图中的四边形

   ![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-23-11-27-34-的屏幕截图-e1592882962126.png)

   - 获取待抓取物块的角点坐标

   ![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-23-11-33-19-的屏幕截图-e1592883255461.png)

5. **转到机械臂坐标系并对抓取姿态进行估计**

   已知物块的角点坐标，则可以计算出物块的姿态，并借助已知坐标的参考圆来估计机械臂的抓取姿态。

### 2.2.4 基于表面特征提取的目标识别

​		借助Find Object功能包进行基于表面特征提取的目标识别（如SIFT），具体见`https://github.com/introlab/find-object/tree/melodic-devel`

**安装：**

`sudo apt-get install ros-melodic-find-object-2d`		

**二维目标检测：**

（Kinect2 Topics见https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge）

```
 $ roscore &
 # Launch your preferred usb camera driver
 $ roslaunch kinect2_bridge kinect2_bridge,launch
 $ rosrun find_object_2d find_object_2d image:=/kinect2/hd/image_color
```

**结果：**

![](https://www.cyberpegasus.cn/wp-content/uploads/2020/06/2020-06-19-17-46-17-的屏幕截图-e1592560536137.png)

TIPS：只对表面纹理丰富的物体有效，无纹理的规则物体用2.2.3的方案。

# 3 协同调试

## 3.1 获取工件在机械臂坐标系下的位置

1. 执行机构朝下时的姿态角ABC

   (关于竖直方向的旋转角，角度决定抓取姿态)  A=-78.11 deg

   B=1.98 deg

   C=-172.92 deg

   测试高度Z：280.13mm

2. 获取四个参照圆在机械臂坐标系下的XYZ

   长方体区域宽度：600mm      高度：148mm

   

   左上：

   | 姿态 | 值          |
   | ---- | ----------- |
   | X    | 538.69mm    |
   | Y    | -295.65mm   |
   | Z    | 空(70mm)    |
   | A    | -109.29 deg |
   | B    | 0.47 deg    |
   | C    | -170.96 deg |

   左下：

   | 姿态 | 值        |
   | ---- | --------- |
   | X    | 656.93mm  |
   | Y    | -279.39mm |
   | Z    | 空(70mm)  |
   | A    |           |
   | B    |           |
   | C    |           |

   右上：

   | 姿态 | 值          |
   | ---- | ----------- |
   | X    | 514.66mm    |
   | Y    | 333.48mm    |
   | Z    | 空(70mm)    |
   | A    | -41.36 deg  |
   | B    | 0.48 deg    |
   | C    | -172.20 deg |

   右下：(可忽略)

   | 姿态 | 值            |
   | ---- | ------------- |
   | X    | 653mm(638.23) |
   | Y    | 316.35mm      |
   | Z    | 空(70mm)      |
   | A    | -44.46        |
   | B    | 2.03          |
   | C    | -171          |

3. 算法获取工件在机械臂坐标系下的位置

   

   ```c++
   int object_center_locate(double *center_xy,double *world_xy,double *circle_center_xy,int h,int w){
       //图像坐标系以左上圆为原点，从左向右为x正方向，从上向下为y正方向
       //double world_x_ave=0;
       //double world_y_ave=0;
       //左上角标定圆的世界坐标
       //机械臂坐标系，从左到右为y，从上到下为x；
       double circle_x=536.117;double circle_y=-296.244;
       /*
       world_x_ave+=(center_xy[0]/w)*(circle_center_xy[2]-circle_center_xy[0])+circle_center_xy[0];
       world_x_ave+=(center_xy[0]/w)*(circle_center_xy[4]-circle_center_xy[6])+circle_center_xy[6];
       world_x_ave=world_x_ave/2.0;
       world_xy[0]=world_x_ave;
   
       world_y_ave+=(center_xy[1]/h)*(circle_center_xy[7]-circle_center_xy[1])+circle_center_xy[1];
       world_y_ave+=(center_xy[1]/h)*(circle_center_xy[5]-circle_center_xy[3])+circle_center_xy[3];
       world_y_ave=world_y_ave/2.0;
       world_xy[1]=world_y_ave;
       */
       //mm为单位的图像距离h_r与w_r
       //以下为图像坐标系
       double h_r=center_xy[1]/h*148;//或者改为154.8
       double w_r=center_xy[0]/w*602;//或者改为629.7
       double sin=0.033;
       double cos=0.999;
       //以下为机械臂坐标系
       double deltay=h_r*sin+w_r*cos;
       double deltax=h_r*cos-w_r*sin;
       world_xy[0]=deltax+circle_x;
       world_xy[1]=deltay+circle_y;
   }
   ```

4.测试位置准确性

​	经过实验，机械臂基座直角坐标系下，x方向准确，y方向向左有5mm～1cm偏差，考虑后期加入补偿量。		

## 3.2 TCP通讯将工件在机械臂下的位置发给机械臂控制端

1. 注意，在工件定位时要有确认坐标位置的环节；

   相机客户端代码如下：

   ```c++
   #include <iostream>
   #include <opencv2/opencv.hpp>
   #include <unistd.h>
   #include <sys/socket.h>
   #include <netinet/in.h>
   #include <arpa/inet.h>
   #include <stdlib.h>
   #include <string.h>
   
   #include <libfreenect2/libfreenect2.hpp>
   #include <libfreenect2/frame_listener_impl.h>
   #include <libfreenect2/registration.h>
   #include <libfreenect2/packet_pipeline.h>
   #include <libfreenect2/logger.h>
   
   
   #define PORT 8888
   
   using namespace std;
   using namespace cv;
   
   const bool enable_rgb = true;
   const bool enable_depth = true;
   int center_cx[10];
   int center_cy[10];
   const int hr = 148;//仿射变换后图像高度
   const int wr = 602;//放射变换后图像宽度
   
   //图像检测定位主函数
   int camera_detect_main(double *world_xy);
   //图像检测定位相关函数
   int img_correct(cv::Mat& sourceImg,cv::Mat& showImg,cv::Mat& rgbd);
   int img_segment(cv::Mat& rgb,cv::Mat& rgbd,int h,int w,double far,double close);
   int img_canny(cv::Mat& sImg,double low_threshold,double high_threshold,double seg_threshold);
   int img_rec_find(cv::Mat& sImg,double *corner_x,double *corner_y);
   int object_center_locate(double *center_xy,double *world_xy,double *circle_center_xy,int h,int w);
   int get_circle_center_xy(double *circle_center_xy);
   //通讯主函数
   int client(double *world_xy);
   
   int main(){
       //目前运行方式为一次性运行
       //获取工件的世界坐标系
       double world_xy[2];
       camera_detect_main(world_xy);
       cout << "待抓取物块的的世界坐标为（" << world_xy[0] <<","<< world_xy[1] <<"）"<< endl;
       //确认是否进行抓取
       char c;
       std::cout<<"Please enter 1 to confirm the location, or enter 2 to quit."<<endl;
       scanf("%c",&c);
       if (c=='\n')
           client(world_xy);
       else
           std::cout<<"quit..."<<endl;
       return 0;
   }
   
   int camera_detect_main(double *world_xy){
       libfreenect2::Freenect2 freenect2;
       libfreenect2::Freenect2Device *dev = nullptr;
       libfreenect2::PacketPipeline *pipeline = nullptr;
   
       // 检测是否有设备连接并列举所有设备--------------
       if(freenect2.enumerateDevices() == 0)
       {
           std::cout << "no device connected!" << std::endl;
           return -1;
       }
       string serial = freenect2.getDefaultDeviceSerialNumber();
       if (serial.empty()){
           std::cout<<"Can't get the serial."<<endl;
           return -1;
       }
       std::cout<<"Begin to connect the kinect v2 camera,The serial number is :"<<serial<<endl;
       //-----------------------------------------
   
       //定义处理pipeline串流的方式-----------------------------
       pipeline = new libfreenect2::OpenCLPacketPipeline();//使用开源并行计算库，比较快
       //---------------------------------------------------
   
       //打开并设置设备-------------------------------------------
       dev = freenect2.openDevice(serial, pipeline);
       if (dev == 0){
           std::cout<<"Open the device failed"<<endl;
           return -1;
       }
       int types = 0;
       if (enable_rgb)
           types |= libfreenect2::Frame::Color;
       if (enable_depth)
           types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
   
       libfreenect2::SyncMultiFrameListener listener(types);
       libfreenect2::FrameMap frames;
       dev->setColorFrameListener(&listener);
       dev->setIrAndDepthFrameListener(&listener);
       //---------------------------------------------------
   
       //设备开始采集深度图像----------------------------------
       if (enable_rgb && enable_depth)
       {
           if (!dev->start()){
               std::cout<<"device can't start"<<endl;
               return -1;
           }
       }
       else
       {
           if (!dev->startStreams(enable_rgb, enable_depth)){
               std::cout<<"device can't start"<<endl;
               return -1;
           }
       }
       std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
       std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
   
   
       //Registration Setup--------------------------------
       libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
       //depth2rgb的上下两行为空白行
       libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),depth2rgb(1920, 1080 + 2, 4);
       //--------------------------------------------------
   
       ////////////////////////////////////////////////////
       //使用OpenCV处理相机数据--------------------------------
       Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
       listener.waitForNewFrame(frames);
       //获取Frame对应数据
       libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
       //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
       libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
       //转为CV MAT格式
       cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
       Mat rgb3;
       cv::cvtColor(rgbmat,rgb3,CV_BGRA2BGR);
       //水平翻转
       cv::flip(rgb3,rgb3,1);
       cv::imshow("rgb", rgb3);
       //registration
       registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
       cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
       //水平翻转
       cv::flip(rgbd2,rgbd2,1);
       cv::imshow("depth2RGB", rgbd2 / 4500.0f);
       //---------------------------------------------------
   
   
       //Image_process-------------------------------------
   
       std::cout<<"The size of the graph:"<<rgbd2.cols<<"*"<<rgbd2.rows<<endl;
       std::cout<<"Test:"<<rgbd2.at<float>(1,0)<<","<<rgbd2.at<float>(540,960)<<endl;
       Mat img_color = rgb3;
       Mat show_Img = rgb3;
   
       img_segment(show_Img,rgbd2,1080,1920,1300,1000);
       imshow("rgb segment",show_Img);
   
       Mat img_cor = show_Img;
       cvtColor(img_cor, img_cor, COLOR_BGR2GRAY);
       imshow("gray",img_cor);
       int loop=0;
       while(img_correct(img_cor,show_Img,rgbd2)){
           loop++;
           if(loop==10){
               waitKey(0);
               listener.release(frames);
               dev->stop();
               dev->close();
               delete registration;
               return 0;
           }
       }
       imshow("rgb Corrected",show_Img);
       imshow("rgbd Corrected",rgbd2 / 4500.0f);
   
   
       cvtColor(show_Img,show_Img,CV_BGR2GRAY);
       threshold(show_Img,show_Img,85,255,THRESH_BINARY);
       imshow("gray",show_Img);
   
       img_canny(show_Img,50,100,80);
       imshow("After Canny",show_Img);
   
       //locate
       double corner_x[5];//第一～四个数组元素是角点x坐标,第五个是中心x坐标
       double corner_y[5];//第一～四个数组元素是角点y坐标,第五个是中心y坐标
       if(!img_rec_find(show_Img,corner_x,corner_y))
           imshow("After Locate",show_Img);
       else{
           std::cout<<"Can't find the object, please check the img_rec_find function."<<endl;
           listener.release(frames);
           dev->stop();
           dev->close();
           delete registration;
           return 0;
       }
       for(int i=0;i<4;i++){
           cout << "待抓取物块的第" << i+1<<"个角点的坐标为（"<< corner_x[i]<<","<< corner_y[i]<<"）"<< endl;
       }
       cout << "待抓取物块的的中心坐标为（" << corner_x[4] <<","<< corner_y[4] <<"）"<< endl;
       //--------------------------------------------------
   
   
       //对抓取位姿进行估计--------------------------------------
   
       //物块中心坐标，第一个为x，第二个为y
       double center_xy[2];center_xy[0]=corner_x[4];center_xy[1]=corner_y[4];
       //xy交叉排序，存储参照圆的圆心在机械臂基座坐标系下的坐标
       double circle_center_xy[8];
       get_circle_center_xy(circle_center_xy);
       object_center_locate(center_xy,world_xy,circle_center_xy,show_Img.rows,show_Img.cols);
   
   
       //结束--------------------------------------------------
       listener.release(frames);
       dev->stop();
       dev->close();
       delete registration;
       std::cout<<"Camera Quit..."<<endl;
       //------------------------------------------------------
       return 0;
   }
   //定位参考圆，并标出
   int img_correct(cv::Mat& sourceImg,cv::Mat& showImg,cv::Mat& rgbd){
   
       vector<Vec3f> circles;
       HoughCircles(sourceImg, circles, CV_HOUGH_GRADIENT, 1, 60,90, 40, 20, 40  );
       for (size_t i = 0; i < circles.size(); i++){
           Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
           center_cx[i]=center.x;
           center_cy[i]=center.y;
           cout << "第"<<i+1<<"个圆的x坐标为" << center_cx[i] <<" ,y坐标为"<< center_cy[i]<< endl;
           int radius = cvRound(circles[i][2]);
           circle(showImg, center, 3, Scalar(155, 50, 255), -1, 8, 0);
           circle(showImg, center, radius, Scalar(155, 50, 255), 3, 8, 0);
       }
       if((center_cx[3]!=0)&&(center_cx[4]==0)){
           std::cout<<"基于参照圆的参考系建立成功"<<endl;
       }
       else{
           std::cout<<"参照圆识别错误，请更改标定参数！"<<endl;
           cvWaitKey(0);
           return 1;
       }
       //标定圆中心坐标排序
       cout<<"正在进行角点排序......"<<endl;
       int zs=0,zx=0,ys=0,yx=0,summax=center_cx[0]+center_cy[0],summin=center_cx[0]+center_cy[0];
       for(int i=0;i<4;i++){
           if(summax<(center_cx[i]+center_cy[i])){
               yx=i;
               summax=center_cx[i]+center_cy[i];
           }
           if(summin>(center_cx[i]+center_cy[i])){
               zs=i;
               summin=center_cx[i]+center_cy[i];
           }
       }
       summax=0;summin=100000;
       for(int i=0;i<4;i++){
           if(i!=zs&&i!=yx&&summax<center_cx[i]){
               ys=i;
               summax=center_cx[i];
           }
           if(i!=zs&&i!=yx&&summin>center_cx[i]){
               zx=i;
               summin=center_cx[i];
           }
       }
       cout<<"左上角坐标为("<<center_cx[zs]<<","<< center_cy[zs]<<")"<<endl;
       cout<<"左下角坐标为("<<center_cx[zx]<<","<< center_cy[zx]<<")"<<endl;
       cout<<"右下角坐标为("<<center_cx[yx]<<","<< center_cy[yx]<<")"<<endl;
       cout<<"右上角坐标为("<<center_cx[ys]<<","<< center_cy[ys]<<")"<<endl;
       //放射变换
   
       Point2f srcPoints[4], dstPoints[4];
       dstPoints[0]=Point2f(0,0);
       dstPoints[1]=Point2f(0,hr);
       dstPoints[2]=Point2f(wr,hr);
       dstPoints[3]=Point2f(wr,0);
   
       srcPoints[0]=Point2f(center_cx[zs],center_cy[zs]);
       srcPoints[1]=Point2f(center_cx[zx],center_cy[zx]);
       srcPoints[2]=Point2f(center_cx[yx],center_cy[yx]);
       srcPoints[3]=Point2f(center_cx[ys],center_cy[ys]);
   
       Mat transMat = getPerspectiveTransform(srcPoints, dstPoints);
       Mat roiImg_rgb(hr, wr, CV_8UC3);
       warpPerspective(showImg, roiImg_rgb, transMat, roiImg_rgb.size());
       Mat roiImg_rgbd(hr,wr,CV_32FC1);
       warpPerspective(rgbd, roiImg_rgbd, transMat, roiImg_rgbd.size());
   
       showImg = roiImg_rgb;
       rgbd = roiImg_rgbd;
       return 0;
   }
   //按深度进行分割
   //h为图片高度，w为图片宽度，far为远距离阈值，close为近距离阈值，函数保留从近距离到远距离的深度图像对应的Color图像部分
   int img_segment(cv::Mat& rgb,cv::Mat& rgbd,int h,int w,double far,double close){
       for(int i=0;i<h;i++)
           for(int j=0;j<w;j++)
               //注意深度图像为1920*1082，其中第一行和最后一行为无效行
               if(rgbd.at<float>(i+1,j)<close||rgbd.at<float>(i+1,j)>far){
                   rgb.at<Vec3b>(i,j)[0]=0;
                   rgb.at<Vec3b>(i,j)[1]=0;
                   rgb.at<Vec3b>(i,j)[2]=0;
               }
   }
   //canny边缘检测
   //seg_threshold为灰度阈值分割参数，low和high threshold为canny高低阈值
   int img_canny(cv::Mat& sImg,double low_threshold,double high_threshold,double seg_threshold){
   
       //threshold(sImg,sImg,seg_threshold,255,THRESH_BINARY);
       Canny(sImg, sImg, low_threshold, high_threshold,3, false);
   }
   //定位矩形工件表面
   int img_rec_find(cv::Mat& sImg,double *corner_x,double *corner_y){
       vector<vector<Point>> contours;
       Mat profile_line, polyPic;
       cv::findContours(sImg, contours, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE);
       vector<vector<Point>> polyContours(2);
   
       int maxArea = 0;
       for (unsigned int index = 0; index < contours.size(); index++) {
           if (contourArea(contours[index]) > contourArea(contours[maxArea]))
               maxArea = index;
       }
       approxPolyDP(contours[maxArea], polyContours[0], arcLength(contours[maxArea],true)*0.1, true);
   
       polyPic = Mat::zeros(sImg.size(), CV_8UC3);
       drawContours(polyPic, polyContours, 0, Scalar(0, 0, 255), 1);
   
       vector<int>  hull;
       convexHull(polyContours[0], hull);
       double obj_center_x=0;
       double obj_center_y=0;
       int hull_size=hull.size();
       if(hull_size>5)
           return -1;
       for (int i = 0; i < hull_size; ++i) {
           circle(polyPic, polyContours[0][i], 5, Scalar(0, 255, 0), 3);
           obj_center_x=obj_center_x+polyContours[0][i].x;
           obj_center_y=obj_center_y+polyContours[0][i].y;
           corner_x[i]=polyContours[0][i].x;
           corner_y[i]=polyContours[0][i].y;
           //cout << "待抓取物块的第" << i+1<<"个角点的坐标为（"<< polyContours[0][i].x<<","<< polyContours[0][i].y<<"）"<< endl;
       }
       obj_center_x=(obj_center_x)/hull_size;
       obj_center_y=(obj_center_y)/hull_size;
       corner_x[4]=obj_center_x;
       corner_y[4]=obj_center_y;
       //cout << "待抓取物块的的中心坐标为（" << obj_center_x<<","<< obj_center_y<<"）"<< endl;
       circle(polyPic, Point(obj_center_x,obj_center_y), 3, Scalar(0, 255, 0), 2);
       sImg = polyPic;
       return 0;
   }
   //对工件中心进行定位
   int object_center_locate(double *center_xy,double *world_xy,double *circle_center_xy,int h,int w){
       //图像坐标系以左上圆为原点，从左向右为x正方向，从上向下为y正方向
       //double world_x_ave=0;
       //double world_y_ave=0;
       //左上角标定圆的世界坐标
       //机械臂坐标系，从左到右为y，从上到下为x；
       double circle_x=536.117;double circle_y=-296.244;
       /*
       world_x_ave+=(center_xy[0]/w)*(circle_center_xy[2]-circle_center_xy[0])+circle_center_xy[0];
       world_x_ave+=(center_xy[0]/w)*(circle_center_xy[4]-circle_center_xy[6])+circle_center_xy[6];
       world_x_ave=world_x_ave/2.0;
       world_xy[0]=world_x_ave;
   
       world_y_ave+=(center_xy[1]/h)*(circle_center_xy[7]-circle_center_xy[1])+circle_center_xy[1];
       world_y_ave+=(center_xy[1]/h)*(circle_center_xy[5]-circle_center_xy[3])+circle_center_xy[3];
       world_y_ave=world_y_ave/2.0;
       world_xy[1]=world_y_ave;
       */
       //mm为单位的图像距离h_r与w_r
       //以下为图像坐标系
       double h_r=center_xy[1]/h*148;//或者改为154.8
       double w_r=center_xy[0]/w*602;//或者改为629.7
       double sin=0.033;
       double cos=0.999;
       //以下为机械臂坐标系
       double deltay=h_r*sin+w_r*cos;
       double deltax=h_r*cos-w_r*sin;
       world_xy[0]=deltax+circle_x;
       world_xy[1]=deltay+circle_y;
   }
   //对四个参照圆的中心坐标手动赋值
   int get_circle_center_xy(double *circle_center_xy){
   
       //实际的坐标要以机械臂工具坐标系为准
       //左上
       circle_center_xy[0]=536.117;//x
       circle_center_xy[1]=-296.224;//y
       //右上
       circle_center_xy[2]=0;
       circle_center_xy[3]=0;
       //右下
       circle_center_xy[4]=0;
       circle_center_xy[5]=0;
       //左下
       circle_center_xy[6]=0;
       circle_center_xy[7]=0;
   }
   //发起TCP客户端通讯
   int client(double *world_xy){
       double send_raw_xy[2];send_raw_xy[0]=world_xy[0];send_raw_xy[1]=world_xy[1];
       std::cout << "打开套接字" << endl;
       int socket_fd = socket(AF_INET,SOCK_STREAM,0);
       if (socket_fd == -1)
       {
           std::cout << "socket 创建失败： "<< endl;
           exit(1);
       }
       struct sockaddr_in addr;
       addr.sin_family = AF_INET;
       addr.sin_port = htons(PORT);//将一个无符号短整型的主机数值转换为网络字节顺序，即大尾顺序(big-endian)
       addr.sin_addr.s_addr = inet_addr("192.168.1.5");;//INADDR_ANY;
       bzero(&(addr.sin_zero), 8);
       int struct_len = sizeof(struct sockaddr_in);
       while (connect(socket_fd,(struct sockaddr*)&addr,struct_len)==-1);
       std::cout << "连接到客户端" << endl;
       char bufferx[10]={};
       char buffery[10]={};
       sprintf(bufferx,"%f",send_raw_xy[0]);
       sprintf(buffery,"%f",send_raw_xy[1]);
       //依次发送x和y
       write(socket_fd,bufferx,sizeof(bufferx));
       write(socket_fd,buffery,sizeof(buffery));
       char buffer_read[20]={};
       read(socket_fd,buffer_read,sizeof(buffer_read));
       std::cout<< "内容： " << buffer_read << endl;
       close(socket_fd);
   }
   ```

   

2. 机械臂控制端代码如下：

   
   
   ```python
   # -*- coding: UTF-8 -*-
   from socket import *
   
   # socket配置
   host = '192.168.1.5'
   port = 54500
   buffer_size = 10
   # 机械臂状态
   joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
   # 抓取高度，以机械臂上位机的位置为准
   h = 100.0
   
   
   def write_xml(filename, cmd, x, y, z, a, b, c):
       print("Writing Command to XML")
       doc = minidom.Document()
       # 主节点Robots
       robots = doc.createElement("Robots")
       doc.appendChild(robots)
       # 命令节点Command，一级节点
       command = doc.createElement("Command")
       # 写入Command的值
       command_value = doc.createTextNode(str(cmd))
       command.appendChild(command_value)
       robots.appendChild(command)
       # 姿态节点Pos，一级节点
       pos = doc.createElement("Pos")
       x_value = pos.setAttribute("X", str(x))
       y_value = pos.setAttribute("Y", str(y))
       z_value = pos.setAttribute("Z", str(z))
       a_value = pos.setAttribute("A", str(a))
       b_value = pos.setAttribute("B", str(b))
       c_value = pos.setAttribute("C", str(c))
       robots.appendChild(pos)
       # 从XML转为Bytes类型
       write_data = doc.toprettyxml(indent="\t", newl="\n", encoding="utf-8")
       # f = open(filename, "wb")
       # f.write()
       # f.close()
       return write_data
   
   
   def read_xml(recv_message):
       print("reading message from XML")
       doc = minidom.parseString(str(recv_message, encoding='utf8'))
       # doc = minidom.parse("test_recv.xml")
       root = doc.getElementsByTagName("Joint")[0]
       joint_state[0] = float(root.getAttribute("A1"))
       joint_state[1] = float(root.getAttribute("A2"))
       joint_state[2] = float(root.getAttribute("A3"))
       joint_state[3] = float(root.getAttribute("A4"))
       joint_state[4] = float(root.getAttribute("A5"))
       joint_state[5] = float(root.getAttribute("A6"))
       print("The current joints of robotic arm are as follow:")
       print(joint_state)
   
   
   def socket_client(ctrl_command):
       addr = (host, port)
       ctrl_client = socket(AF_INET, SOCK_STREAM)
       # 连接机械臂服务端
       print('Begin to connect...')
       ctrl_client.connect(addr)
       print('Connection Established')
       # 发送控制命令
       # send_data = bytes(ctrl_command, encoding="utf8")
       ctrl_client.send(ctrl_command)
       print('Send Successfully')
       recv_data = ctrl_client.recv(buffer_size)
       print('Receive Successfully....Begin to parse')
       read_xml(recv_data)
       ctrl_client.close()
   
   
   def to_catch(world_x, world_y):
       xyzabc = generate_xyzabc(world_x, world_y, h)
       # 写入XML格式控制命令，cmd=0🥌时结束程序
       ctrl_command = write_xml("EkiCtrl.xml", 1, xyzabc[0], xyzabc[1], xyzabc[2], xyzabc[3], xyzabc[4], xyzabc[5])
       # 发送控制命令并读取回传信息
       socket_client(ctrl_command)
   
   
   def generate_xyzabc(world_x, world_y, height):
       xyzabc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
       xyzabc[0] = world_x
       xyzabc[1] = world_y
       xyzabc[2] = height
       xyzabc[3] = -41.36
       xyzabc[4] = 0.48
       xyzabc[5] = -172.20
       return xyzabc
   
   
   def main():
       addr = (host, port)
       ctrl_server = socket(AF_INET, SOCK_STREAM)
       print('Begin to connect...')
       ctrl_server.bind(addr)
       ctrl_server.listen(2)
       print('Listening....')
       while True:
           connection_c, address_c = ctrl_server.accept()
           print('Connection Established,the ip:' + str(address_c) + 'is accepted.')
           recv_data_x = connection_c.recv(buffer_size).decode('UTF-8', 'ignore').strip().strip(b'\x00'.decode())
           print('Receive X Successfully:')
           world_x = float(recv_data_x)
           print(world_x)
           # 去除填充buffer的不可见\x00字符
           recv_data_y = connection_c.recv(buffer_size).decode('UTF-8', 'ignore').strip().strip(b'\x00'.decode())
           print('Receive Y Successfully:')
           world_y = float(recv_data_y)
           print(world_y)
           connection_c.send(b'Already Read.')
           connection_c.close()
           to_catch(world_x, world_y)
           str_cmd = input("Enter 1 to continue:")
           if str_cmd != ' 1':
               break
       ctrl_server.close()
   
   
   if __name__ == "__main__":
       main()
   
   ```
   
## 3.3 联合运行调试

### 1 问题汇总与解决

1. **夹爪控制（已解决）**

   ```
   程序规范如下：
   SIGNAL CATCHINI $OUT[10]
   SIGNAL CATCH $OUT[11]
   
   # 完成抓取放置过程
         LIN Posv
         CATCH = TRUE
         WAIT SEC 5
         PTP Catch0
         CATCH = FALSE
   ```
   
2. **机械臂零位位置不准确**

   机械臂的零位非准确的零位位姿，大概主要发生在A4关节；对于末端位姿中A、B、C姿态角的计算存在影响，会带来前后和侧向翻转方向10度左右的误差，在实际的运行过程中对分拣成功率存在挺大的影响。
   由于机械臂零位校准需要专业的工具，所以该问题无法解决。

3. **机械臂基座与抓取平台的桌面不平行**

   通过使机械臂末端中心与参照圆中心重合获取的两组数据，得到旋转变换矩阵。

### 2 最终程序

相机客户端：

```c++
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#define theta_comp 1.8911
#define PORT 8888

using namespace std;
using namespace cv;

const bool enable_rgb = true;
const bool enable_depth = true;
int center_cx[10];
int center_cy[10];
const int hr = 148;//仿射变换后图像高度
const int wr = 602;//放射变换后图像宽度

//图像检测定位主函数
int camera_detect_main(double *world_xyt);
//图像检测定位相关函数
int img_correct(cv::Mat& sourceImg,cv::Mat& showImg,cv::Mat& rgbd);
int img_segment(cv::Mat& rgb,cv::Mat& rgbd,int h,int w,double far,double close);
int img_canny(cv::Mat& sImg,double low_threshold,double high_threshold,double seg_threshold);
int img_rec_find(cv::Mat& sImg,double *corner_x,double *corner_y);
int object_center_locate(double *center_xy, double *world_xy, double *circle_center_xy, int h, int w);
int get_circle_center_xy(double *circle_center_xy);
//通讯主函数
int client(double *world_xyt);

int main(){
    //目前运行方式为一次性运行
    //获取工件的世界坐标系
    double world_xyt[3];
    camera_detect_main(world_xyt);
    cout << "待抓取物块的的世界坐标为（" << world_xyt[0] <<","<< world_xyt[1] <<"）,转动角度为:"<<world_xyt[2]<< endl;
    //确认是否进行抓取
    char c;
    std::cout<<"Please enter to confirm the location, or to quit."<<endl;
    scanf("%c",&c);
    if (c=='\n')
        client(world_xyt);
    else
        std::cout<<"quit..."<<endl;
    return 0;
}
//工件检测主程序
int camera_detect_main(double *world_xyt){
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::PacketPipeline *pipeline = nullptr;

    // 检测是否有设备连接并列举所有设备--------------
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    string serial = freenect2.getDefaultDeviceSerialNumber();
    if (serial.empty()){
        std::cout<<"Can't get the serial."<<endl;
        return -1;
    }
    std::cout<<"Begin to connect the kinect v2 camera,The serial number is :"<<serial<<endl;
    //-----------------------------------------

    //定义处理pipeline串流的方式-----------------------------
    pipeline = new libfreenect2::OpenCLPacketPipeline();//使用开源并行计算库，比较快
    //---------------------------------------------------

    //打开并设置设备-------------------------------------------
    dev = freenect2.openDevice(serial, pipeline);
    if (dev == 0){
        std::cout<<"Open the device failed"<<endl;
        return -1;
    }
    int types = 0;
    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    //---------------------------------------------------

    //设备开始采集深度图像----------------------------------
    if (enable_rgb && enable_depth)
    {
        if (!dev->start()){
            std::cout<<"device can't start"<<endl;
            return -1;
        }
    }
    else
    {
        if (!dev->startStreams(enable_rgb, enable_depth)){
            std::cout<<"device can't start"<<endl;
            return -1;
        }
    }
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;


    //Registration Setup--------------------------------
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    //depth2rgb的上下两行为空白行
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),depth2rgb(1920, 1080 + 2, 4);
    //--------------------------------------------------

    ////////////////////////////////////////////////////
    //使用OpenCV处理相机数据--------------------------------
    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
    listener.waitForNewFrame(frames);
    //获取Frame对应数据
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    //转为CV MAT格式
    cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
    Mat rgb3;
    cv::cvtColor(rgbmat,rgb3,CV_BGRA2BGR);
    //水平翻转
    cv::flip(rgb3,rgb3,1);
    cv::imshow("rgb", rgb3);
    //registration
    registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
    cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
    //水平翻转
    cv::flip(rgbd2,rgbd2,1);
    cv::imshow("depth2RGB", rgbd2 / 4500.0f);
    //---------------------------------------------------


    //Image_process-------------------------------------

    std::cout<<"The size of the graph:"<<rgbd2.cols<<"*"<<rgbd2.rows<<endl;
    std::cout<<"Test:"<<rgbd2.at<float>(1,0)<<","<<rgbd2.at<float>(540,960)<<endl;
    Mat img_color = rgb3;
    Mat show_Img = rgb3;

    img_segment(show_Img,rgbd2,1080,1920,1300,1000);
    imshow("rgb segment",show_Img);

    Mat img_cor = show_Img;
    cvtColor(img_cor, img_cor, COLOR_BGR2GRAY);
    imshow("gray",img_cor);
    int loop=0;
    while(img_correct(img_cor,show_Img,rgbd2)){
        loop++;
        if(loop==10){
            waitKey(0);
            listener.release(frames);
            dev->stop();
            dev->close();
            delete registration;
            return 0;
        }
    }
    imshow("rgb Corrected",show_Img);
    imshow("rgbd Corrected",rgbd2 / 4500.0f);


    cvtColor(show_Img,show_Img,CV_BGR2GRAY);
    threshold(show_Img,show_Img,85,255,THRESH_BINARY);
    imshow("gray",show_Img);

    img_canny(show_Img,50,100,80);
    imshow("After Canny",show_Img);

    //locate
    double corner_x[5];//第一～四个数组元素是角点x坐标,第五个是中心x坐标
    double corner_y[5];//第一～四个数组元素是角点y坐标,第五个是中心y坐标
    if(!img_rec_find(show_Img,corner_x,corner_y))
        imshow("After Locate",show_Img);
    else{
        std::cout<<"Can't find the object, please check the img_rec_find function."<<endl;
        listener.release(frames);
        dev->stop();
        dev->close();
        delete registration;
        return 0;
    }
    for(int i=0;i<4;i++){
        cout << "待抓取物块的第" << i+1<<"个角点的坐标为（"<< corner_x[i]<<","<< corner_y[i]<<"）"<< endl;
    }
    cout << "待抓取物块的的中心坐标为（" << corner_x[4] <<","<< corner_y[4] <<"）"<< endl;
    //--------------------------------------------------


    //对抓取位姿进行估计-------------------------------------
    double catch_theta = 0.0;
    double x_left = corner_x[0];
    double x_right = corner_x[0];
    int most_left;
    int most_right;
    for (int i = 0; i < 4; ++i) {
        if (x_left>=corner_x[i])
            most_left=i;
        if (x_right<=corner_x[i])
            most_right=i;
    }
    //对角线与图像坐标系的夹角
    double tan_theta = (corner_y[most_left]-corner_y[most_right])/(corner_x[most_left]-corner_x[most_right]);
    catch_theta = atan(tan_theta);
    //两个对边垂线与图像坐标系的夹角（理想的夹持角度）
    catch_theta += 45;
    //加上图像坐标系顺势针旋转1.89度得到机械臂基座坐标系的影响,逆时针还原，对于机械臂A6角是加上该角度
    catch_theta += theta_comp;
    //换算出实际A6角度，因为A6垂直方向是-170度，水平方向是-80度
    catch_theta -= 80;
    //考虑到夹爪导线的缠绕影响，获取实际的夹持角度
    while (catch_theta < -170)
        catch_theta+=90;
    while (catch_theta > -80)
        catch_theta-=90;
    //得到最终角度
    world_xyt[2]=catch_theta;

    // ----------------------------------------------------

    //物块中心坐标，第一个为x，第二个为y
    double center_xy[2];center_xy[0]=corner_x[4];center_xy[1]=corner_y[4];
    //xy交叉排序，存储参照圆的圆心在机械臂基座坐标系下的坐标
    double circle_center_xy[8];
    get_circle_center_xy(circle_center_xy);
    double world_xy[2];
    object_center_locate(center_xy, world_xy, circle_center_xy, show_Img.rows, show_Img.cols);
    world_xyt[0]=world_xy[0];
    world_xyt[1]=world_xy[1];

    //结束--------------------------------------------------
    listener.release(frames);
    dev->stop();
    dev->close();
    delete registration;
    std::cout<<"Camera Quit..."<<endl;
    //------------------------------------------------------
    return 0;
}
//定位参考圆，并标出
int img_correct(cv::Mat& sourceImg,cv::Mat& showImg,cv::Mat& rgbd){

    vector<Vec3f> circles;
    HoughCircles(sourceImg, circles, CV_HOUGH_GRADIENT, 1, 60,90, 40, 20, 40  );
    for (size_t i = 0; i < circles.size(); i++){
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        center_cx[i]=center.x;
        center_cy[i]=center.y;
        cout << "第"<<i+1<<"个圆的x坐标为" << center_cx[i] <<" ,y坐标为"<< center_cy[i]<< endl;
        int radius = cvRound(circles[i][2]);
        circle(showImg, center, 3, Scalar(155, 50, 255), -1, 8, 0);
        circle(showImg, center, radius, Scalar(155, 50, 255), 3, 8, 0);
    }
    imshow("Circle",showImg);
    if((center_cx[3]!=0)&&(center_cx[4]==0)){
        std::cout<<"基于参照圆的参考系建立成功"<<endl;
    }
    else{
        std::cout<<"参照圆识别错误，请更改标定参数！"<<endl;
        cvWaitKey(0);
        return 1;
    }
    //标定圆中心坐标排序
    cout<<"正在进行角点排序......"<<endl;
    int zs=0,zx=0,ys=0,yx=0,summax=center_cx[0]+center_cy[0],summin=center_cx[0]+center_cy[0];
    for(int i=0;i<4;i++){
        if(summax<(center_cx[i]+center_cy[i])){
            yx=i;
            summax=center_cx[i]+center_cy[i];
        }
        if(summin>(center_cx[i]+center_cy[i])){
            zs=i;
            summin=center_cx[i]+center_cy[i];
        }
    }
    summax=0;summin=100000;
    for(int i=0;i<4;i++){
        if(i!=zs&&i!=yx&&summax<center_cx[i]){
            ys=i;
            summax=center_cx[i];
        }
        if(i!=zs&&i!=yx&&summin>center_cx[i]){
            zx=i;
            summin=center_cx[i];
        }
    }
    cout<<"左上角坐标为("<<center_cx[zs]<<","<< center_cy[zs]<<")"<<endl;
    cout<<"左下角坐标为("<<center_cx[zx]<<","<< center_cy[zx]<<")"<<endl;
    cout<<"右下角坐标为("<<center_cx[yx]<<","<< center_cy[yx]<<")"<<endl;
    cout<<"右上角坐标为("<<center_cx[ys]<<","<< center_cy[ys]<<")"<<endl;
    //放射变换

    Point2f srcPoints[4], dstPoints[4];
    dstPoints[0]=Point2f(0,0);
    dstPoints[1]=Point2f(0,hr);
    dstPoints[2]=Point2f(wr,hr);
    dstPoints[3]=Point2f(wr,0);

    srcPoints[0]=Point2f(center_cx[zs],center_cy[zs]);
    srcPoints[1]=Point2f(center_cx[zx],center_cy[zx]);
    srcPoints[2]=Point2f(center_cx[yx],center_cy[yx]);
    srcPoints[3]=Point2f(center_cx[ys],center_cy[ys]);

    Mat transMat = getPerspectiveTransform(srcPoints, dstPoints);
    Mat roiImg_rgb(hr, wr, CV_8UC3);
    warpPerspective(showImg, roiImg_rgb, transMat, roiImg_rgb.size());
    Mat roiImg_rgbd(hr,wr,CV_32FC1);
    warpPerspective(rgbd, roiImg_rgbd, transMat, roiImg_rgbd.size());

    showImg = roiImg_rgb;
    rgbd = roiImg_rgbd;
    return 0;
}
//按深度进行分割
//h为图片高度，w为图片宽度，far为远距离阈值，close为近距离阈值，函数保留从近距离到远距离的深度图像对应的Color图像部分
int img_segment(cv::Mat& rgb,cv::Mat& rgbd,int h,int w,double far,double close){
    for(int i=0;i<h;i++)
        for(int j=0;j<w;j++)
            //注意深度图像为1920*1082，其中第一行和最后一行为无效行
            if(rgbd.at<float>(i+1,j)<close||rgbd.at<float>(i+1,j)>far){
                rgb.at<Vec3b>(i,j)[0]=0;
                rgb.at<Vec3b>(i,j)[1]=0;
                rgb.at<Vec3b>(i,j)[2]=0;
            }
}
//canny边缘检测
//seg_threshold为灰度阈值分割参数，low和high threshold为canny高低阈值
int img_canny(cv::Mat& sImg,double low_threshold,double high_threshold,double seg_threshold){

    //threshold(sImg,sImg,seg_threshold,255,THRESH_BINARY);
    Canny(sImg, sImg, low_threshold, high_threshold,3, false);
}
//定位矩形工件表面
int img_rec_find(cv::Mat& sImg,double *corner_x,double *corner_y){
    vector<vector<Point>> contours;
    Mat profile_line, polyPic;
    cv::findContours(sImg, contours, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    vector<vector<Point>> polyContours(2);

    int maxArea = 0;
    for (unsigned int index = 0; index < contours.size(); index++) {
        if (contourArea(contours[index]) > contourArea(contours[maxArea]))
            maxArea = index;
    }
    approxPolyDP(contours[maxArea], polyContours[0], arcLength(contours[maxArea],true)*0.1, true);

    polyPic = Mat::zeros(sImg.size(), CV_8UC3);
    drawContours(polyPic, polyContours, 0, Scalar(0, 0, 255), 1);

    vector<int>  hull;
    convexHull(polyContours[0], hull);
    double obj_center_x=0;
    double obj_center_y=0;
    int hull_size=hull.size();
    if(hull_size>5)
        return -1;
    for (int i = 0; i < hull_size; ++i) {
        circle(polyPic, polyContours[0][i], 5, Scalar(0, 255, 0), 3);
        obj_center_x=obj_center_x+polyContours[0][i].x;
        obj_center_y=obj_center_y+polyContours[0][i].y;
        corner_x[i]=polyContours[0][i].x;
        corner_y[i]=polyContours[0][i].y;
        //cout << "待抓取物块的第" << i+1<<"个角点的坐标为（"<< polyContours[0][i].x<<","<< polyContours[0][i].y<<"）"<< endl;
    }
    obj_center_x=(obj_center_x)/hull_size;
    obj_center_y=(obj_center_y)/hull_size;
    corner_x[4]=obj_center_x;
    corner_y[4]=obj_center_y;
    //cout << "待抓取物块的的中心坐标为（" << obj_center_x<<","<< obj_center_y<<"）"<< endl;
    circle(polyPic, Point(obj_center_x,obj_center_y), 3, Scalar(0, 255, 0), 2);
    sImg = polyPic;
    return 0;
}
//对工件中心进行定位
int object_center_locate(double *center_xy, double *world_xy, double *circle_center_xy, int h, int w){
    //图像坐标系以左上圆为原点，从左向右为x正方向，从上向下为y正方向
    //double world_x_ave=0;
    //double world_y_ave=0;
    //左上角标定圆的世界坐标
    //机械臂坐标系，从左到右为y，从上到下为x；
    double circle_x=536.117;double circle_y=-296.244;
    /*
    world_x_ave+=(center_xy[0]/w)*(circle_center_xy[2]-circle_center_xy[0])+circle_center_xy[0];
    world_x_ave+=(center_xy[0]/w)*(circle_center_xy[4]-circle_center_xy[6])+circle_center_xy[6];
    world_x_ave=world_x_ave/2.0;
    world_xyt[0]=world_x_ave;

    world_y_ave+=(center_xy[1]/h)*(circle_center_xy[7]-circle_center_xy[1])+circle_center_xy[1];
    world_y_ave+=(center_xy[1]/h)*(circle_center_xy[5]-circle_center_xy[3])+circle_center_xy[3];
    world_y_ave=world_y_ave/2.0;
    world_xyt[1]=world_y_ave;
    */
    //mm为单位的图像距离h_r与w_r
    //以下为图像坐标系
    double h_r=center_xy[1]/h*148;//或者改为154.8
    double w_r=center_xy[0]/w*602;//或者改为629.7
    double sin=0.033;
    double cos=0.999;
    //以下为机械臂坐标系
    double deltay=h_r*sin+w_r*cos;
    double deltax=h_r*cos-w_r*sin;
    world_xy[0]= deltax + circle_x;
    world_xy[1]= deltay + circle_y;
}
//对四个参照圆的中心坐标手动赋值
int get_circle_center_xy(double *circle_center_xy){

    //实际的坐标要以机械臂工具坐标系为准
    //左上
    circle_center_xy[0]=536.117;//x
    circle_center_xy[1]=-296.224;//y
    //右上
    circle_center_xy[2]=0;
    circle_center_xy[3]=0;
    //右下
    circle_center_xy[4]=0;
    circle_center_xy[5]=0;
    //左下
    circle_center_xy[6]=0;
    circle_center_xy[7]=0;
}
//发起TCP客户端通讯
int client(double *world_xyt){
    double send_raw_xyt[2];send_raw_xyt[0]=world_xyt[0];send_raw_xyt[1]=world_xyt[1];send_raw_xyt[2]=world_xyt[2];
    std::cout << "打开套接字" << endl;
    int socket_fd = socket(AF_INET,SOCK_STREAM,0);
    if (socket_fd == -1)
    {
        std::cout << "socket 创建失败： "<< endl;
        exit(1);
    }
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);//将一个无符号短整型的主机数值转换为网络字节顺序，即大尾顺序(big-endian)
    addr.sin_addr.s_addr = inet_addr("192.168.1.5");;//INADDR_ANY;
    bzero(&(addr.sin_zero), 8);
    int struct_len = sizeof(struct sockaddr_in);
    std::cout << "正在连接客户端...." << endl;
    while (connect(socket_fd,(struct sockaddr*)&addr,struct_len)==-1);
    std::cout << "连接到客户端" << endl;
    char bufferx[10]={};
    char buffery[10]={};
    char buffert[10]={};
    sprintf(bufferx, "%f", send_raw_xyt[0]);
    sprintf(buffery, "%f", send_raw_xyt[1]);
    sprintf(buffert, "%f", send_raw_xyt[2]);
    //依次发送x和y
    write(socket_fd,bufferx,sizeof(bufferx));
    write(socket_fd,buffery,sizeof(buffery));
    write(socket_fd,buffert,sizeof(buffert));
    char buffer_read[20]={};
    read(socket_fd,buffer_read,sizeof(buffer_read));
    std::cout<< "内容： " << buffer_read << endl;
    close(socket_fd);
}
```



机械臂控制服务&客户端：

```python
# -*- coding: UTF-8 -*-
from socket import *
from xml.dom import minidom
# socket配置
host = '192.168.1.5'
host_kuka = '192.168.1.234'
port_s = 8888
port = 54600
buffer_size = 10
buffer_size_c = 1024
# 机械臂状态
joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 抓取高度，以机械臂上位机的位置为准
h = 265.0


def write_xml(filename, cmd, x, y, z, a, b, c):
    print("Writing Command to XML")
    doc = minidom.Document()
    # 主节点Robots
    robots = doc.createElement("Robots")
    doc.appendChild(robots)
    # 命令节点Command，一级节点
    command = doc.createElement("Command")
    # 写入Command的值
    command_value = doc.createTextNode(str(cmd))
    command.appendChild(command_value)
    robots.appendChild(command)
    # 姿态节点Pos，一级节点
    pos = doc.createElement("Pos")
    x_value = pos.setAttribute("X", str(x))
    y_value = pos.setAttribute("Y", str(y))
    z_value = pos.setAttribute("Z", str(z))
    a_value = pos.setAttribute("A", str(a))
    b_value = pos.setAttribute("B", str(b))
    c_value = pos.setAttribute("C", str(c))
    robots.appendChild(pos)
    # 从XML转为Bytes类型
    write_data = doc.toprettyxml(indent="\t", newl="\n", encoding="utf-8")
    # f = open(filename, "wb")
    # f.write()
    # f.close()
    return write_data


def read_xml(recv_message):
    print("reading message from XML")
    doc = minidom.parseString(str(recv_message, encoding='utf8'))
    # doc = minidom.parse("test_recv.xml")
    root = doc.getElementsByTagName("Joint")[0]
    joint_state[0] = float(root.getAttribute("A1"))
    joint_state[1] = float(root.getAttribute("A2"))
    joint_state[2] = float(root.getAttribute("A3"))
    joint_state[3] = float(root.getAttribute("A4"))
    joint_state[4] = float(root.getAttribute("A5"))
    joint_state[5] = float(root.getAttribute("A6"))
    print("The current joints of robotic arm are as follow:")
    print(joint_state)


def socket_client(ctrl_command):
    addr_c = (host_kuka, port)
    ctrl_client = socket(AF_INET, SOCK_STREAM)
    # 连接机械臂服务端
    print('Begin to connect...')
    ctrl_client.connect(addr_c)
    print('Connection Established')
    print(ctrl_command)
    # 发送控制命令
    # send_data = bytes(ctrl_command, encoding="utf8")
    ctrl_client.send(ctrl_command)
    print('Send Successfully')
    recv_data = ctrl_client.recv(buffer_size_c)
    print('Receive Successfully....Begin to parse')
    read_xml(recv_data)
    ctrl_client.close()


def to_catch(world_x, world_y, world_theta):
    xyzabc = generate_xyzabc(world_x, world_y, world_theta, h)
    # 写入XML格式控制命令，cmd=0🥌时结束程序
    ctrl_command = write_xml("EkiCtrl.xml", 1, xyzabc[0], xyzabc[1], xyzabc[2], xyzabc[3], xyzabc[4], xyzabc[5])
    # 发送控制命令并读取回传信息
    socket_client(ctrl_command)


def generate_xyzabc(world_x, world_y, world_theta, height):
    xyzabc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    xyzabc[0] = world_x
    xyzabc[1] = world_y
    xyzabc[2] = height
    xyzabc[3] = world_theta  # 从-80度到-170度变化
    xyzabc[4] = -2.0
    xyzabc[5] = -172.0
    return xyzabc


def main():
    addr = (host, port_s)
    ctrl_server = socket(AF_INET, SOCK_STREAM)
    print('Begin to connect...')
    ctrl_server.bind(addr)
    ctrl_server.listen(2)
    print('Listening....')
    while True:
        connection_c, address_c = ctrl_server.accept()
        print('Connection Established,the ip:' + str(address_c) + 'is accepted.')
        recv_data_x = connection_c.recv(buffer_size).decode('UTF-8', 'ignore').strip().strip(b'\x00'.decode())
        print('Receive X Successfully:')
        world_x = float(recv_data_x)
        print(world_x)
        # 去除填充buffer的不可见\x00字符
        recv_data_y = connection_c.recv(buffer_size).decode('UTF-8', 'ignore').strip().strip(b'\x00'.decode())
        print('Receive Y Successfully:')
        world_y = float(recv_data_y)
        print(world_y)
        recv_data_theta = connection_c.recv(buffer_size).decode('UTF-8', 'ignore').strip().strip(b'\x00'.decode())
        print('Receive theta Successfully:')
        world_theta = float(recv_data_theta)
        print(world_theta)
        connection_c.send(b'Already Read.')
        connection_c.close()
        to_catch(world_x, world_y, world_theta)
        str_cmd = input("Enter 1 to continue:")
        if str_cmd != '1':
            break
    ctrl_server.close()


if __name__ == "__main__":
    main()
```



机械臂通讯格式：

```xml
<ETHERNETKRL>
<CONFIGURATION>
<EXTERNAL>
<TYPE>Client</TYPE>
</EXTERNAL>
<INTERNAL>
<ENVIRONMENT>Program</ENVIRONMENT>
<BUFFERING Limit="512"/>
<ALIVE Set_Flag="1"/>
<IP>192.168.1.234</IP>
<PORT>54600</PORT>
<PROTOCOL>TCP</PROTOCOL>
</INTERNAL>
</CONFIGURATION>
<RECEIVE>
<XML>
<ELEMENT Tag="Robots/Command" Type="INT"/>
<ELEMENT Tag="Robots/Pos/@X" Type="REAL"/>
<ELEMENT Tag="Robots/Pos/@Y" Type="REAL"/>
<ELEMENT Tag="Robots/Pos/@Z" Type="REAL"/>
<ELEMENT Tag="Robots/Pos/@A" Type="REAL"/>
<ELEMENT Tag="Robots/Pos/@B" Type="REAL"/>
<ELEMENT Tag="Robots/Pos/@C" Type="REAL"/>
</XML>
</RECEIVE>
<SEND>
<XML>
<ELEMENT Tag="RobotState/Joint/@A1" Type="REAL"/>
<ELEMENT Tag="RobotState/Joint/@A2" Type="REAL"/>
<ELEMENT Tag="RobotState/Joint/@A3" Type="REAL"/>
<ELEMENT Tag="RobotState/Joint/@A4" Type="REAL"/>
<ELEMENT Tag="RobotState/Joint/@A5" Type="REAL"/>
<ELEMENT Tag="RobotState/Joint/@A6" Type="REAL"/>
</XML>
</SEND>
</ETHERNETKRL>
```



机械臂控制柜服务端：

```
&ACCESS RVP
&REL 14
DEF EkiCtrlPo()
DECL POS Posv
DECL POS Cent
DECL EKI_STATUS RET
DECL INT F
DECL INT RobStatus
SIGNAL CATCHINI $OUT[10]
SIGNAL CATCH $OUT[11]
EXT BAS (BAS_COMMAND:IN,REAL:IN)
DECL AXIS HOME
DECL AXIS Catch0
BAS(#INITMOV,0)
HOME={AXIS:A1 0,A2 -90,A3 90,A4 0,A5 0,A6 0}
Catch0={AXIS:A1 46.95,A2 -77.32,A3 124.25,A4 0.08,A5 31.65,A6 56.56}
PTP HOME
PTP Catch0
CATCHINI = TRUE
$BASE=BASE_DATA[1]
$TOOL=TOOL_DATA[1]

Posv = {X 0.0, Y 0.0, Z 0.0, A 0.0, B 0.0, C 0.0}
Cent = {X 616.69, Y 29.13, Z 380.26, A -123.67, B 6.67, C -177.89}
Posv = $POS_ACT
F =0
$FLAG[1] = FALSE
RobStatus = 0

RET=EKI_Init("EkiCtrl")



WHILE (F <> 1)
RET=EKI_Open("EkiCtrl")
EKI_CHECK(RET,#QUIT)

WAIT FOR $FLAG[1]
$FLAG[1] = FALSE

RET = EKI_GetInt("EkiCtrl", "Robots/Command",RobStatus)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@X", Posv.X)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@Y", Posv.Y)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@Z", Posv.Z)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@A", Posv.A)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@B", Posv.B)
RET = EKI_GetReal("EkiCtrl", "Robots/Pos/@C", Posv.C)


SWITCH RobStatus
	CASE 1
      LIN Cent
      LIN Posv
      CATCH = TRUE
      WAIT SEC 5
      PTP Catch0
      CATCH = FALSE


   CASE 2
		F = 1
ENDSWITCH

RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A1", $AXIS_ACT.A1) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A2", $AXIS_ACT.A2) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A3", $AXIS_ACT.A3) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A4", $AXIS_ACT.A4) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A5", $AXIS_ACT.A5) 
RET=EKI_SetReal("EkiCtrl","RobotState/Joint/@A6", $AXIS_ACT.A6) 
RET = EKI_Send("EkiCtrl","RobotState")
RET=EKI_Close("EkiCtrl")

ENDWHILE

RET=EKI_Clear("EkiCtrl")

END
```

### 3 抓取实验

​		由于机械臂零点位置偏移的影响，xyz方向的抓取精度满足要求，但方向角的精度不高。故抓取成功率不高。

