## 软件功能介绍
- 此文件夹(roborts_decision)包含决策部分所有源码.
- 运行测试节点(behavior_test_node),可查看机器人单个动作执行情况.
- 运行决策节点(icra_decision_node),机器人可根据实时条件分析执行相应动作.

## 软件效果展示。需要体现整体项目的创新性和优势,结合可视化数据(例如视频网址,gif 图,测试图表等)对最终效果进行展示与定量分析

### 依赖工具、软硬件环境

- 硬件： 秒算
- 操作系统：Ubuntu-16.04
- ROS: ros-kinetic
- ProtoBuff-2.6
- Socket

### 编译、安装方式

- 使用ros提供的catkin_make进行全局编译  

- 单独编译本功能包：catkin_make --pkg roborts_decision  

### 行为测试

[测试文件](./behavior_test.cpp)中包含各种行为的测试，运行该节点，可以对每个行为进行调试演示。当其他程序运行起来的时候，执行以下节点进行调试：   
*执行指令* ： rosrun roborts_decision behavior_test_node

## 文件目录结构及文件用途说明

### 目录结构说明

    ├── pictures
    │   ├── table1_2.jpg
    │   ├── table1.jpg
    │   ├── table2.jpg
    │   ├── table3.jpg
    │   └── tree.jpg
    ├── README.md
    ├── roborts_decision
    │   ├── action_node
    │   │   ├── BackAwayAction.h
    │   │   ├── BackBootArea.h
    │   │   ├── ChaseAction.h
    │   │   ├── ChassisLimited.h
    │   │   ├── DeckChaseAction.h
    │   │   ├── DefendAction.h
    │   │   ├── EscapeAction.h
    │   │   ├── FollowAction.h
    │   │   ├── FrozeAction.h
    │   │   ├── GainBloodAction.h
    │   │   ├── GainBulletAction.h
    │   │   ├── GimbalLimited.h
    │   │   ├── readme.md
    │   │   ├── SearchAction.h
    │   │   ├── TurnToDetectedDirection.h
    │   │   └── WaitBuffRefresh.h
    │   ├── behavior_test.cpp
    │   ├── blackboard
    │   │   ├── blackboard.h
    │   │   └── topic_name.h
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   ├── buffposition.yaml
    │   │   ├── decision.prototxt
    │   │   ├── decision.yaml
    │   │   └── topic_name.prototxt
    │   ├── executor
    │   │   ├── chassis_executor.cpp
    │   │   ├── chassis_executor.h
    │   │   ├── gimbal_executor.cpp
    │   │   └── gimbal_executor.h
    │   ├── icra_decision_node.cpp
    │   ├── include
    │   │   ├── goal_factory.h
    │   │   └── redme.md
    │   ├── interact
    │   │   ├── mutualboard.h
    │   │   ├── pos_reciever.cpp
    │   │   └── readme.md
    │   └── package.xml
    └── roborts_msgs
        ├── action
        │   ├── ArmorDetection.action
        │   ├── GlobalPlanner.action
        │   └── LocalPlanner.action
        ├── CMakeLists.txt
        ├── msg
        │   ├── GimbalActionlib.msg
        │   ├── GimbalAngle.msg
        │   ├── GimbalControl.msg
        │   ├── GimbalRate.msg
        │   ├── ObstacleMsg.msg
        │   ├── PunishInfo.msg
        │   ├── PyArmorInfo.msg
        │   ├── referee_system
        │   │   ├── BonusStatus.msg
        │   │   ├── GameResult.msg
        │   │   ├── GameStatus.msg
        │   │   ├── GameSurvivor.msg
        │   │   ├── ProjectileSupply.msg
        │   │   ├── RobotBonus.msg
        │   │   ├── RobotDamage.msg
        │   │   ├── RobotHeat.msg
        │   │   ├── RobotShoot.msg
        │   │   ├── RobotStatus.msg
        │   │   └── SupplierStatus.msg
        │   ├── ResizedImage.msg
        │   ├── RobotInfo.msg
        │   ├── SentryInfo.msg
        │   ├── ShootInfo.msg
        │   ├── ShootState.msg
        │   ├── TreeStatus.msg
        │   └── TwistAccel.msg
        ├── package.xml
        └── srv
            ├── FricWhl.srv
            ├── GimbalMode.srv
            ├── ShootCmd.srv
            └── SwingDefend.srv


### 文件用途说明

- [action_node](./action_node) 包含机器人可执行的各种动作行为, 包括一系列攻击行为、防御行为、加成和惩罚响应行为和准备行为  

- [blackboard](./blackboard)  黑板类，负责收集、处理和分发场上可获得的各种信息。包括： 各机器人的位置、各机器人自身的健康参数和场上各类决策时需要用到的参数等

- [interact](./interact)  负责两级交互的模快，使用套接字向双同伴机器人发送自身的相关信息并且与场外哨岗通信获取哨岗识别的结果  

- [include](./include) 包含行为树必需的头文件，这里主要放了goal_factory.h  

- [behavior_test.cpp](./behavior_test.cpp)  这个文件是测试各行为运行情况的节点，通过它可以单独执行指定的节点，观察运行情况  

- [icra_decision_node.cpp](./icra_decision_node.cpp) 决策节点，其中负责搭建好行为树并且将其运行起来，实施决策


## 软件与硬件的系统框图,数据流图

- 数据流图：

![image](https://github.com/cxx-bobo/decision/blob/master/pictures/DataFlow2.png)


## 原理介绍与理论支持分析

- 行为树原理介绍：
    行为树是一种基于规则的决策方法，它通过类似于决策树的树形决策结构来选择当前环境下应该做出的具体行为。
    行为树使状态高度模块化，减少其转移条件，以轮询机制实现复杂决策。
    行为树的优点如下：
    1.行为逻辑和状态数据分离，有良好的层次性和扩展性。
    2.多样的组合节点为行为树提供多样的流程控制方法。
    3.有良好的复用性，任何节点写好以后可以反复利用。
- 通过对场上环境进行分析分类，根据每一种具体的条件，设计并且实现了对应的执行动作。使用行为树，使得决策的流程直观明了，实现起来思路清晰，维护和修改方便，能够随时发现问题并作出修正。行为树通过类似于决策树的树形决策结构来选择当前环境下应该做出的具体行为，与状态机不同，它是一种“轮询式机制”，其优点为：  
    1. 行为逻辑和状态数据分离，有良好的层次性和扩展性；
    2. 多样的组合节点为行为树提供了多样的流程控制方法；
    3. 有良好的复用性，任何节点写好以后可以反复利用。  

- 根据比赛规则，我们对场上探测到的状态进行了细致的分类，并且针对每一种条件的具体情况设计了对应的响应行为，包括但不限于搜寻敌人、追击、逃跑等行为。同时，为了应对随时可能遇到的加成和奖励情况，我们专门为之设计了相应的节点。综合来看，已经足够应对场上会发生的几乎所有情况。
所有节点，根据不同的条件，将会执行相应的动作，因此可通过行为树实现比赛中的机器人自主决策。

### 行为树结构层次图：
![image](https://github.com/cxx-bobo/decision/blob/master/pictures/tree.jpg)  

**动作节点介绍：**
![image](https://github.com/cxx-bobo/decision/blob/master/pictures/table1.jpg)

**选择节点介绍：**
![image](https://github.com/cxx-bobo/decision/blob/master/pictures/table2.jpg)

**条件节点介绍**
![image](https://github.com/cxx-bobo/decision/blob/master/pictures/table3.jpg)

### 两机配合工作
 
- 我们使用Wi-Fi相互传输两机信息，在比赛中AI车能够实时的获取队友剩余血量、剩余子弹数、是否遭受攻击、是否受罚等重要信息。结合这些信息，分析是否处于需要两机合力击败敌方或掩护队友逃跑等情况，从而作出更好的决策。

### 自主搜索

- 搜索的任务主要由场外的哨岗来完成。目前哨岗与决策同样采用Wi-Fi通信。由比赛场地两角的相机分析场上的情况，计算出目前敌方在世界地图的坐标,然后通过Wi-Fi路由器将识别结果传送给场上的机器人，机器人将会整合来自哨岗的敌人坐标来做出决策。传送信息过程中，良好的检测机使得双方都能够及时发现传输过程的异常情况，及时采取应对方案。
- 位于云台顶部的相机能够准确地识别视野中的机器人，当本地计算机与哨岗的场外服务器通信出现异常时，机器人将在场地中进行巡逻搜索, 由相机来完成寻找敌人的任务。巡逻时，我们将整个场地划分为若干区域，机器人进行巡逻时,按照既定路线,逐个区域进行搜寻，同时相机自动来回旋转以检查足够大的视野，直到搜索到敌方机器人为止。

## 决策整体采用黑板模式  

- 黑板与游戏设计中黑板（Blackboard）的概念相似，作为当前决策系统中观察（Observation）的输入，用于调度一系列感知任务并获取感知信息。[黑板模块](./blackboard/readme.md)收集并存储各个节点观察到的信息，分发到需要的节点。  

## 是否存在开源协议 --------------------------------------？  

## ~~函数、变量命名是否规范、统一------~~ 

## ~~整体可编译可运行,容易测试,包含不同情况下的测试脚本或涵盖集成测试~~

## ~~开源与分享,截止到评选日的开源影响力(star 数)------~~

## ~~关键功能函数、变量是否存在清晰的注释~~
