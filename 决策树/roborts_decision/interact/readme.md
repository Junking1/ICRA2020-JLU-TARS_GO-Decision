# 信息交互

## [MutualBoard](./mutualboard.h) 基本框架

- blackboard中设置有记录自身状态和同伴状态的变量

- MutualBoard 对象从blackboard中获取自身信息，通过套接字发送到队友，同时，从套接字接受队友传来的信息， 并将其发布和存储到blackboard中

### 使用方法
- 声明mutualboard对象，执行$ExchangeData$函数
- 主节点中运行流程：
  - 先依次建立blackboard，goal_factory, mutualboard
  - 运行行为树

### 将要交互的数据  
  [*点此查看收发数据的结构*](../proto/decision.proto)

1. 血量
2. 弹量
3. 位置
4. 热量
5. 是否受到惩罚
    - 云台
    - 底盘
6. 目前运行状态
    - 是否运行中
    - 运行中的具体状态
        - 追逐中(若是，则提供追击目标)
        - 逃跑中(若是，则提供逃跑的目的地)
        - 射击中
        - 正要去取buff(若是，则提供目标buff)

## [pos_reciever](./pos_reciever.cpp)

- 通过套接字从场外服务器接受哨岗识别的结果
- 将接收到的信息发布到指定话题

### 使用方法
修改哨岗服务的ip和端口，重新编译功能包，执行节点 pos_reciver

若连接失败或者传输出错将进行三次重连，若持续失败则退出节点并立即自动重启节点
