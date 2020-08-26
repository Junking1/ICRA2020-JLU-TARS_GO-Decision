# Action_node

- BackBootArea  
    回到启动区，游戏结束或者其他必要条件时回到启动区
- BackAwayAction  
    后退，与敌人距离过近时执行
- ChaseAction  
    追击敌人
- ChassisLimited  
    底盘受到惩罚，则此时旋转云台进行扫描
- DeckChaseAction  
    以敌人后侧装甲板为目标方向追击敌人
- DefendAction  
    摆动防御
- EscapeAction  
    逃跑节点（趋向逃离目标并且在途中检测到敌人的时候采取适当攻击或者更改逃跑路线）
- FollowAction  
    跟随我方另一台车，以便合作
- FrozeAction  
    冻结节点，游戏正式开始之前一切保持不动
- GainBloodAction  
    获取子弹补给（buff区刷新之后，趋向适当的buff位置以获取buff）
- GainBulletAction  
    获取回血buff（buff区刷新之后，趋向适当的buff位置以获取buff）
- GimbalLimited  
    云台受到惩罚，则执行逃跑动作
- SearchAction  
在预定的搜索路线中进行搜索
- TurnToDetectedDirection  
    受到攻击时，根据子弹来源，小车转向合适方向
- WaitBuffRefresh   
    在buff区刷新前，离开buff区到不远处，伺机等待刷新
