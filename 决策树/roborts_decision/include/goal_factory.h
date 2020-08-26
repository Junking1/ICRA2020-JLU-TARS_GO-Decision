#ifndef ROBORT_GOAL_FACTORY_H
#define ROBORT_GOAL_FACTORY_H
#include<Eigen/Core>
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/PoseStamped.h>

#include<actionlib/client/simple_action_client.h>
#include "../executor/chassis_executor.h"
#include "example_behavior/line_iterator.h"

#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_node.h"
#include "../behavior_tree/behavior_state.h"

#include "io/io.h"
#include "../proto/decision.pb.h"

namespace roborts_decision{
  class GoalFactory{
  public:
    typedef roborts_msgs::GlobalPlannerGoal GlobalGoal;
    typedef roborts_msgs::LocalPlannerGoal LocalGoal;
    typedef std::shared_ptr<GoalFactory> GoalFactoryPtr;
    
  protected : 
    Blackboard::Ptr blackboard_ptr_;
    std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
    std::shared_ptr<GimbalExecutor> gimbal_executor_ptr_;

  public:
    GoalFactory(const Blackboard::Ptr &blackboard_ptr):
        blackboard_ptr_(blackboard_ptr){
        chassis_executor_ptr_ = blackboard_ptr_->GetChassisExecutor();
        gimbal_executor_ptr_ = blackboard_ptr_ ->GetGimbalExecutor();
        }

    ~GoalFactory() = default;

    void BuffGoal() {
      geometry_msgs::PoseStamped buff_goal;
      buff_goal.header.frame_id = "map";
      buff_goal.pose.position.x =0;
      buff_goal.pose.position.y =0;
      buff_goal.pose.position.z =0;         
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);   
      buff_goal.pose.orientation.x = quaternion.x();
      buff_goal.pose.orientation.y = quaternion.y();
      buff_goal.pose.orientation.z = quaternion.z();
      buff_goal.pose.orientation.w = quaternion.w();
      buff_goal = blackboard_ptr_->GetBuffPose();
      chassis_executor_ptr_->Execute(buff_goal);
    }

    void BuffGoal(int kind_of_buff) {
      geometry_msgs::PoseStamped buff_goal;
      buff_goal.header.frame_id = "map";
      buff_goal.pose.position.x =0;
      buff_goal.pose.position.y =0;
      buff_goal.pose.position.z =0;      
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);  
      buff_goal.pose.orientation.x = quaternion.x();
      buff_goal.pose.orientation.y = quaternion.y();
      buff_goal.pose.orientation.z = quaternion.z();
      buff_goal.pose.orientation.w = quaternion.w();
      blackboard_ptr_->BuffPose(kind_of_buff);  
      buff_goal = blackboard_ptr_->GetBuffPose();
      chassis_executor_ptr_->Execute(buff_goal);
    }

    //受到攻击后转向合适的方向
    void TurnToDetectedDirection() {      
      if(blackboard_ptr_->IfTurn()){
        double q=1;   
        double d_yaw=0;
        switch (blackboard_ptr_->ArmorDamageSource()) {
          case 0:  //前方
              break;  
          case 1:  //左方
              d_yaw = M_PI / 2.;
              break;
          case 2:  //后方
              d_yaw = M_PI;
              q=0.5;
              break;
          case 3:  //右方
              d_yaw = -M_PI / 2.;
              break;
          default:
              return;
        } 
        try{ 
          ros::Rate rate(q);
          geometry_msgs::PoseStamped hurt_pose;
          auto quaternion = tf::createQuaternionMsgFromYaw(d_yaw);
          hurt_pose.header.frame_id = "base_link";
          hurt_pose.header.stamp = ros::Time::now();
          hurt_pose.pose.orientation = quaternion;
          chassis_executor_ptr_->Execute(hurt_pose);
          rate.sleep(); 
        }
        catch(std::exception& e){
          ROS_ERROR("ERROR: %S",e.what());
        }
      } 
    }

    //快刷新时离开buff区
    void LeaveFormBuff(){
      geometry_msgs::PoseStamped leave_buff_goal;
      try{
        leave_buff_goal.header.frame_id = "map";
        leave_buff_goal.pose.position.x =0;
        leave_buff_goal.pose.position.y =0;
        leave_buff_goal.pose.position.z =0;          
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);  
        leave_buff_goal.pose.orientation.x = quaternion.x();
        leave_buff_goal.pose.orientation.y = quaternion.y();
        leave_buff_goal.pose.orientation.z = quaternion.z();
        leave_buff_goal.pose.orientation.w = quaternion.w(); 
        //距下一次刷新时间是否小于1.5秒
        if(blackboard_ptr_->IsNearRefresh()){        
          leave_buff_goal = blackboard_ptr_->GetWaitRefreshPos();
          chassis_executor_ptr_->Execute(leave_buff_goal);
        }
      }
      catch(std::exception& e){
        ROS_ERROR("ERROR: %s",e.what());
      }
    }

    void CostMapFiltar(geometry_msgs::PoseStamped pose1){
      geometry_msgs::PoseStamped filtar_pose;
      filtar_pose.header.frame_id = "map";
      filtar_pose = pose1;
        auto enemy_x = filtar_pose.pose.position.x;
        auto enemy_y = filtar_pose.pose.position.y;
        unsigned int goal_cell_x, goal_cell_y;
        auto get_enemy_cell = blackboard_ptr_->GetCostMap2D()->World2Map(enemy_x,         
                                                                   enemy_y,
                                                                   goal_cell_x,
                                                                   goal_cell_y);
        if (!get_enemy_cell) {
          ROS_ERROR("The conversion from Word to Map was failed!");
          return;
        }
        geometry_msgs::PoseStamped robot_map_pose = blackboard_ptr_->GetRobotMapPose();
        auto robot_x = robot_map_pose.pose.position.x;
        auto robot_y = robot_map_pose.pose.position.y;
        unsigned int robot_cell_x, robot_cell_y;
        double goal_x, goal_y;
        auto get_robot_cell = blackboard_ptr_->GetCostMap2D()->World2Map(robot_x,        
                                              robot_y,
                                              robot_cell_x,
                                              robot_cell_y);
        if (!get_robot_cell) {
          ROS_ERROR("The conversion from Word to Map was failed!");
          return;
        }
        if (blackboard_ptr_->GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= 253) {   
          bool find_goal = false;
          for(FastLineIterator line( goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance()) {     
            auto point_cost = blackboard_ptr_->GetCostMap2D()->GetCost((unsigned int) (line.GetX()), (unsigned int) (line.GetY())); 
            if(point_cost >= 253){
              continue;
            } else {
              find_goal = true;
              blackboard_ptr_->GetCostMap2D()->Map2World((unsigned int) (line.GetX()),
                                                     (unsigned int) (line.GetY()),
                                                     goal_x,
                                                     goal_y);
              filtar_pose.pose.position.x = goal_x;
              filtar_pose.pose.position.y = goal_y;
              break;
            }
          }
          if (find_goal) {
            blackboard_ptr_->SetCancelFlag(true);
            chassis_executor_ptr_->Execute(filtar_pose);
          } else {
            if (blackboard_ptr_->GetCancelFlag()) {                
              chassis_executor_ptr_->Cancel();
              blackboard_ptr_->SetCancelFlag(false);
            }
            return;
          }
        } else {                                              
          blackboard_ptr_->SetCancelFlag(true);
          chassis_executor_ptr_->Execute(filtar_pose);
        }
    }

    //追击敌人
    void ChaseGoal() {
      geometry_msgs::PoseStamped chase_goal;                                
      if(!blackboard_ptr_->GetCameraLost()){
        chase_goal= blackboard_ptr_->CameraEnemyPos();
        if(blackboard_ptr_->GetEnemyDistance()){  
          if (blackboard_ptr_->GetCancelFlag()) {      
            chassis_executor_ptr_->Cancel();
            blackboard_ptr_->SetCancelFlag(false);
          }
          blackboard_ptr_->SetChase(false);
          return;
        }
        chassis_executor_ptr_->Execute(chase_goal);
      }else{
        chase_goal=blackboard_ptr_->GetChaseGoal(); 
      auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();      
      auto dx = chase_goal.pose.position.x - robot_map_pose.pose.position.x;        
      auto dy = chase_goal.pose.position.y - robot_map_pose.pose.position.y;        
      auto yaw = std::atan2(dy, dx);
      // 若敌我距离为1-2米，则视为追到，此时取消追击 
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 0.7 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1.5) {       
        if (blackboard_ptr_->GetCancelFlag()) {      
          chassis_executor_ptr_->Cancel();
          blackboard_ptr_->SetCancelFlag(false);
        }
        blackboard_ptr_->SetChase(false);
        return;
      } else {                                       
        //若未追到，则将坐标转化为代价地图中的坐标计算
        auto orientation = tf::createQuaternionMsgFromYaw(yaw);
        geometry_msgs::PoseStamped reduce_goal;
        reduce_goal.pose.orientation = robot_map_pose.pose.orientation;
        reduce_goal.header.frame_id = "map";
        reduce_goal.header.stamp = ros::Time::now();
        reduce_goal.pose.position.x = chase_goal.pose.position.x - 1.2 * cos(yaw);
        reduce_goal.pose.position.y = chase_goal.pose.position.y - 1.2 * sin(yaw);
        reduce_goal.pose.position.z = 1;
        CostMapFiltar(reduce_goal);
      }
      }
    }

    //跟随队友
    void FollowGoal(){
      auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
      auto friend_pose =blackboard_ptr_->GetTeammatePose();                                 
      auto dx = friend_pose.pose.position.x - robot_map_pose.pose.position.x;        
      auto dy = friend_pose.pose.position.y - robot_map_pose.pose.position.y;        
      auto yaw = std::atan2(dy, dx); 
      //跟随距离设置为1-2米，在此范围内则停止
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1.0 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0) {       
        return;
      } else {                                       
        //不在跟随范围内则跟随
        auto orientation = tf::createQuaternionMsgFromYaw(yaw);
        geometry_msgs::PoseStamped reduce_goal;
        reduce_goal.pose.orientation = robot_map_pose.pose.orientation;

        reduce_goal.header.frame_id = "map";
        reduce_goal.header.stamp = ros::Time::now();
        reduce_goal.pose.position.x = friend_pose.pose.position.x - 1.2 * cos(yaw);   
        reduce_goal.pose.position.y = friend_pose.pose.position.y - 1.2 * sin(yaw);
        reduce_goal.pose.position.z = 1;
        CostMapFiltar(reduce_goal);
      }      
    }

     void DeckChase(){
      geometry_msgs::PoseStamped deck_pose;
      deck_pose.header.frame_id = "map";
      deck_pose.pose.orientation.x = 0;
      deck_pose.pose.orientation.y = 0;
      deck_pose.pose.orientation.z = 0;
      deck_pose.pose.orientation.w = 1;
      deck_pose.pose.position.x = 0;
      deck_pose.pose.position.y = 0;
      deck_pose.pose.position.z = 0;
      deck_pose = blackboard_ptr_->GetDeckPose();
      CostMapFiltar(deck_pose);
    }      
  
    //返回出发地
    void BackToBootArea(){
      geometry_msgs::PoseStamped boot_pose ; 
      boot_pose.header.frame_id = "map";
      boot_pose.pose.orientation.x = 0;
      boot_pose.pose.orientation.y = 0;
      boot_pose.pose.orientation.z = 0;
      boot_pose.pose.orientation.w = 1;
      boot_pose.pose.position.x = 0;
      boot_pose.pose.position.y = 0;
      boot_pose.pose.position.z = 0;
      
      boot_pose = blackboard_ptr_->LoadBootPosition();
      auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
      auto dx = boot_pose.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = boot_pose.pose.position.y - robot_map_pose.pose.position.y;

      auto boot_yaw = tf::getYaw(boot_pose.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(boot_pose.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
        chassis_executor_ptr_->Execute(boot_pose);
      }
    }

    //摆动防御
    void SwingDefend()
    {
      float move_dst = 0.20;               //车身左右移动的距离
      geometry_msgs::PoseStamped swing_goal;
      swing_goal.header.frame_id = "map";
      swing_goal = blackboard_ptr_->GetRobotMapPose();
      ROS_INFO("Got pose.");
      tf::Quaternion cur_q;
      tf::quaternionMsgToTF(swing_goal.pose.orientation, cur_q);
      double r, p, y;
      tf::Matrix3x3(cur_q).getRPY(r, p, y); // 从位置信息中获取车身的角度
      float move_x = asin(y) * move_dst;    // x轴方向上移动距离
      float move_y = -acos(y) * move_dst;   // y轴方向上移动距离
      swing_goal.pose.position.z = 0;
      ros::Rate rate(20);
      while (ros::ok() && blackboard_ptr_->IsSwing())
      {
        ROS_INFO("Is swing");
        // 更新 x, y 轴坐标
        swing_goal.pose.position.x += move_x;
        swing_goal.pose.position.y += move_y;
        chassis_executor_ptr_->Execute(swing_goal);
        rate.sleep();
        swing_goal.pose.position.x -= move_x;
        swing_goal.pose.position.y -= move_y;
        chassis_executor_ptr_->Execute(swing_goal);
        rate.sleep();
      }
    }

    //初始化云台转动相关参数
    void ScanViewInit(){
        tf::Stamped<tf::Pose> gimbal_tf_pose;
        gimbal_tf_pose.setIdentity();
        gimbal_tf_pose.frame_id_ = "gimbal";  
        gimbal_tf_pose.stamp_ = ros::Time();
        geometry_msgs::PoseStamped gimbal_base_pose;
        try{
            geometry_msgs::PoseStamped gimbal_pose;
            tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
            // 从 gimbal 转换到 base_link
            blackboard_ptr_->GetTFptr()->transformPose("base_link", gimbal_pose, gimbal_base_pose);
        }
        catch(tf::LookupException& e){
            ROS_ERROR("Transform Error looking up robot pose: %s", e.what());
        }
        tf::Quaternion q;
        tf::quaternionMsgToTF(gimbal_base_pose.pose.orientation, q);
        double r=0, p=0, y=0;
        tf::Matrix3x3(q).getRPY(r, p, y);
        if(-1.5 <= y && 1.5 >= y){
          blackboard_ptr_->SetYawAngle(y);
        }else blackboard_ptr_->SetYawAngle(0.7);
        ROS_INFO("y: %f",y);
    }

    //搜索敌人
    void SearchGoal(){
      ROS_INFO("start search");
      //blackboard_ptr_->UpdateSearchCount();
      geometry_msgs::PoseStamped path;
      path.header.frame_id = "map";
      path.pose.orientation.x = 0;
      path.pose.orientation.y = 0;
      path.pose.orientation.z = 0;
      path.pose.orientation.w = 1;
      path.pose.position.x = 0;
      path.pose.position.y = 0;
      path.pose.position.z = 0;
      int count = blackboard_ptr_->GetSearchCount();
      ROS_INFO("search count:%d",count);
      int size =  blackboard_ptr_->GetSearchPointSize();
      ROS_INFO("search size:%d",size);
        path = blackboard_ptr_->GetSearchPose(count);
        geometry_msgs::PoseStamped current_pose =blackboard_ptr_->GetRobotMapPose();
        auto dx = current_pose.pose.position.x - path.pose.position.x;        
        auto dy = current_pose.pose.position.y - path.pose.position.y;        
        double s_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        ROS_INFO("s_distance=%f",s_distance);
        if(s_distance<0.4){
           blackboard_ptr_->SetSearchCount((count+1)%size);
            ROS_INFO("COUNT+1");
        }
      count = blackboard_ptr_->GetSearchCount();
      path = blackboard_ptr_->GetSearchPose(count);
      ROS_INFO("next search pose x:%f,y:%f",path.pose.position.x,path.pose.position.y);
      try{
        chassis_executor_ptr_->Execute(path);
        ROS_INFO("excute ");
      }
      catch(std::exception& e){
        ROS_WARN("search execute error %s: ", e.what());
      }
    }

    //逃跑
    void Escape(){
      ROS_INFO("Start scape");
      geometry_msgs::PoseStamped escape_goal;
      escape_goal.header.frame_id = "map";
      escape_goal.pose.orientation.x = 0;
      escape_goal.pose.orientation.y = 0;
      escape_goal.pose.orientation.z = 0;
      escape_goal.pose.orientation.w = 1;
      escape_goal.pose.position.x = 0;
      escape_goal.pose.position.y = 0;
      escape_goal.pose.position.z = 0;
      blackboard_ptr_->UpdateEscapeReg();
      int region = blackboard_ptr_->GetEscapeRegion();
      ROS_INFO("escape region:%d",region);
      escape_goal = blackboard_ptr_ -> GetEscapePoints(region);
      ROS_INFO("escape x:%f,y:%f",escape_goal.pose.position.x,escape_goal.pose.position.y);
      try{
        chassis_executor_ptr_->Execute(escape_goal);
        ROS_INFO("escapping ...");
      }
      catch(std::exception& e){
        ROS_WARN("escape error occured : %s", e.what());
      }
    }

    void CancelGoal() {
      ROS_INFO("Cancel Goal!");
      blackboard_ptr_->SetCancelFlag(false);
      blackboard_ptr_->SetActionState(BehaviorState::IDLE);
    }

    // 放弃云台控制权, 取消转动云台进行扫描的行为
    void CancelScan(){
        if(blackboard_ptr_->IsSwing()){
            blackboard_ptr_->SetSwing(false);
            ROS_INFO("cancel gimbal-scan action .");
        }
    }

    // 禁止地盘的运动
    void CancelChassis(){
      chassis_executor_ptr_->Cancel();
    }

      // 禁止云台转动
    void CancelGimbal(){
      gimbal_executor_ptr_ -> Cancel();
    }
  };  // goalfactory
} //roborts_decision

#endif  // ROBORT_GOAL_FACTORY_H
