#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H
#include <actionlib/client/simple_action_client.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>

#include "roborts_msgs/ArmorDetectionAction.h"
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/ShootInfo.h"
#include "roborts_msgs/GimbalMode.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/SentryInfo.h"
#include "roborts_msgs/PunishInfo.h"
#include "roborts_msgs/RobotInfo.h"
#include "roborts_msgs/TreeStatus.h"
#include "roborts_msgs/GimbalControl.h"
#include "roborts_msgs/GimbalActionlib.h"
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/PyArmorInfo.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "sdk.h"
#include "gimbal.h"


namespace roborts_decision{
  enum class GimbalMode{
      GYRO_CONTROL = 0,
      CODE_CONTROL = 1,
      G_MODE_MAX_NUM = 2,
  };

  enum class ShootMode{
      SHOOT_STOP = 0,
      SHOOT_ONCE = 1,
      SHOOT_CONTINUOUS = 2,
  };

  enum _Status{
    CHASE   = 1,
    ESCAPE  = 2,
    SHOOT   = 3,
    BUFF    = 4
};

  class Blackboard{
    
    public:

      typedef std::shared_ptr<Blackboard> Ptr;
      typedef roborts_costmap::CostmapInterface CostMap;
      typedef roborts_costmap::Costmap2D CostMap2D;
      explicit Blackboard(const std::string &proto_file_path):
        is_follow_(false),
        has_got_(false),
        red_bonus_(0),  
        blue_bonus_(0), 
        red_supplier_(0), 
        blue_supplier_(0), 
        id_(3), 
        is_master_(true),
        action_state_(BehaviorState::IDLE),      
        is_chase_(false),
        remaining_time_(180),            
        enemy_lost_(true),
        sentry_lost_(true),
        cancel_flag_(true),
        //is_scan_(false),
        is_swing_(false),
        search_count_(0),
        search_points_size_(0),
        is_escape_(false),                                             
        escape_yaw_(0),
        damage_source_(0), 
        damage_source_last_(0), 
        damage_source_now_(0), 
        damage_type_(0), 
        mate_attacked_(false),
        last_armor_attack_time_(ros::Time::now()),
        last_check_attacked_time_(ros::Time::now()),
        last_check_attack_time_(ros::Time::now()),
        remain_bullet_(0),
        remain_hp_(2000),
        enemy_hp_(2000),
        last_hp_(2000),
        last_enemy_hp_(2000),
        self_dmp_(0),
        enemy_dmp_(0),
        if_punished_(false),
        mate_punished_(false),         
        gimbal_punished_(false),
        chassis_punished_(false),
        mate_gimbal_(false),
        mate_chassis_(false),
        teammate_hp_(2000),
        teammate_bullet_(0),
        teammate_heat_(0),
        heat_(0),
        mate_tree_running_(false),
        camera_lost_(true),
        camera_z_(0),
        game_status_(0),
        near_enemy_num_(0),
        chase_enemy_num_(0),
        camera_angle_(0),
        c_angle_(0),
        camera_x_(0),
        e_distance_(0),
        escape_region_(0),
        cls_num_(0),
        angle_(0)
      {

        tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
        std::string map_path = ros::package::getPath("roborts_costmap") + \
        "/config/costmap_parameter_config_for_decision.prototxt";
        costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_, map_path);
        chassis_executor_ptr_=std::make_shared<ChassisExecutor>();
        charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
        costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();
      
        //gimbal control
        ros::NodeHandle control_nh;
        gim_control_pub = control_nh.advertise<roborts_msgs::GimbalControl>("GImbalControl",30);
        
        //gimbal actionlib
        ros::NodeHandle actionlib_nh;
        gim_actionlib = actionlib_nh.advertise<roborts_msgs::GimbalActionlib>("GimbalActionlib",30);

        //interact info
        ros::NodeHandle interact_nh;   
        mate_info_sub = interact_nh.subscribe<roborts_msgs::RobotInfo>("robort_info",30, &Blackboard::MateInfoCallBack, this);
        mate_punish_sub = interact_nh.subscribe<roborts_msgs::PunishInfo>("punish",30, &Blackboard::MatePubnishCallBack, this);
        mate_treestatus_sub = interact_nh.subscribe<roborts_msgs::TreeStatus>("status",30, &Blackboard::MateTreeStatusCallBack, this);
      
        //sentry info
        ros::NodeHandle sen_nh;
        enemy_pose_sub = sen_nh.subscribe<roborts_msgs::SentryInfo>("enemy_pose_from_sentry",30, &Blackboard::EnemyPoseCallBack, this);
     
        //camera info
        ros::NodeHandle camera_nh;
        camera_sub = camera_nh.subscribe<roborts_msgs::PyArmorInfo>("PyArmorInfo",30,&Blackboard::CameraCallBack,this);

        /*------------------------------裁判系统有关话题和服务
        ros::NodeHandle referee_nh;
        game_status_sub_ = referee_nh.subscribe<roborts_msgs::GameStatus>("game_status",30, &Blackboard::GameStatusCallback, this);
        game_result_sub_ = referee_nh.subscribe<roborts_msgs::GameResult>("game_result" ,30, &Blackboard::GameResultCallback, this);
        robot_status_sub_ = referee_nh.subscribe<roborts_msgs::RobotStatus>("robot_status",30,&Blackboard::RobotStatusCallback,this);
        robot_damage_sub_ = referee_nh.subscribe<roborts_msgs::RobotDamage>("robot_damage" ,30,&Blackboard::RobotDamageCallback,this);
        robot_shoot_sub_ = referee_nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot",30,&Blackboard::RobotShootCallback,this);
        bonus_status_sub_ = referee_nh.subscribe<roborts_msgs::BonusStatus>("field_bonus_status" ,30,&Blackboard::BonusStatusCallback,this);
        supplier_status_sub_ = referee_nh.subscribe<roborts_msgs::SupplierStatus>("field_supplier_status",30,&Blackboard::SupplierStatusCallback,this);
        bonus_bool_sub_ = referee_nh.subscribe<roborts_msgs::RobotBonus>("robot_bonus",30,&Blackboard::BonusBoolCallback,this);
        shooter_heat_sub_=referee_nh.subscribe<roborts_msgs::RobotHeat>("robot_heat",30,&Blackboard::ShooterHeatCallback,this);
        game_survivor_sub_=referee_nh.subscribe<roborts_msgs::GameSurvivor>("game_survivor",30,&Blackboard::GameSurvivorCallback,this);
        projectile_supply_pub_=referee_nh.advertise<roborts_msgs::ProjectileSupply>("projectile_supply",30);
        whirl_bool_pub_=referee_nh.advertise<roborts_msgs::WhirlBool>("whirl",30); 
        swing_bool_pub_=referee_nh.advertise<roborts_msgs::SwingBool>("swing",30); 
        ------------------------------------------------------------------------------*/
        
        //部分变量初始化
        cls_.resize(5);
        for(int i=0;i<5;i++) cls_[i]=0;
        deck_time_.resize(5);
        for(int i=0;i<5;i++) deck_time_[i]=0;

        chase_goal_.resize(2);
        chase_goal_[0].header.frame_id = "map";
        chase_goal_[0].pose.orientation.x = 0;
        chase_goal_[0].pose.orientation.y = 0;
        chase_goal_[0].pose.orientation.z = 0;
        chase_goal_[0].pose.orientation.w = 1;
        chase_goal_[0].pose.position.x = 0;
        chase_goal_[0].pose.position.y = 0;
        chase_goal_[0].pose.position.z = 0;

        chase_goal_[1].header.frame_id = "map";
        chase_goal_[1].pose.orientation.x = 0;
        chase_goal_[1].pose.orientation.y = 0;
        chase_goal_[1].pose.orientation.z = 0;
        chase_goal_[1].pose.orientation.w = 1;
        chase_goal_[1].pose.position.x = 0;
        chase_goal_[1].pose.position.y = 0;
        chase_goal_[1].pose.position.z = 0;

        buff_pose_.header.frame_id = "map";
        buff_pose_.pose.position.x = 0;
        buff_pose_.pose.position.y = 0;
        buff_pose_.pose.position.z = 0;

        whirl_vel_.accel.linear.x = 0;
        whirl_vel_.accel.linear.x = 0;
        whirl_vel_.accel.linear.y = 0;
        whirl_vel_.accel.linear.z = 0;
        whirl_vel_.accel.angular.x = 0;
        whirl_vel_.accel.angular.y = 0;
        whirl_vel_.accel.angular.z = 0;

        ROS_INFO("load file");
        LoadParam(proto_file_path);

      }

      ~Blackboard() = default;

      /*---------------------------------Get---------------------------------------*/

      std::shared_ptr<tf::TransformListener> GetTFptr(){
        return tf_ptr_;
      }
      
      geometry_msgs::PoseStamped GetEnemy() const {
        return enemy_pose_;
      }

      geometry_msgs::PoseStamped GetBuffPose(){
        return buff_pose_;
      }

      geometry_msgs::PoseStamped GetRobotMapPose(){
        UpdateRobotPose();
        return robot_map_pose_;
      }

      geometry_msgs::PoseStamped GetEnemyPose(){
        geometry_msgs::PoseStamped enemy_pose;
        enemy_pose.header.frame_id = "map";
        enemy_pose.pose.position.x = 0;
        enemy_pose.pose.position.y = 0;
        enemy_pose.pose.position.z = 0;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
        enemy_pose.pose.orientation.x = quaternion.x();
        enemy_pose.pose.orientation.y = quaternion.y();
        enemy_pose.pose.orientation.z = quaternion.z();
        enemy_pose.pose.orientation.w = quaternion.w();
        try{
          enemy_pose = HandleEnemyPose();
        }
        catch(std::exception& e){
          ROS_WARN("handle enemy pose error : %s", e.what());
        }
        return enemy_pose_;
      }

      CostMap2D* GetCostMap2D(){
        return costmap_2d_;
      }
                                            
      int GetSearchCount(){
        return search_count_;
      }
      
      int GetSearchPointSize(){
        return search_points_size_;
      }

      geometry_msgs::PoseStamped GetSearchPose(int s_count){
        return search_point[s_count];
      }

      geometry_msgs::PoseStamped GetEscapePoints(int e_count){
        return escape_points_[e_count];
      }

      std::shared_ptr<ChassisExecutor> GetChassisExecutor(){
        return chassis_executor_ptr_;
      }

      std::shared_ptr<GimbalExecutor> GetGimbalExecutor(){
        return gimbal_executor_ptr_;
      }

      int GetRobotId(){
      	return id_;
      }

      BehaviorState GetActionState(){
        return action_state_;
      }

      double GetShootingRange(){
        return shooting_range_;
      }

      unsigned int GetRemainBullets(){
        return remain_bullet_;
      }

      unsigned int GetRemainHP(){
        return remain_hp_;
      }

      unsigned int GetCurrentHeat(){
        return heat_;
      }

      unsigned int GetTeammateBullets(){
        return teammate_bullet_;
      }

      unsigned int GetTeammateHP(){
        return teammate_hp_;
      }

      int GetGameStatus(){
        return game_status_;
      }

      geometry_msgs::PoseStamped GetTeammatePose(){
        return teammate_pose_;     
      }

      //获取回血buff状态(机器人编号小于10为红方，大于10为蓝方)
      int GetBonusStatus(){
        if(GetRobotId()==3||GetRobotId()==4){ 
          return red_bonus_;
        }
        if(GetRobotId()==13||GetRobotId()==14){
          return blue_bonus_;
        }
        return 0;
      }

      //获取子弹buff状态(机器人编号小于10为红方，大于10为蓝方)
      int GetSupplierStatus(){
        if(GetRobotId()==3||GetRobotId()==4){ 
          return red_supplier_;
        }
        if(GetRobotId()==13||GetRobotId()==14){
          return blue_supplier_;
        }
        return 0;
      }

      geometry_msgs::PoseStamped GetRobotGoal(){
        return robort_goal_;
      }

      _Status GetStatus(){
        return status_;
      }

      geometry_msgs::PoseStamped GetWaitRefreshPos(){
        return wait_refresh_position_;
      }

      int GetNearEnemyNUm(){
        return near_enemy_num_;
      } 

      int GetChaseEnemyNum(){
        return chase_enemy_num_;
      }

      geometry_msgs::PoseStamped GetChaseGoal(){
        return chase_goal_[0];
      }

      double GetEnemyDis(){
        return e_distance_;
        ROS_INFO("e_distance_: %f",e_distance_);
      }

      double GetCompositeAngle(){
        return c_angle_;
        ROS_INFO("c_angle_: %f",c_angle_);
      }
          
      double GetCameraAngle(){
        return camera_angle_;
        ROS_INFO("camera_angle_: %f",camera_angle_);
      }

      int GetEscapeRegion(){
          return escape_region_;
      }
      
      int GetDeck(){
        return cls_[UpdateDeckInfo()]; 
      }

      //是否为主机
      bool IsMaster(){
        return is_master_;
      }

      //是否丢失敌人目标位置
      bool IsEnemyLost(){
        if(sentry_lost_ && camera_lost_ ){
          return true;
        } else{
          return false;
        }
      }

      //是否设置取消
      bool GetCancelFlag(){
        return cancel_flag_;
      }

      //是否进行追击
      bool IsChase(){
        if(GetEnemyDistance() < 2.0){         
 	        is_chase_=false;
        }
        return is_chase_;
      }

      //bool IsFollow(){
	      //return is_follow_;
      //}

      //是否逃跑
      bool IsEscape(){
	      return is_escape_;
      }

      //云台是否旋转扫描
      bool IsSwing(){
        return is_swing_;
      }

      //自己是否受到惩罚
      bool IfPunished(){
        return if_punished_;
      }   

      //队友是否受到惩罚
      bool IfTeammatePunished(){
        return mate_punished_;
      }

      //云台是否受罚
      bool GimbalPunished(){
        return gimbal_punished_;
      }

      //底盘是否受罚
      bool ChassisPunished(){
        return chassis_punished_;
      }

      //队友是否受到攻击
      bool TeammateAttacked(){
        return mate_attacked_;
      }

      //是否在执行动作
      bool IsRunning(){
        return is_running_;
      }
      
      //相机是否丢失敌人目标
      bool GetCameraLost(){
        return camera_lost_;
      }
      
      //判断是否拿到了buff
      bool IfGotBuff(){
        return has_got_;
      }

      //识别出的“敌方装甲板序号”变化是否小于一定频率
      bool DeckChangeSlow(){
        return deck_change_slow_;
      }

      /*---------------------------------Set---------------------------------------*/

      void SetSwing(bool is_swing){
        is_swing_ = is_swing;
      }

      void SetEnemy(geometry_msgs::PoseStamped &enemy_pose){
        enemy_pose_=enemy_pose;
      }

      void SetBuffPose(geometry_msgs::PoseStamped &buff_pose){
        buff_pose_ = buff_pose;
      }

      //void SetChaseCount(unsigned int &count){
        //chase_count_ = count;
      //}

      void CancelSearch() {
        SetSearchCount(0);
      }

      void SetEnemyLost(bool lost){
        enemy_lost_ = lost;
      }

      //void SetChaseBuffer(std::vector<geometry_msgs::PoseStamped> &buffer){
        //chase_buffer_ = buffer;
      //}

      void SetCancelFlag(bool flag){
        cancel_flag_ = flag;
      }

      void SetSearchCount(unsigned int count){
        ROS_INFO("set search count: %d",search_count_);
        search_count_ = count;
      }

      void SetRobotId(int id){
      	 id_ = id;
      }  
      
      void SetActionState(BehaviorState action_state){
        action_state_=action_state;
      }

      void SetFriendMapPose(geometry_msgs::PoseStamped friend_pose){
        teammate_pose_ = friend_pose;
      }

      void SetChase(bool is_chase){
        ROS_INFO("set chase:%d",is_chase);
        is_chase_ = is_chase;
      }

      void SetFollow(bool is_follow){
        ROS_INFO("set chase:%d",is_follow);
        is_follow_ = is_follow;
      }
      void SetBuffGot(bool has_got){
        has_got_ = has_got;
      }

      void SetEscape(bool is_escape){
        ROS_INFO("set escape:%d",is_escape);
        is_escape_=is_escape;
      }

      void SetShootingRange(double shooting_range){
        ROS_INFO("set shooting range:%d",shooting_range);
        shooting_range_ = shooting_range;
      }

    /*  void SetUnderPunish(bool under_punishment){
        ROS_INFO("set under punishment:%d",under_punishment);
        under_punishment_ = under_punishment;
      }*/

      void SetRobotGoal(geometry_msgs::PoseStamped robort_goal){
        robort_goal_ = robort_goal;
      }

      void SetNearEnemyNum(int num){
        near_enemy_num_ = num;
      }

      void SetChaseEnemyNUm(int num){
        chase_enemy_num_ = num;
      }

      void SetCameraAngle(double camera_angle){
          camera_angle_ = camera_angle;
      }

      void SetCompositeAngle(double c_angle){
        c_angle_ = c_angle;
      }

      void SetEnemyDis(double e_dis){
        e_distance_ = e_dis;
      }

      void SetCameraLost(bool e_lost_){
        camera_lost_ = e_lost_;
      }

      void SetEscapeRegion(int escape_region){
          escape_region_ = escape_region;
      }

      /*-----------------------------------------------功能区--------------------------------------------------*/
      //从配置文件加载参数
      void LoadParam(const std::string &proto_file_path) {
        if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
            ROS_ERROR("Load param failed !");
            return ;
        }
        ///-------------------boot
        if(IsMaster()){
          boot_position_.resize(decision_config.master_bot().size());
          for (int i = 0; i != decision_config.master_bot().size(); i++) {
            boot_position_[i].header.frame_id = "map";
            boot_position_[i].pose.position.x = decision_config.master_bot(i).x();
            boot_position_[i].pose.position.z = decision_config.master_bot(i).z();
            boot_position_[i].pose.position.y = decision_config.master_bot(i).y();
            tf::Quaternion master_quaternion = tf::createQuaternionFromRPY(decision_config.master_bot(i).roll(),
                                                                          decision_config.master_bot(i).pitch(),
                                                                          decision_config.master_bot(i).yaw());
            boot_position_[i].pose.orientation.x = master_quaternion.x();
            boot_position_[i].pose.orientation.y = master_quaternion.y();
            boot_position_[i].pose.orientation.z = master_quaternion.z();
            boot_position_[i].pose.orientation.w = master_quaternion.w();
          }
        }else{
          boot_position_.resize(decision_config.auxe_bot().size());
          for (int i = 0; i != decision_config.auxe_bot().size(); i++) {
            boot_position_[i].header.frame_id = "map";
            boot_position_[i].pose.position.x = decision_config.auxe_bot(i).x();
            boot_position_[i].pose.position.z = decision_config.auxe_bot(i).z();
            boot_position_[i].pose.position.y = decision_config.auxe_bot(i).y();
            tf::Quaternion auxe_quaternion = tf::createQuaternionFromRPY(decision_config.auxe_bot(i).roll(),
                                                                          decision_config.auxe_bot(i).pitch(),
                                                                          decision_config.auxe_bot(i).yaw());
            boot_position_[i].pose.orientation.x = auxe_quaternion.x();
            boot_position_[i].pose.orientation.y = auxe_quaternion.y();
            boot_position_[i].pose.orientation.z = auxe_quaternion.z();
            boot_position_[i].pose.orientation.w = auxe_quaternion.w();
          }
        }
    
        ///-------------------buff
        buff_size_ = decision_config.buff_point().size();
        fixed_buff_.resize(buff_size_);
        for (int i = 0; i != buff_size_; i++) {
            fixed_buff_[i].header.frame_id = "map";
            fixed_buff_[i].pose.position.x = decision_config.buff_point(i).x();
            fixed_buff_[i].pose.position.y = decision_config.buff_point(i).y();
            fixed_buff_[i].pose.position.z = decision_config.buff_point(i).z();

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.buff_point(i).roll(),
                                                                    decision_config.buff_point(i).pitch(),
                                                                    decision_config.buff_point(i).yaw());
            fixed_buff_[i].pose.orientation.x = quaternion.x();
            fixed_buff_[i].pose.orientation.y = quaternion.y();
            fixed_buff_[i].pose.orientation.z = quaternion.z();
            fixed_buff_[i].pose.orientation.w = quaternion.w();
        }

        ///-------------------wait for refresh position
        wait_refresh_size_ = decision_config.buff_point().size();
        wait_refresh_pos_.resize(wait_refresh_size_);
        for (int i = 0; i != wait_refresh_size_; i++) {
            wait_refresh_pos_[i].header.frame_id = "map";
            wait_refresh_pos_[i].pose.position.x = decision_config.wait_point(i).x();
            wait_refresh_pos_[i].pose.position.y = decision_config.wait_point(i).y();
            wait_refresh_pos_[i].pose.position.z = decision_config.wait_point(i).z();

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.wait_point(i).roll(),
                                                                    decision_config.wait_point(i).pitch(),
                                                                    decision_config.wait_point(i).yaw());
            wait_refresh_pos_[i].pose.orientation.x = quaternion.x();
            wait_refresh_pos_[i].pose.orientation.y = quaternion.y();
            wait_refresh_pos_[i].pose.orientation.z = quaternion.z();
            wait_refresh_pos_[i].pose.orientation.w = quaternion.w();
        }

        ///-------------------search
        search_points_size_ = decision_config.search_path().size();
        search_point.resize(search_points_size_);
        for (int i = 0; i != search_points_size_; i++) {
            search_point[i].header.frame_id = "map";
            search_point[i].pose.position.x = decision_config.search_path(i).x();
            search_point[i].pose.position.y = decision_config.search_path(i).y();
            search_point[i].pose.position.z = decision_config.search_path(i).z();

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.search_path(i).roll(),
                                                                    decision_config.search_path(i).pitch(),
                                                                    decision_config.search_path(i).yaw());
            search_point[i].pose.orientation.x = quaternion.x();
            search_point[i].pose.orientation.y = quaternion.y();
            search_point[i].pose.orientation.z = quaternion.z();
            search_point[i].pose.orientation.w = quaternion.w();
        }

        ///------------------square
        square_x_.resize(decision_config.square_x().size());
        for (int i = 0; i != decision_config.square_x().size(); i++) {
            square_x_[i]= decision_config.square_x(i);
        }
        square_y_.resize(decision_config.square_y().size());
        for (int i = 0; i != decision_config.square_y().size(); i++) {
            square_y_[i]= decision_config.square_y(i);
        }

        ///-------------------escape
        escape_points_.resize((unsigned int)(decision_config.escape().size()));
        for (int i = 0; i != (unsigned int)(decision_config.escape().size()); i++) {
            escape_points_[i].header.frame_id = "map";
            escape_points_[i].pose.position.x = decision_config.escape(i).x();
            escape_points_[i].pose.position.y = decision_config.escape(i).y();
            escape_points_[i].pose.position.z = decision_config.escape(i).z();

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.escape(i).roll(),
                                                                    decision_config.escape(i).pitch(),
                                                                    decision_config.escape(i).yaw());
            escape_points_[i].pose.orientation.x = quaternion.x();
            escape_points_[i].pose.orientation.y = quaternion.y();
            escape_points_[i].pose.orientation.z = quaternion.z();
            escape_points_[i].pose.orientation.w = quaternion.w();
            ROS_INFO("get x:%f,y:%f",escape_points_[i].pose.position.x,escape_points_[i].pose.position.y);
        }

        whirl_vel_.twist.angular.z = decision_config.whirl_vel().angle_z_vel(); //default == 1
        whirl_vel_.twist.angular.y = decision_config.whirl_vel().angle_y_vel(); //        == 0
        whirl_vel_.twist.angular.x = decision_config.whirl_vel().angle_x_vel(); //        == 0

      }

      //判断是否需要转向
      bool IfTurn(){
        if(damage_type_ == 0){
              return true;
        }else{
          return false;
        }
      }

      //受到攻击时得到转向角
      int  ArmorDamageSource(){
        auto turn_time = (ros::Time::now()-last_armor_attack_time_).toSec();
        auto median = damage_source_last_;
        damage_source_last_ = damage_source_;
        if(turn_time < 0.5 ){
          if(median == damage_source_){
            return damage_source_;
          }
          if((median==1 && damage_source_==3)||(median==3 && damage_source_==1)){
            if(GetEnemyYaw()>0){
              return 1;
            }else{
              return 3;
            }
          }
          if(damage_source_!=2 &&median!= 2){
            return 0;
          }
          if(damage_source_==2){
            return median;
          }
          if(median==2){
            return median;
          }
        }else{
          return 0;
        }
      }

      //更新机器人当前位置，存储在robot_map_pose_中
        /*获取坐标的原理:
          base_link 是 map 中的一个子坐标系, 声明一个点, 默认为原点
          此时base_link在map中的坐标, 就是base_link原点在map中的坐标*/
      void UpdateRobotPose(){
        tf::Stamped<tf::Pose> robot_tf_pose;
        robot_tf_pose.setIdentity();
        robot_tf_pose.frame_id_ = "base_link";  
        robot_tf_pose.stamp_ = ros::Time();
        try{
            geometry_msgs::PoseStamped robot_pose;
            tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
            // 从 base_link 转换到 map
            tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
        }
        catch(tf::LookupException& e){
            ROS_ERROR("Transform Error looking up robot pose: %s", e.what());
        }
      }

      //更新云台与车身之间的夹角
      void UpdateGimbalAngle(){
        tf::Stamped<tf::Pose> robot_tf_pose;
        robot_tf_pose.setIdentity();
        robot_tf_pose.frame_id_ = "base_link"; 
        robot_tf_pose.stamp_ = ros::Time();
        try{
            geometry_msgs::PoseStamped robot_pose;
            tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
            // 从 base_link 转换到 gimbal
            tf_ptr_->transformPose("gimbal", robot_pose, camera_pose_);
            tf::Quaternion camera;
            tf::quaternionMsgToTF(camera_pose_.pose.orientation, camera);
            double r=0, p=0, y=0;
            tf::Matrix3x3(camera).getRPY(r, p, y);
            SetCameraAngle(y);
        }
        catch(tf::LookupException& e){
            ROS_ERROR("Transform Error looking up robot pose: %s", e.what());
        }
      }

      //获取相机识别出的敌人在世界地图中的坐标
      geometry_msgs::PoseStamped CameraEnemyPos(){
        UpdateGimbalAngle();
        geometry_msgs::PoseStamped map_pose=GetRobotMapPose();
        map_pose.header.frame_id = "map";
        geometry_msgs::Quaternion orientation = map_pose.pose.orientation;    
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));    
        double yaw, pitch, roll;    
        mat.getEulerYPR(yaw, pitch, roll);
        double angle = yaw-GetCameraAngle()+GetCompositeAngle();
        angle=angle*(-1);
        float move_dst = GetEnemyDistance();    // 相机检测到与敌人距离
        float move_x = sin(angle) * move_dst;   // x轴方向上移动距离
        float move_y = cos(angle) * move_dst;   // y轴方向上移动距离
        ROS_INFO("move_x:%f,mov_y:%f",move_x,move_y);
        geometry_msgs::PoseStamped c_goal1;
        c_goal1.header.frame_id = "base_link";
        //测试用
        geometry_msgs::PoseStamped c_goal3;
        tf_ptr_->transformPose("map", c_goal1, c_goal3);
        ROS_INFO(" pose in map: x=%f, y=%f",c_goal3.pose.position.x,c_goal3.pose.position.y);
        
        c_goal1.pose.position.z = 0;
        c_goal1.pose.position.x += move_x;
        c_goal1.pose.position.y += move_y;
        geometry_msgs::PoseStamped c_goal2;
        tf_ptr_->transformPose("map", c_goal1, c_goal2);
        ROS_INFO("camera enemy pos: x=%f, y=%f",c_goal2.pose.position.x,c_goal2.pose.position.y);
        return c_goal2;
      }
      
      //根据哨岗得到的两个敌人坐标，返回距离自己近的一个坐标
      geometry_msgs::PoseStamped HandleEnemyPose(){
        auto robot_map_pose = GetRobotMapPose();                                  
        auto dx = robot_map_pose.pose.position.x - enemy_pose1_.pose.position.x;        
        auto dy = robot_map_pose.pose.position.y - enemy_pose1_.pose.position.y; 
        double dis_one = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        auto dp = robot_map_pose.pose.position.x - enemy_pose2_.pose.position.x;        
        auto dq = robot_map_pose.pose.position.y - enemy_pose2_.pose.position.y; 
        double dis_two = std::sqrt(std::pow(dp, 2) + std::pow(dq, 2));
        if(dis_one<=dis_two){
          SetNearEnemyNum(1);    //距自己近的敌人为敌方一号车
          return enemy_pose1_;
        }else{
          SetNearEnemyNum(2);    //距自己近的敌人为敌方二号车
          return enemy_pose2_;
        }
      }

      //获取机器人正方向到敌方坐标的角度
      double GetEnemyYaw(){
        auto robot = GetRobotMapPose();             
        auto enemy = GetEnemyPose();
        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(robot.pose.orientation, rot1);
        tf::quaternionMsgToTF(enemy.pose.orientation, rot2);
        escape_yaw_ =  rot1.angleShortestPath(rot2);
        return escape_yaw_;
      }
      
      //更新搜索目标
      void UpdateSearchCount(){
        geometry_msgs::PoseStamped current_pose =GetRobotMapPose();
        double x, y;
        x = current_pose.pose.position.x;
        y = current_pose.pose.position.y;
        if(x <= 4.04){
          if(y > 2.24){
            SetSearchCount(0);
            ROS_INFO("update search count 0");
          }else
          {
            SetSearchCount(1);
            ROS_INFO("update search count 1");
          }
        }else{
          if(y > 2.4){
            SetSearchCount(6);
            ROS_INFO("update search count 6");
          }else{
            SetSearchCount(4);
            ROS_INFO("update search count 4");
          }
        }
      }

      //获取机器人当前所在地图区域
      int GetCurrentRegion(){
          geometry_msgs::PoseStamped current_pose =GetRobotMapPose();
          double x, y;
          x = current_pose.pose.position.x;
          y = current_pose.pose.position.y;
          if(x <= 4.04){
            if(y > 2.24){
              return 0;
            }else
              {
                return 3;
              }
          }else{
            if(y > 2.4){
              return 1;
            }else{
              return 2;
            }
          }
        }

      //更新逃跑区域
      void UpdateEscapeReg(){
        int reg = GetCurrentRegion();
        geometry_msgs::PoseStamped e_pose =GetEscapePoints(reg);
        geometry_msgs::PoseStamped current_pose =GetRobotMapPose();
        auto dx = current_pose.pose.position.x - e_pose.pose.position.x;        
        auto dy = current_pose.pose.position.y - e_pose.pose.position.y;        
        double e_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        if(e_distance<0.5){
          SetEscapeRegion((reg+2)%4);
        }
        //若距离哨岗传来的敌人坐标小于1.5m，则改变逃跑目标位置
        /*if(!sentry_lost_){
          auto dx = current_pose.pose.position.x - enemy_pose1_.pose.position.x;        
          auto dy = current_pose.pose.position.y - enemy_pose1_.pose.position.y; 
          double dis_one = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
          auto dp = current_pose.pose.position.x - enemy_pose2_.pose.position.x;        
          auto dq = current_pose.pose.position.y - enemy_pose2_.pose.position.y; 
          double dis_two = std::sqrt(std::pow(dp, 2) + std::pow(dq, 2));
          if(dis_one<1.5 || dis_two< 1.5){
            SetEscapeRegion((reg+2)%4);
          }
        }*/
      }

      //是否到达搜索目标地
      bool SearchPosArrived(){
        ROS_INFO("is search pose arrived");
        int s_count = GetSearchCount();
        geometry_msgs::PoseStamped s_pose =GetSearchPose(s_count);
        geometry_msgs::PoseStamped current_pose =GetRobotMapPose();
        auto dx = current_pose.pose.position.x - s_pose.pose.position.x;        
        auto dy = current_pose.pose.position.y - s_pose.pose.position.y;        
        double s_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        if(s_distance<0.4){
          return true;
        }else{
          return false;
        }
      }

      //判断是否马上要刷新buff区状态
      bool IsNearRefresh(){
        if((remaining_time_ - 60 >=0) && (remaining_time_ - 60 <=1.5)){
          return true;
        }
        if((remaining_time_ - 120 >=0) && (remaining_time_ - 120 <=1.5)){
          return true;
        } else{
          return false;
          }
      }

      //判断需要哪种buff，并获取相应的buffpose
      /*void BuffPose(int kind_of_buff_){
        if(kind_of_buff_==0){
          SetBuffPose(random_buff_[0]);  //blood
        }
        if(kind_of_buff_==1){
          SetBuffPose(random_buff_[1]);  //bullets
        }else{
           ROS_INFO("can't gain buff pose!");  
        }
      }*/

      void BuffPose(int kind_of_buff_){
          SetBuffPose(fixed_buff_[kind_of_buff_]);
      }

      //获得与敌人的距离
      double GetEnemyDistance(){
        if(!sentry_lost_){
          auto robot_map_pose = GetRobotMapPose();             
          auto enemy_map_pose = GetEnemyPose();                                  
          auto dx = robot_map_pose.pose.position.x - enemy_map_pose.pose.position.x;        
          auto dy = robot_map_pose.pose.position.y - enemy_map_pose.pose.position.y;        
          double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
          return distance;
        }
        if(!camera_lost_){
          return std::sqrt(std::pow(camera_x_, 2) + std::pow(camera_z_, 2));
        }else{
          return 3.0;    
        }
      }

      //获取出发地坐标
      geometry_msgs::PoseStamped LoadBootPosition(){
        int num = 0;
        if(GetRobotId()>10){         //小于10蓝方，大于10红方
          num = 1;
        }
        return boot_position_[num];
      }

      //上次受到攻击的时刻
      ros::Time GetLastArmorAttackTime(){
        ROS_INFO("Last_Armor_attacker_time:%lf",last_armor_attack_time_.toSec());     
        ROS_INFO("Now_time: %lf",ros::Time::now().toSec());
        ROS_INFO("Time_difference: %lf",(ros::Time::now()-last_armor_attack_time_).toSec()); 
        return last_armor_attack_time_;
      }
      
      //相对于队友，自己是否距离buff区更近
      bool NearBuff(){
        geometry_msgs::PoseStamped buff_pose;
        if(remain_hp_<=1000 || teammate_hp_<=1000){
          buff_pose = random_buff_[0];
        }
        if(remain_bullet_<=20 || teammate_bullet_<=20){
          buff_pose = random_buff_[1];
        }else{
          ROS_INFO("don't need buff!");  
          return false;
        }
        auto robot_map_pose = GetRobotMapPose();             
        auto teammate_pose = GetTeammatePose();                                  
        auto dx = robot_map_pose.pose.position.x - buff_pose.pose.position.x;        
        auto dy = robot_map_pose.pose.position.y - buff_pose.pose.position.y; 
        double dis_self = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        auto dp = teammate_pose.pose.position.x - buff_pose.pose.position.x;        
        auto dq = teammate_pose.pose.position.y - buff_pose.pose.position.y; 
        double dis_mate = std::sqrt(std::pow(dp, 2) + std::pow(dq, 2));
        if(dis_self<=dis_mate){
          return true;
        }else{
          return false;
        }
      }

      //获取每秒掉血量
      double HurtedPerSecond() {
        auto reduce_hp_ = last_hp_ - remain_hp_;         
        auto time_diff = (ros::Time::now()-last_check_attacked_time_).toSec();
        if (time_diff > 0.5) {  
          self_dmp_ = reduce_hp_ / time_diff;
          last_hp_ = remain_hp_;
          last_check_attacked_time_ = ros::Time::now();
          ROS_INFO("meet_if_condition_last_hp: %d",last_hp_);
          ROS_INFO("meet_if_condition_remain_hp: %d",remain_hp_);
          ROS_INFO("meet_if_condition_reduce_hp: %d",reduce_hp_);
          ROS_INFO("meet_if_condition_timeDiff: %f",time_diff);
          ROS_INFO("meet_if_condition_dmp: %lf",self_dmp_);
          return self_dmp_;
        } else {
            ROS_INFO("meet_else_condition_last_hp: %d",last_hp_);
            ROS_INFO("meet_else_condition_remain_hp: %d",remain_hp_);
            ROS_INFO("meet_else_condition_reduce_hp: %d",reduce_hp_);
            ROS_INFO("meet_else_condition_timeDiff: %f",time_diff);
            ROS_INFO("meet_else_condition_dmp: %lf",self_dmp_);
            return self_dmp_;
          }
      }

      //每秒敌人掉血量
      double HurtPerSecond() {
        auto reduce_hp_ = last_enemy_hp_ - enemy_hp_;         
        auto time_diff = (ros::Time::now()-last_check_attack_time_).toSec();
        if (time_diff > 0.5) {
          enemy_dmp_ =int( reduce_hp_) / time_diff;
          last_enemy_hp_ = enemy_hp_;
          last_check_attack_time_ = ros::Time::now();
          ROS_INFO("meet_if_condition_enemy_last_hp: %d",last_hp_);
          ROS_INFO("meet_if_condition_enemy_remain_hp: %d",remain_hp_);
          ROS_INFO("meet_if_condition_enemy_reduce_hp: %d",reduce_hp_);
          ROS_INFO("meet_if_condition_enemy_timeDiff: %f",time_diff);
          ROS_INFO("meet_if_condition_enemy_dmp: %lf",enemy_dmp_);
          return enemy_dmp_;
        } else {
            ROS_INFO("meet_else_condition_enemy_last_hp: %d",last_hp_);
            ROS_INFO("meet_else_condition_enemy_remain_hp: %d",remain_hp_);
            ROS_INFO("meet_else_condition_enemy_reduce_hp: %d",reduce_hp_);
            ROS_INFO("meet_else_condition_enemy_timeDiff: %f",time_diff);
            ROS_INFO("meet_else_condition_enemy_dmp: %lf",enemy_dmp_);
            return enemy_dmp_;
          }
      }      

      //根据敌我双方掉血量和剩余血量，判断是否有击败敌方的优势
      double WithAdvantage(){
        if(HurtedPerSecond()<=0 || HurtPerSecond()<=0){
          ROS_INFO("dmp_ <= 0! No war ?");
          return -10;
        }
        double self_over = remain_hp_ / HurtedPerSecond();
        double enemy_over = enemy_hp_ / HurtPerSecond();
        return enemy_over - self_over;
      }

      //是否处于六个特殊区域
      bool OnBuffLocation(){
        for(int i=0;i<6;i++){
          auto robot_map_pose = GetRobotMapPose();             
          auto special_pose = fixed_buff_[i];                                  
          auto dx = robot_map_pose.pose.position.x - special_pose.pose.position.x;        
          auto dy = robot_map_pose.pose.position.y - special_pose.pose.position.y;        
          double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
          if(distance<=0.67){
            wait_refresh_position_ = wait_refresh_pos_[i];
            return true;
          }
        }
        return false;
      }

      //更新云台控制权
      void UpdateGimcontrol(){
        gimcontrol.decision_control = is_swing_;
      }
      //更新子弹发射控制权
      void UpdateShootPermit(){
        gimcontrol.can_shoot = !gimbal_punished_;
      }
      //用于发布关于云台控制，子弹发射信息的函数
      void PublishGimcontrol(){
        UpdateGimcontrol();
        UpdateShootPermit();
        gim_control_pub.publish(gimcontrol);
      }

     //messages published to Client to send goal
      void PunishGimbalActionlib(){
        gimbalmsg.isscan = is_swing_;
        gimbalmsg.gamestatus = game_status_;
        gimbalmsg.cameralost = camera_lost_;
        gimbalmsg.yawangle = angle_;
        gim_actionlib.publish(gimbalmsg);
      }

      //返回从参数列表中获取的剩余血量
      void YamlRemainHp(int yaml_hp_){
        remain_hp_ = yaml_hp_;
      }

      void SetYawAngle(float angle){
        angle_=angle;
      }

      //返回从参数文件获取的目标位置
      void YamlChasePos(int x, int y){
        chase_goal_[0].pose.position.x = x;
        chase_goal_[0].pose.position.x = y;
      }

      //更新装甲板识别信息
      int UpdateDeckInfo(){
        int a =0;
        float t=deck_time_[0];
        for(int i=0;i<5;i++){
          if(cls_[0]!=cls_[i]){
            float diff = fabs(deck_time_[0]-deck_time_[i]);
            if(diff < 0.5) deck_change_slow_ = false;
            else deck_change_slow_ = true;
          }
          if(t<deck_time_[i]){
            t=deck_time_[i];
            a=i;
          }
        }
        return a;
      }

      //获取以敌方装甲板为目标的进攻坐标
      geometry_msgs::PoseStamped GetDeckPose(){
        geometry_msgs::PoseStamped enemy_pose;
        enemy_pose.header.frame_id = "map";
        enemy_pose =CameraEnemyPos();
        num_x_ = floor(enemy_pose.pose.position.x/0.5);
        num_y_ = floor(enemy_pose.pose.position.y/0.5);
        int deck_num = GetDeck();
        tf::Quaternion quaternion;
        if(deck_num==1){
          if(num_x_!=0) num_x_--;
          if(num_y_!=7) num_y_++;
          quaternion = tf::createQuaternionFromRPY(0,0,M_PI/4); 
        }
        geometry_msgs::PoseStamped deck_pos;
        deck_pos.header.frame_id = "map";
        deck_pos.pose.position.x = square_x_[num_x_];
        deck_pos.pose.position.y = square_y_[num_y_];
        deck_pos.pose.position.z = 0;
        deck_pos.pose.orientation.x = quaternion.x();
        deck_pos.pose.orientation.y = quaternion.y();
        deck_pos.pose.orientation.z = quaternion.z();
        deck_pos.pose.orientation.w = quaternion.w();
      }

      /*geometry_msgs::PoseStamped GetDeckPose(){
        geometry_msgs::PoseStamped enemy_pose;
        enemy_pose.header.frame_id = "map";
        enemy_pose =CameraEnemyPos();
        num_x_ = floor(enemy_pose.pose.position.x/0.5);
        num_y_ = floor(enemy_pose.pose.position.y/0.5);
        int deck_num = GetDeck();
        tf::Quaternion quaternion;
        if(deck_num==1){
          if(num_x_!=0) num_x_--;
          if(num_y_!=7) num_y_++;
          quaternion = tf::createQuaternionFromRPY(0,0,M_PI/4); 
        }
        geometry_msgs::PoseStamped map_pose=GetRobotMapPose();
        map_pose.header.frame_id = "map";
        geometry_msgs::Quaternion orientation = map_pose.pose.orientation;    
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));    
        double yaw, pitch, roll;    
        mat.getEulerYPR(yaw, pitch, roll);
        if(deck_num==left){
          yaw=yaw+M_PI/2;
        }
        if(deck_num==right){
          yaw=yaw-M_PI/2;
        }  
        if(yaw<-M_PI) yaw=2*M_PI+yaw;
        if(yaw>M_PI)  yaw=yaw-2*M_PI;
        double add_x=cos(yaw),add_y=sin(yaw);
        if(fabs(add_y)< 0.7) num_y_=0;
        else if(add_y<0) num_y_ = num_y_-1;
              else num_y_=num_y_+1;
        if(fabs(add_x)<0.7) num_x_=0;
        else if(add_x<0) num_x_=1;
              else num_x_=-1;
        geometry_msgs::PoseStamped deck_pos;
        deck_pos.header.frame_id = "map";
        deck_pos.pose.position.x = square_x_[num_x_];
        deck_pos.pose.position.y = square_y_[num_y_];
        deck_pos.pose.position.z = 0;
        deck_pos.pose.orientation.x = quaternion.x();
        deck_pos.pose.orientation.y = quaternion.y();
        deck_pos.pose.orientation.z = quaternion.z();
        deck_pos.pose.orientation.w = quaternion.w();
      }*/


      /*-------------------------------------回调函数-----------------------------------------*/
      //interact callback
      void MateInfoCallBack(const roborts_msgs::RobotInfo::ConstPtr & mate_info){
        teammate_hp_ = static_cast<unsigned int>(mate_info->Hp);
        teammate_bullet_ = static_cast<unsigned int>(mate_info->Bullet);
        teammate_heat_ = static_cast<unsigned int>(mate_info->Heat);
        teammate_pose_ = mate_info->RobotPose;
      }

      void MatePubnishCallBack(const roborts_msgs::PunishInfo::ConstPtr & punish_info){
        mate_gimbal_ = punish_info->on_gimbal;
        mate_chassis_ = punish_info->on_chassis;
        if(mate_gimbal_ || mate_chassis_){
          mate_punished_ = true;
        }else{
          mate_punished_ = false;
        }
      }

      void MateTreeStatusCallBack(const roborts_msgs::TreeStatus::ConstPtr & tree_status){
        mate_status_=static_cast<unsigned int>(tree_status->status); 
        mate_tree_running_=tree_status->is_running;
        mate_goal_=tree_status->goal; 
      }

      void EnemyPoseCallBack(const roborts_msgs::SentryInfo::ConstPtr & sentry_info){
        sentry_lost_ = sentry_info->can_view;
        if(!sentry_lost_){
          ROS_INFO("recieve enemy pose from sentry!");
          enemy_pose1_.pose.position.x =sentry_info->x1;
          enemy_pose1_.pose.position.y =sentry_info->y1;
          enemy_pose2_.pose.position.x =sentry_info->x2;
          enemy_pose2_.pose.position.y =sentry_info->y2;
          chase_goal_[0].pose.position.x = enemy_pose1_.pose.position.x;
          chase_goal_[0].pose.position.x = enemy_pose1_.pose.position.y;
          chase_goal_[1].pose.position.x = enemy_pose2_.pose.position.x;
          chase_goal_[1].pose.position.y = enemy_pose2_.pose.position.y;
          ROS_INFO("Eenemy_pose1: x=%f,y=%f",chase_goal_[0].pose.position.x,chase_goal_[0].pose.position.y);
          ROS_INFO("Eenemy_pose2: x=%f,y=%f",chase_goal_[1].pose.position.x,chase_goal_[1].pose.position.y);
        }else{
          ROS_INFO("can't get enemy pose from sentry !");

        }
      }

      void CameraCallBack(const roborts_msgs::PyArmorInfo::ConstPtr & camera_info){
        camera_lost_ = !camera_info->is_enemy;     
        if(!camera_lost_){
          ROS_INFO("recieve enemy info from camera !");
          camera_z_ = static_cast<double>(camera_info->z); 
          camera_x_ = static_cast<double>(camera_info->x);   
          double c_yaw=0;
          if(camera_x_==0||camera_z_<=0){
            c_yaw=0;
          }else c_yaw = atan2(camera_x_,camera_z_);    
          SetCompositeAngle(c_yaw);
          SetSwing(false);

          cls_[cls_num_] = static_cast<int>(camera_info->cls);
          deck_time_[cls_num_] = ros::Time::now().toSec();
          cls_num_++;
          if(cls_num_==5) cls_num_=0;
        }else{
          ROS_INFO("can't get enemy info from camera !");
        }
        UpdateDeckInfo();
      }

      /*裁判系统
      void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr & game_info)
      {
      game_status_ = static_cast<int>(game_info->game_status);
      remaining_time_ = static_cast<unsigned int>(game_info->remaining_time);
      }

      void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr & robot_status)
      {
      id_ = static_cast<int>(robot_status->id);
      remain_hp_ = static_cast<unsigned int>(robot_status->remain_hp);
      }

      void RobotDamageCallback(const roborts_msgs::RobotDamage::ConstPtr & robot_damage)
      {
      damage_type_ = static_cast<int>(robot_damage->damage_type);
      damage_source_ = static_cast<int>(robot_damage->damage_source);  
      }*/

  

  private:

      std::vector<geometry_msgs::PoseStamped> boot_position_;    //出发地位置
      
      bool has_got_;                                //是否拿到了buff
      unsigned int red_bonus_;                      //红方回血buff状态
      unsigned int blue_bonus_;                     //蓝方回血buff状态
      unsigned int red_supplier_;                   //红方子弹buff状态
      unsigned int blue_supplier_;                  //蓝方子弹buff状态
      geometry_msgs::PoseStamped buff_pose_;        //buff区位置

      int id_;                                      //机器人编号
      bool is_master_;                              //是否主机        
      geometry_msgs::PoseStamped teammate_pose_;    //队友机器人位置
      bool is_follow_;                              //是否跟随

      std::vector<geometry_msgs::PoseStamped> chase_goal_;       //追击目标坐标
      bool is_chase_;                               //是否正在追击
      unsigned int near_enemy_num_;                 //距离更近的敌人编号
      unsigned int chase_enemy_num_;                 //正在追击的敌人编号
      std::vector<float> square_y_;                    //栅格层y
      std::vector<float> square_x_;                    //栅格层x
      int num_x_;                                     //栅格层x序号
      int num_y_;                                     //栅格层y序号

      geometry_msgs::PoseStamped enemy_pose_;       //现在正在攻击的敌人位置        
      double shooting_range_;
      std::vector<geometry_msgs::PoseStamped> fixed_buff_;   //六个惩罚加成区位置
      std::vector<geometry_msgs::PoseStamped> random_buff_;  //两个随机的加成区
      geometry_msgs::PoseStamped wait_refresh_position_;     //等待刷新位置
      std::vector<geometry_msgs::PoseStamped> wait_refresh_pos_; //等待刷新位置

      geometry_msgs::PoseStamped robot_map_pose_;   // 机器人当前位置
      geometry_msgs::PoseStamped robort_goal_;      // 下一步目标位置

      bool enemy_lost_;                             // 追击时敌人是否观察不到
      bool sentry_lost_;                            // 是否与哨岗失去联系
      geometry_msgs::PoseStamped enemy_pose1_;      //敌方一号车
      geometry_msgs::PoseStamped enemy_pose2_;      // 敌方二号车
      bool cancel_flag_;                            // 是否需要采取取消行为
      CostMap2D* cost_map_2d_;

      unsigned int search_count_;
      unsigned int search_points_size_;                                
      std::vector<geometry_msgs::PoseStamped> search_point;           //搜索目标

      std::vector<geometry_msgs::PoseStamped> escape_points_;         // 撤离目标
      bool is_escape_;                                                //是否逃跑
      double escape_yaw_;                                             //逃跑时转向角
      int escape_region_;                                             //逃跑区域

      std::shared_ptr<tf::TransformListener> tf_ptr_;                 // 声明tf树 
      bool is_running_;                                               //是否正在运行动作节点
      _Status status_;
      BehaviorState action_state_;                                    //动作节点的状态

      int damage_source_;            //伤害源位置
      int damage_source_last_;       //上次伤害源位置
      int damage_source_now_;        //此次伤害源位置
      int damage_type_;              //伤害类型
      bool mate_attacked_;           //队友是否受到攻击
      ros::Time last_armor_attack_time_;          //上次受到攻击的时刻

      ros::Time last_check_attacked_time_;        //上次检测自己剩余血量的时刻
      ros::Time last_check_attack_time_;          //上次检测敌人剩余血量的时刻
      unsigned int remain_bullet_;      //自己剩余子弹量
      unsigned int remain_hp_;          //自己剩余血量
      unsigned int enemy_hp_;           //敌人剩余血量
      unsigned int last_hp_;               //上一次检测自己剩余血量
      unsigned int last_enemy_hp_;         //上一次检测敌人的剩余血量
      double self_dmp_;                 //自己每秒掉血量
      double enemy_dmp_;                //敌人每秒掉血量

      //bool first_action_;         //是否执行开场动作
      bool if_punished_;            //自己是否受到惩罚
      bool mate_punished_;          //队友是否受罚
      bool gimbal_punished_;        //自己云台是否受限
      bool chassis_punished_;       //自己底盘是否受限
      bool mate_gimbal_;            //队友云台是否受限
      bool mate_chassis_;           //队友底盘是否受限
      bool is_swing_;               // 是否进行旋转扫描
      float angle_;                 //gimbal_angle_msg.yaw_angle 

      unsigned int remaining_time_;           //游戏剩余时间，由裁判系统的游戏信息获得
      unsigned int heat_;                     //枪口热量
      unsigned short buff_size_;              //loadparam
      unsigned short supplier_size_;          //loadparam
      unsigned short wait_refresh_size_;      //loadparam
      
      //interact info
      unsigned int teammate_hp_;        //队友剩余血量
      unsigned int teammate_bullet_;    //队友剩余子弹量
      unsigned int teammate_heat_;      //队友枪口热量
      unsigned int mate_status_;        //1-追逐，2-逃跑，3-射击，4-找buff
      bool mate_tree_running_;           //队友行为树是否在运行
      geometry_msgs::PoseStamped mate_goal_;  //队友目标位置

      //camera info
      bool camera_lost_;                 //敌人是否不在视野内
      double camera_z_;                //摄像头测量的与敌人的距离
      //unsigned int cls_;                 //摄像头检测的装甲版方位
      geometry_msgs::PoseStamped camera_pose_; //机器人在gimbal坐标系的位置
      double camera_angle_;              //相机和车身之间的角度
      double c_angle_;                   //相机和装甲版之间的角度
      double camera_x_;
      double e_distance_;               //相机检测到与敌人的距离

      //chase deck
      bool behind_deck_;                //是否面向敌人的后方装甲板
      std::vector<int> cls_;            //敌人装甲板方位（1前;2侧;3后）
      std::vector<float> deck_time_;    //接收装甲板方位的时刻
      int cls_num_;                     //vector序号
      bool deck_change_slow_;            //判断敌人装甲板变化是否小于一定频率

      //refree info
      unsigned int game_status_;        //比赛状态

      //cost map
      std::shared_ptr<CostMap> costmap_ptr_;
      CostMap2D* costmap_2d_;
      unsigned char* charmap_;

      //decision config
      roborts_decision::DecisionConfig decision_config;

      roborts_msgs::TwistAccel whirl_vel_;

      //executor
      std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
      std::shared_ptr<GimbalExecutor>  gimbal_executor_ptr_;
      
      //gimbal publisher
      ros::Publisher gim_control_pub;
      roborts_msgs::GimbalControl gimcontrol;  

      //gimbal publisher
      ros::Publisher gim_actionlib;
      roborts_msgs::GimbalActionlib gimbalmsg; 

      //interact subscriber
      ros::Subscriber mate_info_sub;
      ros::Subscriber mate_punish_sub;
      ros::Subscriber mate_treestatus_sub;

      //sentry subscriber
      ros::Subscriber enemy_pose_sub;

      //camera subscriber
      ros::Subscriber camera_sub;

      /*refree subscriber
      ros::Subscriber game_status_sub_;
      ros::Subscriber game_result_sub_;
      ros::Subscriber robot_status_sub_;
      ros::Subscriber robot_damage_sub_;
      ros::Subscriber robot_shoot_sub_;
      ros::Subscriber bonus_status_sub_;
      ros::Subscriber supplier_status_sub_;
      ros::Subscriber bonus_bool_sub_;
      ros::Subscriber supplier_bool_sub_;
      ros::Subscriber shoot_info_sub_;
      ros::Subscriber shooter_heat_sub_;
      ros::Subscriber game_survivor_sub_;*/

  };// blackboard

}// robort_decision

#endif
// ROBORTS_DECISION_BLACKBOARD_H
