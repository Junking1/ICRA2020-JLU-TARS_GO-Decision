
#include "behavior_tree/behavior_tree.h"

#include "action_node/BackBootArea.h"
#include "action_node/ChaseAction.h"
#include "action_node/ChassisLimited.h"
#include "action_node/DefendAction.h"
#include "action_node/EscapeAction.h"
#include "action_node/FollowAction.h"
#include "action_node/FrozeAction.h"
#include "action_node/GainBloodAction.h"
#include "action_node/GainBulletAction.h"
#include "action_node/GimbalLimited.h"
#include "action_node/SearchAction.h"
#include "action_node/TurnToDetectedDirection.h"
#include "action_node/WaitBuffRefresh.h"
#include "interact/mutualboard.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "icra_decision_node");
    ros::Time::init();
    std::string config_file__path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
   
    auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(config_file__path);
    auto goal_factory_ = std::make_shared<roborts_decision::GoalFactory>(blackboard_ptr_);
    auto mutualboard_ptr_ = std::make_shared<roborts_decision::MutualBoard>(blackboard_ptr_);    

    //action
    auto froze_action_=std::make_shared<roborts_decision::FrozeAction>(blackboard_ptr_,goal_factory_);                             //A1
    auto back_boot_area_=std::make_shared<roborts_decision::BackBootArea>(blackboard_ptr_,goal_factory_);                          //A2
    auto chassis_limited_action_=std::make_shared<roborts_decision::ChassisLimited >(blackboard_ptr_,goal_factory_);               //A3
    auto gimbal_limited_action_ = std::make_shared<roborts_decision::GimbalLimited>(blackboard_ptr_, goal_factory_);               //A4       
    auto swing_action_=std::make_shared<roborts_decision::DefendAction>(blackboard_ptr_,goal_factory_);                            //A5
    auto gain_blood_action_=std::make_shared<roborts_decision::GainBloodAction>(blackboard_ptr_,goal_factory_);                    //A6
    auto wait_buff_refresh_ = std::make_shared<roborts_decision::WaitBuffRefresh>(blackboard_ptr_, goal_factory_);                 //A7
    auto escape_action_=std::make_shared<roborts_decision::EscapeAction>(blackboard_ptr_,goal_factory_);                           //A8
    auto gain_bullet_action_=std::make_shared<roborts_decision::GainBulletAction>(blackboard_ptr_, goal_factory_);                 //A9
    auto chase_action_ = std::make_shared<roborts_decision::ChaseAction>(blackboard_ptr_, goal_factory_);                          //A10
    auto turn_to_detected_direction_= std::make_shared<roborts_decision::TurnToDetectedDirection>(blackboard_ptr_, goal_factory_); //A11                
    auto search_action_ = std::make_shared<roborts_decision::SearchAction>(blackboard_ptr_, goal_factory_);                        //A12
    auto follow_action_=std::make_shared<roborts_decision::FollowAction>(blackboard_ptr_,goal_factory_);                           //A13
                  
    //----------------------------------------------------------------------------------------------------------------------------------
//s1
    auto game_status_selector_=std::make_shared<roborts_decision::SelectorNode>("game_status_selector",blackboard_ptr_);
    
    //c1:游戏未开始的条件节点
        auto game_not_start_condition_ = std::make_shared<roborts_decision::PreconditionNode>("game_not_start_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GetGameStatus()!=4){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

    game_not_start_condition_->SetChild(froze_action_);

    //c2:游戏结束
        auto game_over_condition_ = std::make_shared<roborts_decision::PreconditionNode>("game_over_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GetGameStatus()==4){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

    game_over_condition_->SetChild(back_boot_area_);
    
//s2
    auto game_start_selector_=std::make_shared<roborts_decision::SelectorNode>("game_start_selector",blackboard_ptr_);
    
    //游戏开始
        auto game_start_condition_ = std::make_shared<roborts_decision::PreconditionNode>("game_start_condition_", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GetGameStatus()==4){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

    game_start_condition_->SetChild(game_start_selector_);

    game_status_selector_->AddChildren(game_not_start_condition_);
    game_status_selector_->AddChildren(game_over_condition_);
    game_status_selector_->AddChildren(game_start_condition_); 



//c3:自己受到惩罚
    auto under_punish_condition_ = std::make_shared<roborts_decision::PreconditionNode>("under_punish_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IfPunished()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

//s3
    auto under_punish_selector_=std::make_shared<roborts_decision::SelectorNode>("under_punish_selector",blackboard_ptr_);
    under_punish_condition_->SetChild(under_punish_selector_);

    //c6:云台受限
    auto gimbal_pubnished_condition_ = std::make_shared<roborts_decision::PreconditionNode>("gimbal_pubnished_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GimbalPunished()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    gimbal_pubnished_condition_->SetChild(gimbal_limited_action_);    

    //c7:底盘受限
    auto chassis_pubnished_condition_ = std::make_shared<roborts_decision::PreconditionNode>("chassis_pubnished_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->ChassisPunished()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    chassis_pubnished_condition_->SetChild(chassis_limited_action_);    

    under_punish_selector_->AddChildren(gimbal_pubnished_condition_);
    under_punish_selector_->AddChildren(chassis_pubnished_condition_);

    //c4:一方血量不足
    auto one_side_blood_short_condition_= std::make_shared<roborts_decision::PreconditionNode>("one_side_blood_short_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GetRemainHP()<=1000 || blackboard_ptr_->GetTeammateHP()<=1000){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

//s4
    auto plan_to_gain_blood_selector_=std::make_shared<roborts_decision::SelectorNode>("plan_to_gain_blood_selector",blackboard_ptr_);
    one_side_blood_short_condition_->SetChild(plan_to_gain_blood_selector_);

    //c8:血量不足,受到攻击且有击败优势
    auto blood_short_with_advantage_condition_= std::make_shared<roborts_decision::PreconditionNode>("blood_short_with_advantage_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GetRemainHP()<=1000 && blackboard_ptr_->HurtedPerSecond()>10 && blackboard_ptr_->WithAdvantage()<2.0){
             return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    blood_short_with_advantage_condition_->SetChild(swing_action_);

    //c11:血量不足,受到攻击
    auto blood_short_and_escape_condition_= std::make_shared<roborts_decision::PreconditionNode>("blood_short_and_escape_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GetRemainHP()<=1000 && blackboard_ptr_->HurtedPerSecond()>10){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    blood_short_and_escape_condition_->SetChild(escape_action_); 

    //c9:靠近buff的机器人取buff
    auto near_blood_buff_condition_= std::make_shared<roborts_decision::PreconditionNode>("near_blood_buff_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->NearBuff() && blackboard_ptr_->GetBonusStatus()==0){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    near_blood_buff_condition_->SetChild(gain_blood_action_); 

    //c10:快到buff区刷新时间
    auto near_buff_refresh_time_condition_= std::make_shared<roborts_decision::PreconditionNode>("near_buff_refresh_time_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IsNearRefresh() && blackboard_ptr_->OnBuffLocation()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    near_buff_refresh_time_condition_->SetChild(wait_buff_refresh_); 

    plan_to_gain_blood_selector_->AddChildren(blood_short_with_advantage_condition_);
    plan_to_gain_blood_selector_->AddChildren(near_blood_buff_condition_);
    plan_to_gain_blood_selector_->AddChildren(near_buff_refresh_time_condition_);
    plan_to_gain_blood_selector_->AddChildren(blood_short_and_escape_condition_);

    //c5:一方子弹不足
    auto one_side_bullet_short_condition_ = std::make_shared<roborts_decision::PreconditionNode>("one_side_bullet_short_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GetRemainBullets()<20 ){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

//s5
    auto plan_to_gain_bullet_selector_=std::make_shared<roborts_decision::SelectorNode>("plan_to_gain_bullet_selector",blackboard_ptr_);
    one_side_bullet_short_condition_->SetChild(plan_to_gain_bullet_selector_);

    //c12:子弹不足，受到攻击且有击败优势
    auto bullet_short_with_advantage_condition_= std::make_shared<roborts_decision::PreconditionNode>("bullet_short_with_advantage_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GetRemainBullets()<=20 && blackboard_ptr_->HurtedPerSecond()>10 && blackboard_ptr_->WithAdvantage()<= -1.0){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    bullet_short_with_advantage_condition_->SetChild(swing_action_);

    //c14:子弹不足，受到攻击且没有击败优势
    auto bullet_short_and_escape_condition_= std::make_shared<roborts_decision::PreconditionNode>("bullet_short_and_escape_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GetRemainBullets()<=20 && blackboard_ptr_->HurtedPerSecond()>10 ){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    bullet_short_and_escape_condition_->SetChild(escape_action_);

    //c13:靠近buff的机器人取buff
    auto near_bullet_buff_condition_= std::make_shared<roborts_decision::PreconditionNode>("near_bullet_buff_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->NearBuff() && blackboard_ptr_->GetSupplierStatus()==0){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    near_blood_buff_condition_->SetChild(gain_bullet_action_); 

    plan_to_gain_bullet_selector_->AddChildren(bullet_short_with_advantage_condition_);
    plan_to_gain_bullet_selector_->AddChildren(near_bullet_buff_condition_);
    plan_to_gain_bullet_selector_->AddChildren(near_buff_refresh_time_condition_); 
    plan_to_gain_bullet_selector_->AddChildren(bullet_short_and_escape_condition_);


//s6
    auto actively_approach_selector_=std::make_shared<roborts_decision::SelectorNode>("actively_approach_selector",blackboard_ptr_);

    //c15:与敌人距离大于3米
    auto actively_chase_condition_ = std::make_shared<roborts_decision::PreconditionNode>("actively_chase_condition", blackboard_ptr_,
        [&]() {
 		    if (!blackboard_ptr_->IsEnemyLost() && blackboard_ptr_->GetEnemyDistance()>3){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_chase_condition_->SetChild(chase_action_);

    //c16:主动进攻时受到装甲伤害
    auto actively_with_attacked_condition_ = std::make_shared<roborts_decision::PreconditionNode>("actively_with_attacked_condition", blackboard_ptr_,
        [&]() {
 		    if (((ros::Time::now()-blackboard_ptr_->GetLastArmorAttackTime()).toSec()<0.5)&&blackboard_ptr_->IfTurn()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_with_attacked_condition_->SetChild(turn_to_detected_direction_);

    //c19:与哨岗失去联系且摄像头看不到敌人
    auto actively_search_condition_ = std::make_shared<roborts_decision::PreconditionNode>("actively_search_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IsEnemyLost()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_search_condition_->SetChild(search_action_);

    //c18:主动进攻时受到攻击且无攻击优势
    auto actively_escape_condition_ = std::make_shared<roborts_decision::PreconditionNode>("actively_escape_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->WithAdvantage()>2.0){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_escape_condition_->SetChild(escape_action_);

    //c17:我方每秒掉血量大于一定值
    auto actively_swing_condition_ = std::make_shared<roborts_decision::PreconditionNode>("actively_swing_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->HurtedPerSecond()>=60){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_swing_condition_->SetChild(swing_action_);

    //c20:队友受到惩罚且正在遭受攻击
    auto under_pubulish_and_follow_condition_ = std::make_shared<roborts_decision::PreconditionNode>("chassis_pubnished_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IfTeammatePunished() && blackboard_ptr_->TeammateAttacked()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    under_pubulish_and_follow_condition_->SetChild(follow_action_);   

    actively_approach_selector_->AddChildren(actively_chase_condition_);
    actively_approach_selector_->AddChildren(actively_with_attacked_condition_);
    actively_approach_selector_->AddChildren(actively_swing_condition_);
    actively_approach_selector_->AddChildren(actively_escape_condition_);
    actively_approach_selector_->AddChildren(actively_search_condition_);
    actively_approach_selector_->AddChildren(under_pubulish_and_follow_condition_);

    game_start_selector_->AddChildren(under_punish_condition_);
    game_start_selector_->AddChildren(one_side_blood_short_condition_);
    game_start_selector_->AddChildren(one_side_bullet_short_condition_);
    game_start_selector_->AddChildren(actively_approach_selector_);

    ros::Rate rate(30);
    roborts_decision::BehaviorTree root_(game_status_selector_, 100); 
    while(ros::ok()){
        root_.Run();
        mutualboard_ptr_->ExchangeData();
        blackboard_ptr_->PublishGimcontrol();
        ros::spinOnce();
        rate.sleep();
    }
  
} //main
