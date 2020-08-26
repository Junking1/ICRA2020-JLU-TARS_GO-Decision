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
#include "action_node/BackAwayAction.h"
#include "action_node/DeckChaseAction.h"

void Command();
char command = '0';

int main(int argc, char **argv){    
    //
    ros::init(argc, argv, "behavior_test_node");
    ros::Time::init();
    
    //param yaml
    ros::NodeHandle param_yaml_nh;
    int manual_hp;    
    int buff_number; 
    param_yaml_nh.param<int>("manual_hp", manual_hp, 4);
    ROS_INFO("yaml:manual_hp=%d",manual_hp);
    param_yaml_nh.param<int>("buff_number", buff_number, 5);
    ROS_INFO("yaml:buff_number=%d",buff_number);
    int escape_r;    
    double chase_x;       
    double chase_y;       
    bool e_lost;
    bool c_lost;
    param_yaml_nh.param<int>("escape_r", escape_r, 0);
    ROS_INFO("yaml:escape_r=%d",escape_r);
    param_yaml_nh.param<double>("chase_x", chase_x, 0);
    ROS_INFO("yaml:chase_x=%f",chase_x);
    param_yaml_nh.param<double>("chase_y", chase_y, 0);
    ROS_INFO("yaml:chase_y=%f",chase_y);
    param_yaml_nh.param<bool>("e_lost", e_lost, false);
    ROS_INFO("yaml:e_lost=%d",e_lost);
    param_yaml_nh.param<bool>("c_lost", c_lost, false);
    ROS_INFO("yaml:c_lost=%d",c_lost);

    std::string file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";

    auto black_ptr = std::make_shared<roborts_decision::Blackboard>(file_path);
    auto g_factory = std::make_shared<roborts_decision::GoalFactory>(black_ptr);
    auto cha_executor = new roborts_decision::ChassisExecutor;

    black_ptr->SetEscapeRegion(escape_r);
    //black_ptr->YamlChasePos(chase_x,chase_y);
    black_ptr->YamlRemainHp(manual_hp);
    black_ptr->BuffPose(buff_number);


    //action
    auto back_boot_area=std::make_shared<roborts_decision::BackBootArea>(black_ptr,g_factory);                          //                 //
    auto chase_action = std::make_shared<roborts_decision::ChaseAction>(black_ptr, g_factory);                          //
    auto chassis_limited_action=std::make_shared<roborts_decision::ChassisLimited >(black_ptr,g_factory);               //
    auto swing_action=std::make_shared<roborts_decision::DefendAction>(black_ptr,g_factory);                            //
    auto escape_action=std::make_shared<roborts_decision::EscapeAction>(black_ptr,g_factory);                           //
    auto follow_action=std::make_shared<roborts_decision::FollowAction>(black_ptr,g_factory);                           //
    auto froze_action=std::make_shared<roborts_decision::FrozeAction>(black_ptr,g_factory);                             //
    auto gain_blood_action=std::make_shared<roborts_decision::GainBloodAction>(black_ptr,g_factory);                    //
    auto gain_bullet_action=std::make_shared<roborts_decision::GainBulletAction>(black_ptr, g_factory);                 //
    auto search_action= std::make_shared<roborts_decision::SearchAction>(black_ptr, g_factory);                        //
    auto turn_to_detected_direction= std::make_shared<roborts_decision::TurnToDetectedDirection>(black_ptr, g_factory); //                   
    auto wait_buff_refresh = std::make_shared<roborts_decision::WaitBuffRefresh>(black_ptr, g_factory);                 //
    auto gimbal_limited_action = std::make_shared<roborts_decision::GimbalLimited>(black_ptr, g_factory);               //          
    auto back_away_action = std::make_shared<roborts_decision::BackAwayAction>(black_ptr, g_factory);               //                    
    auto deck_chase_action = std::make_shared<roborts_decision::DeckChaseAction>(black_ptr, g_factory);               //       
    //----------------------------------------------------------------------------------------------------------------------------------

//s1
    auto game_status_selector=std::make_shared<roborts_decision::SelectorNode>("game_status_selector",black_ptr);
    
    //c1:游戏未开始的条件节点
        auto game_not_start_condition = std::make_shared<roborts_decision::PreconditionNode>("game_not_start_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GetGameStatus()!=4){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

    game_not_start_condition->SetChild(froze_action);

    
//s2
    auto game_start_selector=std::make_shared<roborts_decision::SelectorNode>("game_start_selector",black_ptr);

    //c3:自己受到惩罚
    auto under_punish_condition = std::make_shared<roborts_decision::PreconditionNode>("under_punish_condition", black_ptr,
        [&]() {
 		    if (black_ptr->IfPunished()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

//s3
    auto under_punish_selector=std::make_shared<roborts_decision::SelectorNode>("under_punish_selector",black_ptr);
    under_punish_condition->SetChild(under_punish_selector);

    //c6:云台受限
    auto gimbal_pubnished_condition = std::make_shared<roborts_decision::PreconditionNode>("gimbal_pubnished_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GimbalPunished()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    gimbal_pubnished_condition->SetChild(gimbal_limited_action);    

    //c7:底盘受限
    auto chassis_pubnished_condition = std::make_shared<roborts_decision::PreconditionNode>("chassis_pubnished_condition", black_ptr,
        [&]() {
 		    if (black_ptr->ChassisPunished()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    chassis_pubnished_condition->SetChild(chassis_limited_action);    

    
    //c4:一方血量不足
    auto one_side_blood_short_condition= std::make_shared<roborts_decision::PreconditionNode>("one_side_blood_short_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GetRemainHP()<=1000 || black_ptr->GetTeammateHP()<=1000){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

//s4
    auto plan_to_gain_blood_selector=std::make_shared<roborts_decision::SelectorNode>("plan_to_gain_blood_selector",black_ptr);
    one_side_blood_short_condition->SetChild(plan_to_gain_blood_selector);

    //c8:血量不足,受到攻击且有击败优势
    auto blood_short_with_advantage_condition= std::make_shared<roborts_decision::PreconditionNode>("blood_short_with_advantage_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GetRemainHP()<=1000 && black_ptr->HurtedPerSecond()>10 && black_ptr->WithAdvantage()<2.0){
             return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    blood_short_with_advantage_condition->SetChild(swing_action);

    //c11:血量不足,受到攻击且没有击败优势
    auto blood_short_and_escape_condition= std::make_shared<roborts_decision::PreconditionNode>("blood_short_and_escape_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GetRemainHP()<=1000 && black_ptr->HurtedPerSecond()>10){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    blood_short_and_escape_condition->SetChild(escape_action); 

    //c9:靠近buff的机器人取buff
    auto near_blood_buff_condition= std::make_shared<roborts_decision::PreconditionNode>("near_blood_buff_condition", black_ptr,
        [&]() {
 		    if (black_ptr->NearBuff()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    near_blood_buff_condition->SetChild(gain_blood_action); 

    //c10:快到buff区刷新时间
    auto near_buff_refresh_time_condition= std::make_shared<roborts_decision::PreconditionNode>("near_buff_refresh_time_condition", black_ptr,
        [&]() {
 		    //if (black_ptr->IsNearRefresh() && black_ptr->OnBuffLocation()){
            if (black_ptr->OnBuffLocation()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    near_buff_refresh_time_condition->SetChild(wait_buff_refresh); 

    //c5:一方子弹不足
    auto one_side_bullet_short_condition = std::make_shared<roborts_decision::PreconditionNode>("one_side_bullet_short_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GetRemainBullets()<20 ){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

//s5
    auto plan_to_gain_bullet_selector=std::make_shared<roborts_decision::SelectorNode>("plan_to_gain_bullet_selector",black_ptr);
    one_side_bullet_short_condition->SetChild(plan_to_gain_bullet_selector);

    //c12:子弹不足，受到攻击且有击败优势
    auto bullet_short_with_advantage_condition= std::make_shared<roborts_decision::PreconditionNode>("bullet_short_with_advantage_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GetRemainBullets()<=20 && black_ptr->HurtedPerSecond()>10 && black_ptr->WithAdvantage()<= -1.0){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    bullet_short_with_advantage_condition->SetChild(swing_action);

    //c14:子弹不足，受到攻击且没有击败优势
    auto bullet_short_and_escape_condition= std::make_shared<roborts_decision::PreconditionNode>("bullet_short_and_escape_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GetRemainBullets()<=20 && black_ptr->HurtedPerSecond()>10 ){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    bullet_short_and_escape_condition->SetChild(escape_action);

    //c13:靠近buff的机器人取buff
    auto near_bullet_buff_condition= std::make_shared<roborts_decision::PreconditionNode>("near_bullet_buff_condition", black_ptr,
        [&]() {
 		    if (black_ptr->NearBuff()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);      

    near_blood_buff_condition->SetChild(gain_bullet_action); 

//s6
    auto actively_approach_selector=std::make_shared<roborts_decision::SelectorNode>("actively_approach_selector",black_ptr);

    //c15:与敌人距离大于2米
    auto actively_chase_condition = std::make_shared<roborts_decision::PreconditionNode>("actively_chase_condition", black_ptr,
        [&]() {
 		    if (!black_ptr->IsEnemyLost() && black_ptr->GetEnemyDistance()>1.7){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_chase_condition->SetChild(chase_action);

    //c16:主动进攻时受到装甲伤害
    auto actively_with_attacked_condition = std::make_shared<roborts_decision::PreconditionNode>("actively_with_attacked_condition", black_ptr,
        [&]() {
 		    if (((ros::Time::now()-black_ptr->GetLastArmorAttackTime()).toSec()<0.5)&&black_ptr->IfTurn()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_with_attacked_condition->SetChild(turn_to_detected_direction);

    //c19:与哨岗失去联系且摄像头看不到敌人
    auto actively_search_condition = std::make_shared<roborts_decision::PreconditionNode>("actively_search_condition", black_ptr,
        [&]() {
 		    if (black_ptr->IsEnemyLost()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_search_condition->SetChild(search_action);

    //c18:主动进攻时受到攻击且无攻击优势
    auto actively_escape_condition = std::make_shared<roborts_decision::PreconditionNode>("actively_escape_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GetRemainHP()<1300 && black_ptr->GetEnemyDistance()<3.0){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_escape_condition->SetChild(escape_action);

    //c17:我方每秒掉血量大于一定值
    auto actively_swing_condition = std::make_shared<roborts_decision::PreconditionNode>("actively_swing_condition", black_ptr,
        [&]() {
 		    if (black_ptr->HurtedPerSecond()>=60){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_swing_condition->SetChild(swing_action);

    //c20:队友受到惩罚且正在遭受攻击
    auto under_pubulish_and_follow_condition = std::make_shared<roborts_decision::PreconditionNode>("chassis_pubnished_condition", black_ptr,
        [&]() {
 		    if (black_ptr->IfTeammatePunished() && black_ptr->TeammateAttacked()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    under_pubulish_and_follow_condition->SetChild(follow_action);   
    actively_approach_selector->AddChildren(actively_chase_condition);
    actively_approach_selector->AddChildren(actively_escape_condition);
    actively_approach_selector->AddChildren(froze_action);

//test chase
    auto back_away_condition = std::make_shared<roborts_decision::PreconditionNode>("back_away_condition", black_ptr,
        [&]() {
 		    if (!black_ptr->IsEnemyLost() && black_ptr->GetEnemyDistance()<1.2){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    actively_chase_condition->SetChild(back_away_action);

//test deck_chase
    auto deck_chase_condition = std::make_shared<roborts_decision::PreconditionNode>("deck_chase_condition", black_ptr,
        [&]() {
 		    if (black_ptr->GetDeck()!= 3 && black_ptr->DeckChangeSlow()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    deck_chase_condition->SetChild(deck_chase_action);
    


    roborts_decision::BehaviorTree root(game_status_selector, 100);
    auto command_thread= std::thread(Command);
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        switch (command) {
            //back to boot area
            case '1':
                game_status_selector->AddChildren(back_boot_area);
                game_status_selector->AddChildren(froze_action);
                root.Run();
                break;
            //chassis punished
            case '2':
                game_status_selector->AddChildren(chassis_limited_action);
                game_status_selector->AddChildren(froze_action);
                black_ptr->PublishGimcontrol();
                root.Run();
                break;
            //gain buff
            case '3':
                game_status_selector->AddChildren(gain_blood_action);
                game_status_selector->AddChildren(froze_action);
                root.Run();
                break;
            //swing defend
            case '4':
                game_status_selector->AddChildren(swing_action);
                game_status_selector->AddChildren(froze_action);
                root.Run();
            //test search
            case '5':
                game_status_selector->AddChildren(search_action);
                game_status_selector->AddChildren(froze_action);
                root.Run();
                break;
            //test chase
            case '6':
                game_status_selector->AddChildren(chase_action);
                game_status_selector->AddChildren(froze_action);
                root.Run();
                break;
            //test escape
            case '7':
                game_status_selector->AddChildren(escape_action);
                game_status_selector->AddChildren(froze_action);
                root.Run();
                break;
            //test wait for refresh
            case '8':
                game_status_selector->AddChildren(near_buff_refresh_time_condition);
                game_status_selector->AddChildren(froze_action);
                root.Run();
                break;
            //test actively_approach_selector 
            case '9':
                game_status_selector->AddChildren(actively_approach_selector);
                game_status_selector->AddChildren(froze_action);
                root.Run();
                break;
            //actively_chase_condition
            case '10':
                game_status_selector->AddChildren(back_away_condition);
                game_status_selector->AddChildren(actively_chase_condition);
                game_status_selector->AddChildren(froze_action);
                root.Run();
                break;
            //deck chase
            case '11':
                game_status_selector->AddChildren(deck_chase_condition);
                game_status_selector->AddChildren(actively_chase_condition);
                game_status_selector->AddChildren(froze_action);
                root.Run();
                break;
            case 27:
                if (command_thread.joinable()){
                    command_thread.join();
                }
                return 0;
            default:
                break;
        }
    rate.sleep();
  }
  return 0;
} //main


void Command() {

  while (command != 27) {//esc
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: test: back boot area" << std::endl
              << "2: test: gimbal swing" << std::endl
              << "3: test: gain buff" << std::endl
              << "4: test: swing defend" << std::endl
              << "5: test: search" << std::endl
              << "6: test: chase" << std::endl
              << "7: test: escape"<<std::endl
              << "8: test: wait for refresh"<<std::endl
              << "9: test: actively_approach_selector"<<std::endl
              << "10: test: actively_chase_condition"<<std::endl
              << "11: test: deck chase" <<std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6'&& command != '7'&& command != '8' && command != '9' && command != '10'  && command != '11' && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}//command