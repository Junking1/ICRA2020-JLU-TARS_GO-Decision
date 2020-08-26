#ifndef ROBORTS_DECISION_SEARCHACTION_H
#define ROBORTS_DECISION_SEARCHACTION_H
#include <unistd.h>      
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../behavior_tree/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../behavior_tree/behavior_node.h"
#include "goal_factory.h"
#include "../executor/chassis_executor.h"
#include <actionlib/client/simple_action_client.h>
#include "roborts_msgs/GimbalSwingAction.h"
namespace roborts_decision{
    class SearchAction : public ActionNode {
        public:
            SearchAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("search_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
                    chassis_executor_ptr_ = blackboard_ptr_->GetChassisExecutor();
                }

            virtual ~SearchAction() = default;

        private:
            std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
            GoalFactory::GoalFactoryPtr goal_factory_ptr_;
            ros::Time start_time_;
            typedef actionlib::SimpleActionClient<roborts_msgs::GimbalSwingAction> Client;

            virtual void OnInitialize() {
                start_time_=ros::Time::now();
                ROS_INFO("%s %s",name_.c_str(),__FUNCTION__);
                blackboard_ptr_->UpdateSearchCount();
            };

            virtual BehaviorState Update() {
                ROS_INFO("start search action");
                ros::Time now_time=ros::Time::now();
                ROS_INFO("search time:%f",(now_time-start_time_).toSec());
                if((now_time-start_time_).toSec()>3.0){
	                goal_factory_ptr_->CancelGoal();
                    blackboard_ptr_->SetSwing(false);
    	            return BehaviorState::SUCCESS;
                }

                goal_factory_ptr_->SearchGoal();
                ROS_INFO("search ！");
                blackboard_ptr_->SetActionState(chassis_executor_ptr_->Update());
                return blackboard_ptr_->GetActionState();
            }

            virtual void OnTerminate(BehaviorState state) {
                switch (state){
                case BehaviorState::IDLE:
                    goal_factory_ptr_->CancelGoal();
                    ROS_INFO("%s %s IDLE!",name_.c_str(),__FUNCTION__);
                break;
                case BehaviorState::SUCCESS:
                    blackboard_ptr_->SetSwing(false);
                    ROS_INFO("%s %s SUCCESS!",name_.c_str(),__FUNCTION__);
                break;
                case BehaviorState::FAILURE:
                    ROS_INFO("%s %s FAILURE!",name_.c_str(),__FUNCTION__);
                break;
                default:
                    ROS_INFO("%s %s ERROR!",name_.c_str(),__FUNCTION__);
                return;
                }
        }

    };// class SearchAction

}//namespace roborts_decision

#endif
//ROBORTS_DECISION_SEARCHACTION_H