#ifndef ROBORTS_DECISION_FROZEACTION_H
#define ROBORTS_DECISION_FROZEACTION_H
#include <unistd.h>      
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../behavior_tree/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../behavior_tree/behavior_node.h"
#include "goal_factory.h"
#include "../executor/chassis_executor.h"

namespace roborts_decision{
class FrozeAction : public ActionNode {
    public:

        FrozeAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
                ActionNode::ActionNode("froze_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
        }
        virtual ~FrozeAction() = default;

    private:
        GoalFactory::GoalFactoryPtr goal_factory_ptr_;

        virtual void OnInitialize() {
            goal_factory_ptr_->CancelGoal();
            ROS_INFO("%s %s",name_.c_str(),__FUNCTION__);
        };

        virtual BehaviorState Update() {
            ROS_INFO("froze");
            return BehaviorState::SUCCESS;
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state){
                case BehaviorState::IDLE:
                    ROS_INFO("%s %s IDLE!",name_.c_str(),__FUNCTION__);
                    break;
                case BehaviorState::SUCCESS:
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
        
    }; //class FrozeAction

}  //namespace roborts_decision


#endif
//ROBORTS_DECISION_FROZEACTION_H
    