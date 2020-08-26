#ifndef MUTUAL_BOARD_H_
#define MUTUAL_BOARD_H_

#include "ros/ros.h"
#include "../blackboard/blackboard.h"
#include "../proto/decision.pb.h"
// #include <geometry_msgs/PoseStamped.h>
// #include "roborts_msgs/PunishInfo.h"
// #include "roborts_msgs/RobotInfo.h"
// #include "roborts_msgs/TreeStatus.h"
#include "io/io.h"
#include <memory>

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>


namespace roborts_decision{
// enum _Status{
//     CHASE   = 1,
//     ESCAPE  = 2,
//     SHOOT   = 3,
//     BUFF    = 4
// };

class MutualBoard{
    protected:
        Blackboard::Ptr blackboard_ptr_;
    private:
        // 队友信息话题作用域
        std::string remote_name;     
        // 传出参数列表, 本身参数传递到另一台小车
        int hp_;                                    // 血量
        int bullets_;                               // 单量
        int heat_;                                  // 热量

        bool gimbal_punish_;                        // 云台有没有收到惩罚
        bool chassis_punish_;                       // 地盘是否收到惩罚
        bool is_running_;                           // 是否正在运行
        Status_RunStatus status_;                   // 运行状态
        geometry_msgs::PoseStamped pose_;           // 自身位置
        geometry_msgs::PoseStamped goal_;           // 当前目标

        // 传入参数列表，从队友小车获取来的信息
        int teammate_hp_;
        int teammate_bullets_;
        int teammate_heat_;

        bool teammate_on_gimbal_;
        bool teammate_on_chassis_;
        bool teammate_is_running_;
        Status_RunStatus teammate_status_;
        geometry_msgs::PoseStamped teammate_pose_;
        geometry_msgs::PoseStamped teammate_goal_;

        // 发布者
        ros::Publisher robort_info_pub;
        ros::Publisher punish_pub;
        ros::Publisher status_pub;

        // 订阅者
        ros::Subscriber robort_info_sub;
        ros::Subscriber punish_sub;
        ros::Subscriber status_sub;

        // roborts_decision::Referee topic;            // 裁判系统话题名称
        roborts_decision::MutualInfo exchange;
        roborts_decision::MutualInfo reciver;

        // 声明消息
        roborts_msgs::PunishInfo punish;
        roborts_msgs::RobotInfo robort_msg;
        roborts_msgs::TreeStatus tree_status;

        // SOCKET
        int sock;
        int sock_;
        struct sockaddr_in addr = {0};  
        // struct sockaddr_in remote_addr = {0};
        fd_set read_fd;
        fd_set write_fd;
        fd_set fd;
        struct timeval timeout {0, 33};
        char BUFF[sizeof(roborts_decision::MutualInfo) + 1]{0};
        struct sockaddr_in clnt_addr;
        socklen_t clnt_addr_size = sizeof(clnt_addr);

    public:
        MutualBoard(Blackboard::Ptr& blackboard):
        blackboard_ptr_(blackboard),
        hp_(2000),
        teammate_hp_(2000),
        bullets_(50),
        teammate_bullets_(0),
        heat_(0),
        teammate_heat_(0),
        gimbal_punish_(false),
        teammate_on_gimbal_(false),
        chassis_punish_(false),
        teammate_on_chassis_(false),
        is_running_(true),
        teammate_is_running_(true),
        status_(Status_RunStatus::Status_RunStatus_CHASE),
        teammate_status_(Status_RunStatus::Status_RunStatus_CHASE)
        {
            if(!blackboard_ptr_->IsMaster()){
                bullets_ = 0;
                teammate_bullets_ = 50;
            }

            // 定义发布者和订阅者
            ros::NodeHandle nh(remote_name);
            robort_info_pub = nh.advertise<roborts_msgs::RobotInfo>("robort_info", 30);
            punish_pub = nh.advertise<roborts_msgs::PunishInfo>("punish", 30);
            status_pub = nh.advertise<roborts_msgs::TreeStatus>("status", 30);

            // socket
            SocketInit();
            // TODO
            // 订阅裁判系统



        }
        ~MutualBoard(){
            close(sock_);
            close(sock);
        }

        // 订阅裁判系统和blackboard发布的消息，跟新对本机信息的记录

        // socket初始化
        void SocketInit(){
            sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = inet_addr("127.0.0.1");
            addr.sin_port = htons(6666);
            if(blackboard_ptr_->IsMaster()){
                if(0 > bind(sock_, (struct sockaddr*)&addr, sizeof(addr))){
                    ROS_INFO("bind error ...");
                    exit(-1);
                };
                listen(sock_, 5);
                sock = accept(sock_, (struct sockaddr*)&clnt_addr, &clnt_addr_size);
            }else{
                if(!connect(sock, (struct sockaddr*)&addr, sizeof(addr))){
                    ROS_WARN("connect error .");
                    exit(-1);
                };
            }
            ROS_INFO("mutual node listening ...");
        }
        // socket 收发
        // 收发的主要函数，声明对象后执行此函数
        void ExchangeData(){
            FD_ZERO(&read_fd);
            FD_ZERO(&write_fd);
            FD_SET(sock, &read_fd);
            FD_SET(sock, &write_fd);
            int sr = select(sock+1, &read_fd, &write_fd, NULL, &timeout);
            switch(sr){
                case 0:
                    ROS_INFO("select time out error.");
                    break;
                case -1:
                    ROS_INFO("Select failed.\nTry again...");
                    break;
                default:
                    if (FD_ISSET(sock, &read_fd)){
                        read(sock, BUFF, sizeof(reciver));
                        reciver.ParseFromArray(BUFF, sizeof(reciver));
                        //memcpy(&reciver, BUFF, sizeof(reciver));
                        SetReciver();
                    }
                    if(FD_ISSET(sock, &write_fd)){
                        // 发送自身的信息给队友
                        SetExchange();
                        exchange.SerializeToArray(BUFF, sizeof(exchange));
                        write(sock, BUFF, sizeof(exchange));
                    }
            }
            PublishTeammate();
        }

        // 接受远端数据
        // 获取队友信息
        void SetReciver(){
            teammate_hp_ = reciver.robort_info().hp();
            teammate_bullets_ = reciver.robort_info().bullets();
            teammate_heat_ = reciver.robort_info().heat();

            teammate_on_chassis_ = reciver.punish().on_chassis();
            teammate_on_gimbal_ = reciver.punish().on_gimbal();

            teammate_is_running_ = reciver.status().is_running();
            teammate_status_ = reciver.status().status();
            teammate_goal_.pose.position.x = reciver.status().position().x();
            teammate_goal_.pose.position.y = reciver.status().position().y();
            teammate_goal_.pose.position.z = reciver.status().position().z();// 0
            teammate_goal_.pose.orientation.x = 0;  
            teammate_goal_.pose.orientation.y = 0;
            teammate_goal_.pose.orientation.z = 0;
            teammate_goal_.pose.orientation.w = 0;
        }

        // 调取自身信息, 向外发送的时候执行一次
        void SetExchange(){
            MsgNeedSend();
            auto robort = new RobotInfo;
            auto pun = new PunishInfo;
            auto sts = new Status;
            auto p = new Point;
            robort->set_bullets(bullets_);
            robort->set_heat(heat_);
            robort->set_hp(hp_);

            pun->set_on_chassis(chassis_punish_);
            pun->set_on_gimbal(gimbal_punish_);

            sts->set_is_running(is_running_);
            sts->set_status(status_);

            p->set_x(pose_.pose.position.x);
            p->set_y(pose_.pose.position.y);
            p->set_z(pose_.pose.position.z);
            p->set_yaw(0);
            p->set_pitch(0);
            p->set_roll(0);

            sts->set_allocated_position(p);
            exchange.set_allocated_punish(pun);
            exchange.set_allocated_robort_info(robort);
            exchange.set_allocated_status(sts);
        }

        // 赋值要发布的消息
        void UpdateMsg(){
            punish.on_chassis = teammate_on_chassis_;
            punish.on_gimbal = teammate_on_gimbal_;
            robort_msg.Hp = teammate_hp_;
            robort_msg.Bullet = teammate_bullets_;
            robort_msg.Heat = teammate_heat_;
            robort_msg.RobotPose = teammate_pose_;
            tree_status.is_running = teammate_is_running_;
            tree_status.status = teammate_status_;
            tree_status.goal = teammate_goal_;
        }

        // 将获取到的队友信息发布到本机话题上
        void PublishTeammate(){
            UpdateMsg();
            robort_info_pub.publish(robort_msg);
            punish_pub.publish(punish);
            status_pub.publish(tree_status);
        }

        // 从blackboard里面获取自身的信息
        void MsgNeedSend(){
            hp_ = blackboard_ptr_->GetRemainHP();
            bullets_ = blackboard_ptr_->GetRemainBullets();
            heat_ = blackboard_ptr_->GetCurrentHeat();
            gimbal_punish_ = blackboard_ptr_->GimbalPunished();
            chassis_punish_ = blackboard_ptr_->ChassisPunished();
            pose_ = blackboard_ptr_->GetRobotMapPose();
            goal_ = blackboard_ptr_->GetRobotGoal();
            is_running_ = blackboard_ptr_->IsRunning();
            status_ = (Status_RunStatus)blackboard_ptr_->GetStatus();
        }

        // TODO
        // 回调函数
        // 裁判系统信息
        // 当前状态

};//MutualBoard
}//roborts_decision
#endif  // MUTUAL_BOARD_H_