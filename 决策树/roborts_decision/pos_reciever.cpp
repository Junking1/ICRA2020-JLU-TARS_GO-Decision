#include "ros/ros.h"
#include <sys/socket.h>
#include "std_msgs/String.h"
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netinet/in.h>
#include "roborts_msgs/SentryInfo.h"

struct PoseFromSock{
	double x1 = 0;
	double y1 = 0;
	double x2 = 0;
	double y2 = 0;
	
};

int main(int argc, char **argv){
	ros::init(argc, argv, "pos_reciver_node");
	ros::NodeHandle nh;
	// ros::Publisher pos_trans_pub = nh.advertise<std_msgs::String>("pos_from_sentry", 1000);
	//-----------------------------right???
	// 已修改
	ros::NodeHandle sentry_nh;
	ros::Publisher enemy_pose_pub = sentry_nh.advertise<roborts_msgs::SentryInfo>("enemy_pose_from_sentry", 30);

	ros::Rate loop_rate(30);
	int sock = socket(AF_INET, SOCK_STREAM, 0);
	struct sockaddr_in servAddr;
	memset(&servAddr, 0, sizeof(servAddr));
	servAddr.sin_family = AF_INET;
	// TODO ip和端口记得修改
	servAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	servAddr.sin_port = htons(2222);
	
	// bool can_view = false;
	roborts_msgs::SentryInfo sentry_info;
	sentry_info.can_view = false;

	// connecting
	try{
		int con_count = 3;
		while(con_count){
			if(!connect(sock, (struct sockaddr*)&servAddr, sizeof(servAddr))){
				printf("connected successfull !");
				break;
			}
			con_count--;
		}
		if (!con_count) throw std::exception();
	}
	catch(std::exception& e){
		ROS_INFO("connecting error.\n");
		exit(-1);
	}
	
	int buffsize = sizeof(PoseFromSock) / sizeof(char);
	char revBuff[sizeof(PoseFromSock)/sizeof(char)+1];
	struct PoseFromSock pose;
	memset(&pose, 0, sizeof(pose));
	while(ros::ok()){
		try{
			if (read(sock, revBuff, sizeof(revBuff)-1) < 0) throw std::exception();	
			memcpy(&pose, revBuff, sizeof(pose));
			printf("recived position from sentry:\n");
			printf("pos NO.1 x= %lf, y= %lf \n", pose.x1, pose.y1);
			printf("pos NO.2 x= %lf, y= %lf \n\n", pose.x2, pose.y2);
			sentry_info.x1 = pose.x1;
			sentry_info.x2 = pose.x2;
			sentry_info.y1 = pose.y1;
			sentry_info.y2 = pose.y2;
			sentry_info.can_view = true;
		}
		catch(std::exception& ee){
			printf("read error once.");
			sentry_info.can_view = false;
			sentry_info.x1 = -1;
			sentry_info.x2 = -1;
			sentry_info.y1 = -1;
			sentry_info.y2 = -1;
			continue;
		}
		enemy_pose_pub.publish(sentry_info);
		ros::spinOnce();
		loop_rate.sleep();
	}

	
	return 0;

}
