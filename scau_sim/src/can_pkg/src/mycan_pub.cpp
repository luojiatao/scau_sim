//样例只是提供一个简单的调用so库的方法供参考，程序接收，与发送函数设置在两个线程中，并且线程没有同步。
//现实中客户编程中，发送与接收函数不能同时调用（不支持多线程），如果在多线程中，一定需要互锁。需要客户自行完善代码。




#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "can_pkg/controlcan.h"
#include <ros/ros.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <geometry_msgs/Twist.h>

#include <turtlesim/Pose.h>
int send_flag = 0;
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	if(send_flag==1)//相当于解锁，开始对接收到的cmd话题进行接收发送为can报文
	{
		// 主循环
		ros::Rate rate(1); // 设置循环频率，这里假设为1Hz
			// 发送当前cmd_vel的CAN报文
		VCI_CAN_OBJ can_msg;
		can_msg.ID = 0x02;  // 设置CAN报文的ID
		can_msg.SendType = 0;
		can_msg.RemoteFlag = 0;
		can_msg.ExternFlag = 0;
		can_msg.DataLen = 8;  // 设置CAN报文的数据长度
		int time=10;//发送次数
			while (time--) 
			{
				//初始化
				for(int i=0;i<8;i++)
				{
					can_msg.Data[i]=0;
				}
			// 将速度信息存储在CAN报文的数据域中
			can_msg.Data[0] = 0x01; //固定为1；
			can_msg.Data[1] = msg->linear.x;//油门 小乌龟速度默认2，实际油门0~100自行缩放
			can_msg.Data[2] = msg->linear.z;//刹车 0~100
			//左转 0~100同油门 小乌龟默认2自行缩放
			if (msg->angular.z>0)
			{
				can_msg.Data[3]=msg->angular.z;
				can_msg.Data[4]=0;
			}
			//右转 0~100同油门 小乌龟默认2自行缩放
			if (msg->angular.z<0)
			{
				can_msg.Data[3]=0;
				can_msg.Data[4]=-(msg->angular.z);
			}


			// 打印接收到的cmd_vel消息
			printf("Received cmd_vel: linear=(%f, %f, %f), angular=(%f, %f, %f)\n",msg->linear.x, msg->linear.y, msg->linear.z,msg->angular.x, msg->angular.y, msg->angular.z);



			// 发送CAN报文
			if (VCI_Transmit(VCI_USBCAN2, 0, 1, &can_msg, 1) != STATUS_OK) 
			{
			printf("Failed to send CAN ");
			}
			else
			{
				printf("CAN message sent successfully\n");
			}
			usleep(300000);//100ms延时
			}
			ros::spinOnce();
			rate.sleep();
	}
   

}

VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1 [50];
int num=0;

//接收线程，接收控制板的can信号
//变量send_flag 1：直线 2：八字 3：寻轨 
void *receive_func(void* param) 
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;
	
	int *run=(int*)param;//线程启动，退出控制。
    int ind_recv=0; //0: CAN1  1:CAN2
	
	while((*run)&0x0f)
	{
		usleep(300000);//延时300ms。
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind_recv,rec,3000,1000))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				// if (rec[j].ID==0x317)
				// {
					printf("Index:%04d  ",count);count++;//序号递增
					printf("CAN%d RX ID:0x%08X", ind_recv+1, rec[j].ID);//ID
					if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
					if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
					if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
					if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
					printf("DLC:0x%02X",rec[j].DataLen);//帧长度
					printf(" data:0x");	//数据
					for(i = 0; i < rec[j].DataLen; i++)
					{
						printf(" %02X", rec[j].Data[i]);
					}
					printf("\n");
					// printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
					if (rec[j].Data[0]==1)
					{
						printf("直线加速解锁");
						send_flag = 1;
					}
					if (rec[j].Data[0]==2)
					{
						printf("八字解锁");
						send_flag = 2;
					}
					if (rec[j].Data[0]==3)
					{
						printf("寻迹解锁");
						send_flag = 3;
					}
	
					printf("\n");
				// }
				
			}
		}		
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

int main(int argc, char** argv)

{
	ros::init(argc, argv, "can_converter");
    ros::NodeHandle nh;
	
	ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("turtle1/cmd_vel", 10, cmdVelCallback);
	
	
	
	printf(">>this is pub !\r\n");//指示程序已运行

	
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		// exit(1);
	}
	

	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x01;/*波特率250 Kbps  0x03  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式		
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,1);
	}
    else
    {
        printf(">>Init CAN1 success\n");
    }


    
	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,1);

	}
    else
    {
        printf(">>Start CAN1 success\n");
    }

    if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init CAN2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
    }
    else
    {
        printf(">>Init CAN2 success\n");
    }
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start CAN2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}
    else
    {
        printf(">>Start CAN2 success\n");
    }
	
	int m_run0=1;
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);//建立接收线程
	
	
	ros::spin();
	//可通过延时终止程序
	// usleep(3000000) //1us延时函数 30s延时

	m_run0=0;//线程关闭指令。
	pthread_join(threadid,NULL);//等待线程关闭。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
	usleep(100000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
	//goto ext;
}
