#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <geometry_msgs/Twist.h>

#include "can_pkg/controlcan.h"
#include <mutex>           //加锁
//多个线程访问同一资源时，为了保证数据的一致性，最简单的方式就是使用 mutex（互斥锁）

//系统为-(CAN2.0 /250k/ID=10)
//send[0].ID =0x0a; Timing0=0x01; Timing1=0x1C   
//[闪灯],[油门],[制动],[左转],[右转],[空],[空],[空]
//版本号：2024-1-15

//分别对两个线程解加锁
std::mutex mtx1;
std::mutex mtx2;

class turn_on_robot
{
	private:
        //创建句柄对象
		ros::NodeHandle nh;

        //订阅话题对象建立
		ros::Subscriber Cmd_Vel_Sub; //初始化话题订阅者

		//速度话题订阅回调函数
		void Cmd_Vel_Callback(const geometry_msgs::Twist &cmd_vel)
        {
            int count = 0;
            VCI_CAN_OBJ  send[1];
            send[0].ID =0x02;          //ID是确定的
            send[0].SendType=0;        //发送帧类型 0:正常发送（发送失败会自动重发，重发最长时间为1.5-3秒）； 1:单次发送（只发送一次，不自动重发）
            send[0].RemoteFlag=0;      //=0时为为数据帧，=1时为远程帧（数据段空）
            send[0].ExternFlag=0;      //=0时为标准帧（11位ID），=1时为扩展帧（29 位ID）。
            send[0].DataLen=8;         //数据长度DLC
            short transition;
            transition = 0;
 
            if (int(cmd_vel.linear.z) != 1)  //y值等于1时结束任务
            {
                send[0].Data[0] = 1; //黄灯闪灯
            }else
            {
                send[0].Data[0] = 2; //蓝灯常亮
            }


            //油门和制动控制选择
            if (cmd_vel.linear.x > 0)
            {
                transition = cmd_vel.linear.x*100;
                send[0].Data[1] = transition; //前进
                send[0].Data[2] = 0;
            }else
            {
                transition = cmd_vel.linear.x*100;
                send[0].Data[1] = 0; //前进
                send[0].Data[2] = -transition;
            }
            //左转和右转控制选择
            if (cmd_vel.angular.z > 0)
            {
                transition = cmd_vel.angular.z*100;
                send[0].Data[3] = transition; 
                send[0].Data[4] = 0;
            }else
            {
                transition = cmd_vel.angular.z*100;
                send[0].Data[3] = 0; 
                send[0].Data[4] = -transition;
            }
            
            //指令空
            send[0].Data[5] = 0;    
            send[0].Data[6] = 0;
            send[0].Data[7] = 0;

            //判断是否发送成功，发送成功则在终端打印数据，否则打印发送失败
            if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)       //第三个参数表示CAN通信索引号，不确定
            {
                std::lock_guard<std::mutex> lock(mtx2);
                printf("Index:%04d  ",count);count++;
                printf("FRAME_ID:0x%02x",send[0].ID);
                if(send[0].ExternFlag==0) printf(" Standard ");
                if(send[0].ExternFlag==1) printf(" Extend   ");
                if(send[0].RemoteFlag==0) printf(" Data   ");
                if(send[0].RemoteFlag==1) printf(" Remote ");
                //printf("DLC:%d",send[0].DataLen);
                printf(" data:0x");
                int i;
                i = 0;
                for(i=0;i<send[0].DataLen;i++)
                {
                printf(" %02X",send[0].Data[i]);
                }
                printf("\n");
            }else
            {
                printf(">>send data error!\n");
            }          
        }

	public:
        //Constructor 构造函数
		turn_on_robot()
        {
            int num = 0;
            VCI_BOARD_INFO pInfo;
            VCI_BOARD_INFO pInfo1 [50];
            num=VCI_FindUsbDevice2(pInfo1);
            VCI_INIT_CONFIG config;
            config.AccCode=0x20200000;//只接受帧ID为0x101
            config.AccMask=0x00000000;//范围0
            config.Filter=1;        //接收所有帧，0/1接受所有类型，2只接受标准帧，3只接受拓展帧
            config.Timing0=0x01;    //波特率250Kbps时，Timing0=0x01，Timing1=0x1C
            config.Timing1=0x1C;    //波特率500Kbps时，Timing0=0x00，Timing1=0x1C
            config.Mode=0;          //0-正常模式，1-监听模式，2-自发自收模式

            //判断CAN设备是否成功开启
            if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
            {
                printf(">>Open Deivce Success!\n");//打开设备成功
            }else
            {
                printf(">>Open Deivce Error!\n");
                exit(1);
            }
            usleep(5000);

            //判断can是否初始化成功
            if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)//返回值为1表示初始化成功，0表示操作失败，-1表示设备不存在
            {
                printf(">>Init CAN1 error!\n");
                VCI_CloseDevice(VCI_USBCAN2,0);
            }else
            {
                printf(">>Init CAN1 Success! \n");
            }//判断是否启用can0通道成功
            usleep(5000);

            if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
            {
                printf(">>Start CAN1 error!\n");
                VCI_CloseDevice(VCI_USBCAN2,0);
            }else
            {
                printf(">>Start CAN1 Success!\n");
            }
            //ros::NodeHandle n;

        }

        //持续订阅cmd_vel并输出can控制指令
		void can_out()  
        {
            Cmd_Vel_Sub = nh.subscribe("cmd_vel", 100, &turn_on_robot::Cmd_Vel_Callback, this);
            ros::spin();
        }

        //析构函数
        ~turn_on_robot()
        {
        VCI_CAN_OBJ  send[1];
        send[0].ID =0x0a;
            send[0].SendType=0;
            send[0].RemoteFlag=0;
            send[0].ExternFlag=1;
            send[0].DataLen=8;
            send[0].Data[0] = 0;
            send[0].Data[1] = 0;
            send[0].Data[2] = 0;
            send[0].Data[3] = 0;
            send[0].Data[4] = 0;
            send[0].Data[5] = 0;
            send[0].Data[6] = 0;
            send[0].Data[7] = 0;
            usleep(10000);
            if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) ==1)
            {		
                int i;
                i = 0;
                for(i=0;i<send[0].DataLen;i++)
                {
                    printf(" %02X",send[0].Data[i]);
                }
                printf("\n");
            }
            usleep(10000);
            if(VCI_CloseDevice(VCI_USBCAN2,0)==1)//关闭设备。
            {
                printf("Close Device Success!\n");
            }else
            {
                printf("Close Device Error!\n");
            }
        }        


};

int main(int argc,char** argv)
{
   //设置编码，避免中文乱码
   setlocale(LC_ALL,"");
   
   ros::init(argc, argv, "serial_can"); 

   turn_on_robot Robot_Control; //实例化一个对象

   Robot_Control.can_out();//不放在构造体内才能实现一直检测会不会断连can仪器
  
   ros::spin();

   return 0;  
} 