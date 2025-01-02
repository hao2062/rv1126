/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: icm20608App.c
作者	  	: 正点原子Linux团队
版本	   	: V1.0
描述	   	: icm20608设备测试APP。
              通过 ioctl 系统调用和设备驱动通信，实现对设备的控制和数据读写
其他	   	: 无
使用方法	 ：./icm20608App /dev/icm20608
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2021/03/22 正点原子Linux团队创建
***************************************************************/

// 头文件
#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <fcntl.h>  
#include <unistd.h>  
#include <linux/videodev2.h>
#include <linux/types.h>
#include <stdbool.h>

// 宏定义
#define  SET_POS  1		// 设置位置
#define  SET_VD  4		// 设置 VD
#define  WRITE  2		// 写寄存器
#define  READ  3		// 读寄存器
#define RK_VIDIOC_IRIS_CORRECTION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 6, unsigned int)		// 光耦修正
#define RK_VIDIOC_ZOOM_CORRECTION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 7, unsigned int)		// 变焦修正 zoom
#define RK_VIDIOC_ZOOM1_CORRECTION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 12, unsigned int)		// 变焦修正 zoom1
#define RK_VIDIOC_WRITE_REG \
	_IOW('V', BASE_VIDIOC_PRIVATE + 17, struct reg_struct)	// 写寄存器
#define RK_VIDIOC_READ_REG \
	_IOW('V', BASE_VIDIOC_PRIVATE + 18, struct reg_struct)	// 读寄存器
#define RK_VIDIOC_FOCUS_CORRECTION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 5, unsigned int)

/*
 * @description		: main主程序
 * @param - argc 	: argv数组元素个数
 * @param - argv 	: 具体参数
 * @return 			: 0 成功;其他 失败
 */

// 寄存器操作结构体
struct reg_struct{
	int address;	// 寄存器地址
	int args;		// 寄存器值
};

// 主函数入口：argc - 命令行参数个数；argv - 命令行参数数组
int main(int argc, char *argv[])
{ 
	// 定义结构体，用于读写寄存器
	struct reg_struct reg={
		.address=0,
		.args=0
	};

	// 定义变量
	int fd;		// 文件描述符
	int pos=0; 	// pos，用于设置镜头位置
	int cmd;	// 命令编号，从命令行参数解析
	int std=1;	// 未使用的变量
	char *filename;	// 设备文件路径 ( /dev/v4l-subdev10 )
	signed int databuf[7];	
	unsigned char data[14];
	int ret = 0;

	// 解析命令行参数
	cmd= atoi(argv[2]);			// atoi：把字符串转成整数。argv[2]：命令行的第三个参数（1：./pirisApp 2：/dev/v4l-subdev10 3：0）
								// cmd 为要执行的命令编号
	printf("cmd:%d\n",cmd);		// 打印编号：cmd：0/1
	
	// 确认命令行参数为 4 个：0-程序名 1-设备文件路径 2-命令编号 3-命令参数
	if (argc == 4) {		
		
		// 提取位置参数：argv[3] 转成整数
		pos=atoi(argv[3]);	
		
		// 打开设备文件
		filename = argv[1];	
		fd = open(filename, O_RDWR);
		if(fd < 0) {
			printf("can't open file %s\r\n", filename);
			return -1;
		}
    	
		// 根据 cmd 的值，执行不同功能

		// 寻找光耦
		if(cmd==0){
			// if (ioctl(fd, RK_VIDIOC_IRIS_CORRECTION, NULL) == -1) {  
			if (ioctl(fd, RK_VIDIOC_FOCUS_CORRECTION, NULL) == -1) {  
			} 
		}  
		
		// 设置位置
		if(cmd==1){
			if (ioctl(fd, SET_POS, &pos) == -1) {  
				// 错误处理
			}  
		}

		//设置 VD 信号
		if(cmd==2){
			if (ioctl(fd, SET_VD, NULL) == -1) {  
    		}  
		}
	} else if (argc == 5) {		// 判断参数是否为 5 ？
		// 提取寄存器地址和值（从 命令行输入中）
		// 将命令行输入参数解析为 16 进制，并存到 address 和 args 里
		sscanf(argv[3], "%x", &reg.address) ;
		sscanf(argv[4], "%x", &reg.args) ;
		
		// 打印输入的寄存器地址和值
		printf("地址%x\n",reg.address);
		printf("值%x\n",reg.args);
		
		// 打开设备文件
		filename = argv[1];
		fd = open(filename, O_RDWR);
		if(fd < 0) {
			printf("can't open file %s\r\n", filename);
			return -1;
		}

    	// 写寄存器
		if(cmd==3){
			if (ioctl(fd, RK_VIDIOC_WRITE_REG, &reg) == -1) {  
    		}	 
		}  
		
		//读寄存器
		if(cmd==4){
			if (ioctl(fd, RK_VIDIOC_READ_REG , &reg) == -1) {  
    		}  
		}

	}
	
	
	close(fd);	/* 关闭文件 */	
	return 0;
}

