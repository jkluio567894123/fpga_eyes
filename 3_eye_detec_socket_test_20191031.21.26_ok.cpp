	#include<opencv2/core/core.hpp>
	#include<opencv2/highgui/highgui.hpp>
	#include<opencv2/opencv.hpp>
	#include<iostream>

	#include "iostream"
	#include "string.h"
	#include "fstream"

	#include "stdio.h"
	#include <opencv2/opencv.hpp>
	#include <string>
	
	#include "sys/types.h"
	#include "sys/socket.h"
	#include "sys/un.h"
	#include "unistd.h"
	#define UNIX_DOMAIN "/mnt/UNIX.domain"

#define PIC_WIDTH 640
#define PIC_HIGHT 480
#define PIC_Pix_Num (PIC_WIDTH*PIC_HIGHT)

#define img_size_4th (PIC_WIDTH*PIC_HIGHT/4)	//soc每次传递的像素的个数
	
#define img_size 76800

	using namespace std;
	using namespace cv;

		int img_number=0;

	class ReadAndWriteFile
	{

	public:
		bool ReadFile2Array(string fileName, int* readData, int* Datalength);
		Mat Array2Mat(int* a, const char* str);
		int RGB565ToRGB888(int n565Color);
		int RGBTOGray(int n565Color);
	};

	Mat ReadAndWriteFile::Array2Mat(int* a, const char* str)
	{
		Mat M(480, 640, CV_8UC1, a);
		for (int i = 0; i < M.rows; ++i)
		{
		        uchar* p = M.ptr<uchar>(i);
		        for (int j = 0; j < M.cols; ++j)
		                p[j] = a[i * 640 + j];
		}
//		if(img_number==5)
//			imwrite(str, M);
		return M;
	}
	int ReadAndWriteFile::RGB565ToRGB888(int n565Color)
	{
		int n888Color = 0;      
		unsigned char cRed   = (n565Color & 0xF800)     >>    8;
		unsigned char cGreen = (n565Color & 0x07E0)     >>    4;
		unsigned char cBlue  = (n565Color & 0x001F)     <<    3;

		n888Color = (cRed * 299 + cGreen * 587 + cBlue * 114 + 500) / 1000;
		return n888Color;
	}

int main()
{
	  int ReadDate[640 * 480];
	  socklen_t clt_addr_len;
	  int listen_fd;
	  int com_fd;
	  int ret;
	  int i,j;
	  int k=0;
	  static short int recv_buf[img_size_4th];
	  int snd_buf[100]={0};
	  int len;
          int num=0;
	  int face_num=0;
	  int eyes_num=0;
	  int snd_buf_pos=0;
	  struct sockaddr_un clt_addr;
	  struct sockaddr_un srv_addr;		

			cv::CascadeClassifier face_cascade;
			char* face_cascade_name = ("haarcascade_frontalface_default.xml");

			cv::CascadeClassifier eyes_cascade;
			char* eyes_cascade_name  = ("haarcascade_eye.xml");

	
			if (!face_cascade.load(face_cascade_name))
				printf("--(!)Error loading face cascade\n");
			else
				printf("load finish\n");

			if (!eyes_cascade.load(eyes_cascade_name))			
				printf("--(!)Error loading eyes cascade\n");
			else
				printf("load finish\n");	

	  listen_fd=socket(PF_UNIX,SOCK_STREAM,0);
	  if(listen_fd<0)
	  {
		perror("cannot create communication socket");
		return 1;
	  }

	  srv_addr.sun_family=AF_UNIX;
	  strncpy(srv_addr.sun_path,UNIX_DOMAIN,sizeof(srv_addr.sun_path)-1);
	  unlink(UNIX_DOMAIN);

	  ret=bind(listen_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
	  if(ret==-1)
	  {
		perror("cannot bind server socket");
		close(listen_fd);
		unlink(UNIX_DOMAIN);
		return 1;
	  }

	  ret=listen(listen_fd,1);
	  if(ret==-1)
	  {
		perror("cannot listen the client connect request");
		close(listen_fd);
		unlink(UNIX_DOMAIN);
		return 1;
	  }


	  com_fd=accept(listen_fd,NULL,NULL);
	  if(com_fd<0)
	  {
		perror("cannot accept client connect request");
		close(listen_fd);
		unlink(UNIX_DOMAIN);
		return 1;
	  }
		

	  while(1)
	  {
		for(i=0;i<4;i++)
		{
		  memset(recv_buf,0,2*img_size_4th);
		  num=recv(com_fd,recv_buf,img_size_4th*sizeof(short int),MSG_WAITALL);//change read to recv (and add MSG_WAITALL)
		  printf("num=%d\n",num);
		 
		  for(k=0;k<img_size_4th;k++)
		  {
			ReadDate[i*img_size_4th+k]=(int)recv_buf[k];
		  }
//		  snd_buf[i]=i+1;
//		  write(com_fd,snd_buf,sizeof(snd_buf));
		}
		img_number++;

		ReadAndWriteFile rd;

		string fileName;
		string outfileName;
		outfileName = "rectangle_place.txt";
		ofstream outFile;

		for (int i = 0; i < 307200; i++)
		{
			ReadDate[i] = rd.RGB565ToRGB888(ReadDate[i]);
		}

		Mat img_gray_detect = rd.Array2Mat(ReadDate, "eye.jpg");

		std::vector<cv::Rect>   faces;
		face_cascade.detectMultiScale(img_gray_detect, faces, 1.3, 5, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
	
		if(faces.size()==0)
			cout << "no face" <<endl;
		else
		{
				outFile.open(outfileName);

				if (!outFile.is_open())
				{
					std::cout << "打开文件失败 \n" << endl;
				}
		}

                //jian ce niande shumu 
		
		memset(snd_buf,0,sizeof(snd_buf));

                face_num=faces.size();
                snd_buf_pos=0;
		snd_buf[snd_buf_pos]=face_num;

		for (int i = 0; i < face_num; i++)
		{
			cout << "faces number :"<< i+1 <<endl;
			cout << faces[i]<<endl;
				

			cv::Mat faceROI = img_gray_detect(faces[i]);

			std::vector<cv::Rect> eyes;
			eyes_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
			eyes_num=eyes.size();
			// fa nian 
			snd_buf_pos++;
			snd_buf[snd_buf_pos]=10000*(i+1);
			snd_buf_pos++;
			snd_buf[snd_buf_pos]=faces[i].width;
			snd_buf_pos++;
			snd_buf[snd_buf_pos]=faces[i].x;
			snd_buf_pos++;
			snd_buf[snd_buf_pos]=faces[i].y;
			//fa eye
			eyes_num=eyes.size();
			for(j = 0; j < eyes_num; j++)
			{
				snd_buf_pos++;
				snd_buf[snd_buf_pos]=10000*(i+1)+1+j;
				snd_buf_pos++;
				snd_buf[snd_buf_pos]=eyes[j].width;
				snd_buf_pos++;
				snd_buf[snd_buf_pos]=eyes[j].x;
				snd_buf_pos++;
				snd_buf[snd_buf_pos]=eyes[j].y;
			}
				

		
			if(eyes_num==0)
				cout << "no eyes" <<endl;			

			for(int j = 0; j < eyes_num; j++)
			{
			        cout << "eyes number :"<< j+1 <<endl;
			        cout << eyes[j]<<endl;

			}

		}
		write(com_fd,snd_buf,sizeof(snd_buf));
	  }
		close(com_fd);
		  close(listen_fd);
		 unlink(UNIX_DOMAIN);


	    return 0;
}

/*
 * sim900a.h
 *
 *  Created on: 2019年10月25日
 *      Author: Lenovo
 */

#ifndef SIM900A_H_
#define SIM900A_H_


//gcc标准头文件
#include <stdio.h>
//#include <unistd.h>
//#include <fcntl.h>
//#include <sys/mman.h>
//#include <string.h>
//#include "uart_gprs.h"
//hps 厂家提供的底层定义头文件
//#define soc_cv_av	//定义使用soc_cv_av硬件平台

//#include "hwlib.h"
//#include "socal/socal.h"
//#include "socal/hps.h"

//与用户具体HPS应用系统相关的硬件描述头文件
//#include "hps_0.h"

//串口字符发送函数
void uart_putc(char c) {
	unsigned short uart_status;	//状态寄存器值
	do {
		uart_status = *(uart_0_virtual_base + 2);	//读取状态寄存器
	} while (!(uart_status & 0x40));	//等待状态寄存器bit6（trdy）为1

	*(uart_0_virtual_base + 1) = c;	//发送一个字符
}

//串口字符串发送函数
void uart_printf(char *str) {
	while (*str != '\0')	//检测当前指针指向的数是否为空字符
	{
		uart_putc(*str);	//发送一个字符
		if(*(str+1)=='\0'){
			uart_putc('\0');}
		str++;	//字符串指针+1
	}
}

//串口字符接收函数
int uart_getc(void) {
	unsigned short uart_status;	//状态寄存器值
	do {
		uart_status = *(uart_0_virtual_base + 2);	//读取状态寄存器
	} while (!(uart_status & 0x80));	//等待状态寄存器bit7（rrdy）为1

	return *(uart_0_virtual_base + 0);	//读取一个字符并作为函数返回值返回
}

//串口字符串接收函数
int uart_scanf(char *p) {
	int cnt = 0;	//接收个数计数器
	while (1) {
		*p = uart_getc();	//读取一个字符的数据
		cnt++;
		if (*p == '\n')	//判断数据是否为换行
			return cnt;	//换行则停止计数,返回当前接收的字符个数
		else
			p++;	//接收指针增1
	}
}

void fasong(void) {

	*(uart_0_virtual_base + 4) = (int) (UART_0_FREQ / 115200 + 0.5);

	//uart_putc(0x0);
	uart_printf("AT+CSCS=\"GSM\"\r\n");
	usleep(20000);
	uart_printf("AT+CMGF=1\r\n");
	usleep(20000);
	uart_printf("AT+CMGS=\"15171022951\"\r\n");
	usleep(20000);
	uart_printf("blink_times\r\n");
	//uart_printf((char)m);
	//uart_printf("");
	usleep(20000);
	uart_putc(0x1a);
}





#endif /* SIM900A_H_ */

#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <dirent.h>
#include <inttypes.h>
#include <sys/time.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include "math.h"
#include "hps_0.h"
#include  "./OV7670/ov5640.h"
#include <linux/fb.h>
#include <pthread.h>

#include "stdio.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "sys/un.h"
#define UNIX_DOMAIN "/mnt/UNIX.domain"
#define UNIX_DOMAIN2 "/mnt/UNIX2.domain"

#define MY_CDEV_MAGIC 'k'
#define MY_CDEV_CMD  _IO(MY_CDEV_MAGIC,0x1a)
#define MY_CDEV_CMD_READ  _IOR(MY_CDEV_MAGIC,0x1b,int)
#define DEVICE_MAJOR 205


#define HW_REGS_BASE (ALT_STM_OFST )
#define HW_REGS_SPAN (0x04000000 )
#define HW_REGS_MASK (HW_REGS_SPAN - 1 )
#define BMP_WIDTH 640
#define BMP_HEIGHT 480
#define WRITE_COUNT_PILXEL 2
#define BMP_COUNT_PILXEL 3
#define BUFFER_SIZE BMP_WIDTH*BMP_HEIGHT*2
#define BMP_BUFFER_SIZE BMP_WIDTH*BMP_HEIGHT*BMP_COUNT_PILXEL
#define N_SIGNAL 20

volatile unsigned long *h2p_dvp_capture_addr = NULL;
volatile unsigned long *h2p_uart_addr = NULL;
volatile unsigned long *h2p_oc_iic_0_addr = NULL;
volatile unsigned long *h2p_dipsw_addr = NULL;
volatile unsigned long *h2p_pwm0_addr = NULL;
volatile unsigned long *h2p_pwm1_addr = NULL;
volatile unsigned long *h2p_pwm2_addr = NULL;
volatile unsigned long *h2p_pwmlsj0_addr = NULL;
volatile unsigned long *h2p_pwmlsj1_addr = NULL;
volatile unsigned long *h2p_pwmlsj2_addr = NULL;
volatile unsigned long *h2p_pio0_output_addr=NULL;
volatile unsigned long *h2p_ADC_LTC2308_ly_addr=NULL;
volatile unsigned long *h2p_pio_timel_addr=NULL;
volatile unsigned long *h2p_pio_timeh_addr=NULL;//
volatile unsigned long *h2p_pio_led_addr=NULL;
volatile unsigned long *h2p_uart_gprs_addr=NULL;
volatile unsigned long *uart_0_virtual_base = NULL;	//uart_0虚拟地址


struct  GradeTime//用于线程2和线程3通信，线程2每0.25秒钟分析一次烟叶并给出烟叶等级，将分析等级和时间写进该机构体。
{
	int grade[N_SIGNAL];
	unsigned int time[N_SIGNAL];
};
struct GradeTime gradetime;
int GRADE;
//char shuzi[10];
//unsigned int cnt=0;



#define ALT_UART_BASE UART_0_BASE
int capture_init(void)
{
	int fd;
	unsigned long dma_base;
	fd = open("/dev/hbmy_cmos_cap", O_RDWR);
	if (fd == 1) {
		perror("open error\n");
		exit(-1);
	}
	ioctl(fd,MY_CDEV_CMD,&dma_base);

	printf("dma_base is %ld\n",dma_base);
	h2p_dvp_capture_addr[1] = 640*480*2;	//写入捕获的数据长度
	h2p_dvp_capture_addr[2] = dma_base;		//指定图像存储的位置，指定在驱动申请的DMA指向的存储区的首地址
	h2p_dvp_capture_addr[0] = 1;			//使能图像捕获
	return fd;
}
//-----------------  初始化各种外设的虚拟地址指针  -------------------------//
int fpga_ip_init()
{
	int fd;
	void *lw_axi_virtual_base;
	/*开始系统初始化*/
	printf("start app\n");
	if ((fd = open("/dev/mem", ( O_RDWR | O_SYNC))) == -1)
	{
		printf("ERROR: could not open \"/dev/mem\"...\n");
		return (1);
	}
	lw_axi_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE),MAP_SHARED, fd, HW_REGS_BASE);
	if (lw_axi_virtual_base == MAP_FAILED)
	{
		printf("ERROR: mmap() failed...\n");
		close(fd);
		return (1);
	}

	h2p_dvp_capture_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + DVP_CAP_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_uart_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + ALT_UART_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_oc_iic_0_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + OC_IIC_0_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_dipsw_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + DIPSW_PIO_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_pwmlsj0_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + LSJ_PWM0_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_pwmlsj1_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + LSJ_PWM1_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_pwmlsj2_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + LSJ_PWM2_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_pio0_output_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + PIO_0_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_ADC_LTC2308_ly_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + ADC_LTC2308_0_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_pio_timel_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + PIO_TIMEL_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_pio_timeh_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + PIO_TIMEH_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_pio_led_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + LED_PIO_BASE) & (unsigned long) ( HW_REGS_MASK)));
	h2p_uart_gprs_addr = (lw_axi_virtual_base + ((unsigned long) ( ALT_LWFPGASLVS_OFST + UART_1_BASE) & (unsigned long) ( HW_REGS_MASK)));
	uart_0_virtual_base = (lw_axi_virtual_base+ ((unsigned long) ( ALT_LWFPGASLVS_OFST + UART_0_BASE)& (unsigned long) ( HW_REGS_MASK)));
	return fd;//LED_PIO_BASE
}

typedef struct fbdev{
    int fdfd; //open "dev/fb0"
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    long int screensize;
    char *map_fb;
}FBDEV;

void init_dev(FBDEV *dev)
{
    FBDEV *fr_dev=dev;
    fr_dev->fdfd=open("/dev/fb0",O_RDWR);
    printf("the framebuffer device was opended successfully.\n");
    ioctl(fr_dev->fdfd,FBIOGET_FSCREENINFO,&(fr_dev->finfo)); //获取 固定参数
    ioctl(fr_dev->fdfd,FBIOGET_VSCREENINFO,&(fr_dev->vinfo)); //获取可变参数
    fr_dev->screensize=fr_dev->vinfo.xres*fr_dev->vinfo.yres*fr_dev->vinfo.bits_per_pixel/8;
    fr_dev->map_fb=(char *)mmap(NULL,fr_dev->screensize,PROT_READ|PROT_WRITE,MAP_SHARED,fr_dev->fdfd,0);
    printf("init_dev successfully.\n");
}

void draw_pix(FBDEV *dev,int x,int y, unsigned short rgb) //(x.y) 是坐标
{
    FBDEV *fr_dev=dev;
    int *xx=&x;
    int *yy=&y;
    unsigned char r,g,b;
    unsigned int rgb24;
    long int location=0;
    location=(*xx+fr_dev->vinfo.xoffset)*(fr_dev->vinfo.bits_per_pixel/8)+(*yy+fr_dev->vinfo.yoffset)*fr_dev->finfo.line_length;
    r = (rgb & 0xf800) >> 11;
    g = (rgb & 0x07e0) >> 5;
    b = rgb & 0x1f;
    rgb24 = ((r<<3<<16) | (g << 2 << 8) | (b << 3));
    *((unsigned int *)(fr_dev->map_fb+location))=rgb24;
}

void draw_pic(FBDEV *dev,int x1,int y1,int x2,int y2,unsigned short *p)
{
    FBDEV *fr_dev=dev;
    int *xx1=&x1;
    int *yy1=&y1;
    int *xx2=&x2;
    int *yy2=&y2;
    int i=0,j=0;
    for(j=*yy1;j<*yy2;j++) //注意 这里要 xx1 < xx2
        for(i=*xx1;i<*xx2;i++)
        {
    		draw_pix(fr_dev,i,j,*p);
    		p++;
        }
}

//近似的灰度处理
void huiduchuli(unsigned short *p,unsigned short *white_ly)
{
	unsigned int i,j;
	unsigned char r,g,b;
	for (i = 0; i < 480; i++)
		{
			for (j = 0; j < 640; j++)
			{
			  r = (*p & 0xf800) >> 11; //取*p的11到15bit，共5bit，最大值为32
			  g = (*p & 0x07e0) >> 5;  //取*p的5到10bit，共6bit，最大值为64
			  b = *p & 0x1f;		   //取*p的0到4bit，共5bit，最大值为32
              r =  r*0.299 ;
              g=   g*0.587;
              b =  b*0.114 ;
			  *white_ly = ((r<<11) | (g <<5) | (b))+0.5;//把白色变成黑色后的指针数组。
			  p++;	//图片转到下一个像素点，及指针地址加一
			  white_ly++;
			}
		}
}



//通过闪烁来展示是否眨眼
unsigned short led_flash(unsigned short *p,unsigned int x,unsigned int y)
{
	unsigned int i,j;
	unsigned char r,g,b;
	unsigned int cnt=0;
	static int n,m;
	for (i = 0; i < 480; i++)
		{
			for (j = 0; j < 640; j++)
			{
		      r = (*p & 0xf800) >> 11; //取*p的11到15bit，共5bit，最大值为32
			  g = (*p & 0x07e0) >> 5;  //取*p的5到10bit，共6bit，最大值为64
			  b = *p & 0x1f;		   //取*p的0到4bit，共5bit，最大值为32
			  if((i==x)&&(j==y))
			  {
				  if((r==0)&&(g==0)&&(b==0))
				  {
					 m=n;
					 n=1;
				  }
				  else
				  {
					  m=n;
					  n=0;
				  }
			  }
			  p++;	//图片转到下一个像素点，及指针地址加一
		    }
		}
	if(n==1&&m==1)
		cnt=1;
	return cnt;
}

//对规定区域的二值化
void erzhihua(unsigned short *p,unsigned int x,unsigned int y,unsigned int l)
{
	unsigned int i,j;
	unsigned char r,g,b;
	x=x+(3*l/8);
	y=y+(3*l/8);
	l=l/4;
	for (i = 0; i < 480; i++)
		{
			for (j = 0; j <640; j++)
			{
			  r = (*p & 0xf800) >> 11; //取*p的11到15bit，共5bit，最大值为32
			  g = (*p & 0x07e0) >> 5;  //取*p的5到10bit，共6bit，最大值为64
			  b = *p & 0x1f;		   //取*p的0到4bit，共5bit，最大值为32
			  if((j>x)&&(j<x+l)&&(i>y)&&(i<y+l))
			  {

				  if((r>10)||(g>11)||(b>12))
				  {
					r =255 ;
					g= 255 ;
					b =255 ;
				  }
				  else
				  {
					r =0 ;
					g= 0 ;
					b =0 ;
				  }
			  }
			  *p = ((r<<11) | (g <<5) | (b));//把白色变成黑色后的指针数组。
			  p++;	//图片转到下一个像素点，及指针地址加一
			}
		}
}


void draw_pic_gh(FBDEV *dev,int rec_buf[])
{
	FBDEV *fr_dev=dev;
    int  i=0;
    int l,x,y,x1,y1,x2,y2;
    unsigned char r,g,b;
    int k=0;
	for(i=1;i<100;i++)
	{
		if(rec_buf[i]%10000==0)
		{
			l=rec_buf[i+1];
			x=rec_buf[i+2];
			y=rec_buf[i+3];
			r = 0;g = 0;b = 255;
			x1=x;
			y1=y;
			x2=x1+l;
			y2=y1+l;
		    int *xx1=&x1;
		    int *yy1=&y1;
		    int *xx2=&x2;
		    int *yy2=&y2;
			int n=0,m=0;
				for(n=*yy1;n<=*yy2;n++) //注意 这里要 xx1 < xx2
					for(m=*xx1;m<=*xx2;m++)
					{
						if(((m==*xx1)&&(n>=*yy1&&n<=*yy2))||((m==*xx2)&&(n>=*yy1&&n<=*yy2))||((n==*yy1)&&(m>=*xx1&&m<=*xx2))||((n==*yy2)&&(m>=*xx1&&m<=*xx2)))
						{
							draw_pix2(fr_dev,m,n,r,g,b);
						}
					}
		}
		else
		{
			if(rec_buf[i]>10000)
			{
				l=rec_buf[i+1];
				x1=rec_buf[i+2]+x;
				y1=rec_buf[i+3]+y;
				r = 0;g = 255;b = 0;
				x2=x1+l;
				y2=y1+l;
				int *xx1=&x1;
				int *yy1=&y1;
				int *xx2=&x2;
				int *yy2=&y2;
				int n=0,m=0;
					for(n=*yy1;n<=*yy2;n++) //注意 这里要 xx1 < xx2
						for(m=*xx1;m<=*xx2;m++)
						{
							if(((m==*xx1)&&(n>=*yy1&&n<=*yy2))||((m==*xx2)&&(n>=*yy1&&n<=*yy2))||((n==*yy1)&&(m>=*xx1&&m<=*xx2))||((n==*yy2)&&(m>=*xx1&&m<=*xx2)))
								{
								  draw_pix2(fr_dev,m,n,r,g,b);
								}
						}
			}
		}
			if(rec_buf[i]==0)
		    {
		    	i=100;
		    }
	}
}
void draw_pix2(FBDEV *dev,int x,int y,unsigned char r,unsigned char g,unsigned char b) //(x.y) 是坐标
{
    FBDEV *fr_dev=dev;
    int *xx=&x;
    int *yy=&y;
    unsigned int rgb24;
    long int location=0;
    location=(*xx+fr_dev->vinfo.xoffset)*(fr_dev->vinfo.bits_per_pixel/8)+(*yy+fr_dev->vinfo.yoffset)*fr_dev->finfo.line_length;
    rgb24 = ((r<<3<<16) | (g << 2 << 8) | (b << 3));
    *((unsigned int *)(fr_dev->map_fb+location))=rgb24;
}

pthread_t ntid1;
pthread_t ntid2;
pthread_t ntid3;
pthread_t ntid4;

