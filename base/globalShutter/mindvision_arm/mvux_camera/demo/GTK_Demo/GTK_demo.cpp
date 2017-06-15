#include <iostream>
#include <gtk/gtk.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>

#include "CameraApi.h"
#include "interface.h"
//#include <pcap.h>

#include "Demo.h"
using namespace std;

#define     GLADE_NAME      "GTK_DEMO.glade"

//SDK
int                     g_hCamera = -1;     //设备句柄
unsigned char           * g_pRawBuffer;     //raw数据
unsigned char           * g_pRgbBuffer;     //处理后数据缓存区
tSdkFrameHead           g_tFrameHead;       //图像帧头信息
tSdkCameraCapbility     g_tCapability;      //设备描述信息

//gtk demo使用
GtkBuilder              *Demo_builder =NULL;        //glade进行界面设计
GtkWidget               *main_window = NULL;        //主窗体
GtkWidget               *display_drawingarea= NULL; //画板

Width_Height            g_W_H_INFO;         //显示画板到大小和图像大小
int                     g_display_state=1;  //显示状态  1读取数据 0退出
BYTE                    *g_readBuf=NULL;    //画板显示数据区
pthread_t               g_thread_id;        //线程
pthread_t               g_thread_getcounts; //线程
int                     g_read_fps=0;       //统计读取帧率
int                     g_disply_fps=0;     //统计显示帧率

pthread_mutex_t mutex;

int main (int argc, char **argv)
{

    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList[4];
    tSdkCameraDevInfo       *tCameraEnumList_tmp;
    int                     iCameraCounts = 4;
    int                     num=0,i;


	if(!g_thread_supported())
	{
	g_thread_init(NULL);
	}
	gdk_threads_init();

	gtk_init(&argc,&argv);

	Demo_builder = gtk_builder_new();
    gtk_builder_add_from_file(Demo_builder, GLADE_NAME, NULL);

    pthread_mutex_init(&mutex,NULL);

    //sdk初始化  0 English 1中文
    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);


    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }


    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList[num],-1,-1,&g_hCamera);

    //初始化失败
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(g_hCamera,&g_tCapability);

    g_pRgbBuffer = (unsigned char*)malloc(g_tCapability.sResolutionRange.iHeightMax*g_tCapability.sResolutionRange.iWidthMax*3);
    g_readBuf = (unsigned char*)malloc(g_tCapability.sResolutionRange.iHeightMax*g_tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(g_hCamera);

/*
    设置图像处理的输出格式，彩色黑白都支持RGB24位
*/
    if(g_tCapability.sIspCapacity.bMonoSensor){
        CameraSetIspOutFormat(g_hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        CameraSetIspOutFormat(g_hCamera,CAMERA_MEDIA_TYPE_RGB8);
    }
    //创建窗体构建
    main_window=create_gtkdemo_window(g_hCamera,&g_tCapability);

    //根据设备设置参数初始化界面
    Gtk_GUI_Init(g_hCamera,&g_tCapability);


    //创建preview 读取相机每帧数据
    pthread_create(&g_thread_id, NULL, preview_thread, NULL);

    //创建
   // pthread_create(&g_thread_getcounts, NULL, getcounts_thread, NULL);


    gtk_widget_show (main_window);


	gdk_threads_enter();
	gtk_main();
	gdk_threads_leave();

    return 0;
}



