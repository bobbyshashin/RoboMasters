#include "capturethread.h"
#include "mainwindow.h"
#include <QDebug>

#include "CameraApi.h"

//SDK使用
extern int                  g_hCamera;          //设备句柄
extern unsigned char        * g_pRawBuffer;     //raw数据
extern unsigned char        * g_pRgbBuffer;     //处理后数据缓存区
extern tSdkFrameHead        g_tFrameHead;       //图像帧头信息
extern tSdkCameraCapbility  g_tCapability;      //设备描述信息


extern Width_Height         g_W_H_INFO;         //显示画板到大小和图像大小
extern BYTE                 *g_readBuf;         //显示数据buffer
extern int                  g_read_fps;         //统计帧率
extern int                  g_SaveImage_type;   //保存图像格式

CaptureThread::CaptureThread(QObject *parent) :
    QThread(parent)
{
    pause_status = true;
    play = false;

    for(int i = 0; i < 256; i++)
    {
       grayColourTable.append(qRgb(i, i, i));
    }
}

void CaptureThread::run()
{



    forever
    {
        if(!pause_status)
        {
			if(play) break;
            if (CameraGetImageBuffer(g_hCamera,&g_tFrameHead,&g_pRawBuffer,2000) == CAMERA_STATUS_SUCCESS)
            {
                CameraImageProcess(g_hCamera,g_pRawBuffer,g_pRgbBuffer,&g_tFrameHead);

                if(g_tFrameHead.uiMediaType==CAMERA_MEDIA_TYPE_MONO8){

                    memcpy(g_readBuf,g_pRgbBuffer,g_W_H_INFO.buffer_size);

					if(play) break;
                    QImage img(g_readBuf, g_W_H_INFO.sensor_width, g_W_H_INFO.sensor_height,QImage::Format_Indexed8);
                    img.setColorTable(grayColourTable);
                    emit captured(img);

                }else{
                    memcpy(g_readBuf,g_pRgbBuffer,g_W_H_INFO.buffer_size*3);
					if(play) break;
                    QImage img(g_readBuf, g_W_H_INFO.sensor_width, g_W_H_INFO.sensor_height,QImage::Format_RGB888);
                    emit captured(img);
                }
                

                g_read_fps++;//统计抓取帧率

            }else{
                printf("timeout \n");
                usleep(1000);
            }


        } else usleep(1000);
        if(play) break;

    }
}

void CaptureThread::stream()
{
    pause_status = false;
}

void CaptureThread::pause()
{
    pause_status = true;
}

void CaptureThread::stop()
{
    pause_status = false;
    play = true;
}

