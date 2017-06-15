#include <iostream>
#include "CameraApi.h" //Ïà»úSDKµÄAPIÍ·ÎÄ¼þ

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using namespace std;
unsigned char           * g_pRgbBuffer;     //´¦ÀíºóÊý¾Ý»º´æÇø
#define FILENAME        "./test"

int main()
{

    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //Éè±¸ÃèÊöÐÅÏ¢
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    tSdkImageResolution     sImageSize;


    CameraSdkInit(1);



    //Ã¶¾ÙÉè±¸£¬²¢½¨Á¢Éè±¸ÁÐ±í
    CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);

    //Ã»ÓÐÁ¬½ÓÉè±¸
    if(iCameraCounts==0){
        return -1;
    }

    //Ïà»ú³õÊ¼»¯¡£³õÊ¼»¯³É¹¦ºó£¬²ÅÄÜµ÷ÓÃÈÎºÎÆäËûÏà»úÏà¹ØµÄ²Ù×÷½Ó¿Ú
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

		printf("CameraInit iStatus =%d \n",iStatus);
    //³õÊ¼»¯Ê§°Ü
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }


    //»ñµÃÏà»úµÄÌØÐÔÃèÊö½á¹¹Ìå¡£¸Ã½á¹¹ÌåÖÐ°üº¬ÁËÏà»ú¿ÉÉèÖÃµÄ¸÷ÖÖ²ÎÊýµÄ·¶Î§ÐÅÏ¢¡£¾ö¶¨ÁËÏà¹Øº¯ÊýµÄ²ÎÊý
    CameraGetCapability(hCamera,&tCapability);
	printf("CameraGetCapability \n");
    //
    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);




    /*ÈÃSDK½øÈë¹¤×÷Ä£Ê½£¬¿ªÊ¼½ÓÊÕÀ´×ÔÏà»ú·¢ËÍµÄÍ¼Ïñ
    Êý¾Ý¡£Èç¹ûµ±Ç°Ïà»úÊÇ´¥·¢Ä£Ê½£¬ÔòÐèÒª½ÓÊÕµ½
    ´¥·¢Ö¡ÒÔºó²Å»á¸üÐÂÍ¼Ïñ¡£    */
    CameraPlay(hCamera);
		printf("CameraPlay \n");
    /*ÆäËûµÄÏà»ú²ÎÊýÉèÖÃ
    ÀýÈç CameraSetExposureTime   CameraGetExposureTime  ÉèÖÃ/¶ÁÈ¡ÆØ¹âÊ±¼ä
         CameraSetImageResolution  CameraGetImageResolution ÉèÖÃ/¶ÁÈ¡·Ö±æÂÊ
         CameraSetGamma¡¢CameraSetConrast¡¢CameraSetGainµÈÉèÖÃÍ¼ÏñÙ¤Âí¡¢¶Ô±È¶È¡¢RGBÊý×ÖÔöÒæµÈµÈ¡£
         ¸ü¶àµÄ²ÎÊýµÄÉèÖÃ·½·¨£¬£¬Çå²Î¿¼MindVision_Demo¡£±¾Àý³ÌÖ»ÊÇÎªÁËÑÝÊ¾ÈçºÎ½«SDKÖÐ»ñÈ¡µÄÍ¼Ïñ£¬×ª³ÉOpenCVµÄÍ¼Ïñ¸ñÊ½,ÒÔ±ãµ÷ÓÃOpenCVµÄÍ¼Ïñ´¦Àíº¯Êý½øÐÐºóÐø¿ª·¢
    */

#if 0
    memset(&sImageSize,0,sizeof(tSdkImageResolution));
    sImageSize.iIndex=0xff;
    sImageSize.iHOffsetFOV=352;
    sImageSize.iVOffsetFOV=264;
    sImageSize.iWidthFOV=3648;
    sImageSize.iHeightFOV=2742;
    sImageSize.iWidth=3648;
    sImageSize.iHeight=2742;


    CameraSetImageResolution(hCamera,&sImageSize);
#endif
    CameraSetImageResolution(hCamera,&tCapability.pImageSizeDesc[0]);


    if(tCapability.sIspCapacity.bMonoSensor){
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_RGB8);
    }
    printf("CameraSetIspOutFormat \n");
    sleep(2);



    if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,2000) == CAMERA_STATUS_SUCCESS)
    {
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
        //½«Í¼Ïñ»º³åÇøµÄÊý¾Ý±£´æ³ÉÍ¼Æ¬ÎÄ¼þ¡£
        CameraSaveImage(hCamera, (char *)FILENAME,g_pRgbBuffer, &sFrameInfo, FILE_BMP, 100);

        //ÔÚ³É¹¦µ÷ÓÃCameraGetImageBufferºó£¬±ØÐëµ÷ÓÃCameraReleaseImageBufferÀ´ÊÍ·Å»ñµÃµÄbuffer¡£
        //·ñÔòÔÙ´Îµ÷ÓÃCameraGetImageBufferÊ±£¬³ÌÐò½«±»¹ÒÆðÒ»Ö±×èÈû£¬Ö±µ½ÆäËûÏß³ÌÖÐµ÷ÓÃCameraReleaseImageBufferÀ´ÊÍ·ÅÁËbuffer
        CameraReleaseImageBuffer(hCamera,pbyBuffer);

    }else{

    	printf("timeout \n");
    }



    CameraUnInit(hCamera);
    //×¢Òâ£¬ÏÖ·´³õÊ¼»¯ºóÔÙfree
    free(g_pRgbBuffer);


    printf("end  \n");

    return 0;
}

