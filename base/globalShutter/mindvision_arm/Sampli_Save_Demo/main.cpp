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

    int                     iCameraCounts = 4;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList[4];
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //Éè±¸ÃèÊöÐÅÏ¢
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    tSdkImageResolution     sImageSize;
    int                     i=0;
    int                     num=0;

    CameraSdkInit(1);



    //Ã¶¾ÙÉè±¸£¬²¢½¨Á¢Éè±¸ÁÐ±í
    CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);

    //Ã»ÓÐÁ¬½ÓÉè±¸
    if(iCameraCounts==0){
        return -1;
    }



    for(i=0;i<iCameraCounts;i++)
    {
        printf("num =%d %s  %s \n",i,tCameraEnumList[i].acProductName,tCameraEnumList[i].acFriendlyName);
    }


    printf("input   0-%d:",iCameraCounts-1);
    scanf("%d", &num);
    printf("you input num %d \n",num);

    if(num>=iCameraCounts || num < 0)
    {
        printf("Enter a number is invalid  %d \n",num);
        return -1;
    }


    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList[num],-1,-1,&hCamera);

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

#if  0
    memset(&sImageSize,0,sizeof(tSdkImageResolution));
    sImageSize.iIndex=0xff;
    sImageSize.iHOffsetFOV=0;
    sImageSize.iVOffsetFOV=0;
    sImageSize.iWidthFOV=800;
    sImageSize.iHeightFOV=600;
    sImageSize.iWidth=800;
    sImageSize.iHeight=600;


    CameraSetImageResolution(hCamera,&sImageSize);
#else
    CameraSetImageResolution(hCamera,&tCapability.pImageSizeDesc[0]);
#endif

    if(tCapability.sIspCapacity.bMonoSensor){
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_RGB8);
    }
    printf("CameraSetIspOutFormat   =====================\n");
    //sleep(2);


    long int j=0;
    char filename[128]={0};
    while(j<10){
    printf("CameraSetIspOutFormat   =====================\n");
   printf("%s - %d  ====\n",__func__,__LINE__);
    if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,2000) == CAMERA_STATUS_SUCCESS)
    {
        printf("%s - %d  ====\n",__func__,__LINE__);
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);

        printf("%s - %d  ====\n",__func__,__LINE__);


        sprintf(filename,"%s_%d",FILENAME,j);
        //½«Í¼Ïñ»º³åÇøµÄÊý¾Ý±£´æ³ÉÍ¼Æ¬ÎÄ¼þ¡£
        CameraSaveImage(hCamera, (char *)filename,g_pRgbBuffer, &sFrameInfo, FILE_BMP, 100);

        printf("%s - %d  ====\n",__func__,__LINE__);
        //ÔÚ³É¹¦µ÷ÓÃCameraGetImageBufferºó£¬±ØÐëµ÷ÓÃCameraReleaseImageBufferÀ´ÊÍ·Å»ñµÃµÄbuffer¡£
        //·ñÔòÔÙ´Îµ÷ÓÃCameraGetImageBufferÊ±£¬³ÌÐò½«±»¹ÒÆðÒ»Ö±×èÈû£¬Ö±µ½ÆäËûÏß³ÌÖÐµ÷ÓÃCameraReleaseImageBufferÀ´ÊÍ·ÅÁËbuffer
        CameraReleaseImageBuffer(hCamera,pbyBuffer);

    }else{

    	printf("timeout \n");
    }
    printf("j =%ld\n",j);
    j++;

    }
    printf("%s - %d  ====\n",__func__,__LINE__);
    CameraUnInit(hCamera);
    //×¢Òâ£¬ÏÖ·´³õÊ¼»¯ºóÔÙfree
    free(g_pRgbBuffer);


    printf("end  \n");

    return 0;
}

