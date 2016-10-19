#include <stdlib.h>
#include <iostream>
#include <string>
//【1】
#include <XnCppWrapper.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/eigen.hpp>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace cv;

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
};

// 定义生成点云函数
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

void CheckOpenNIError( XnStatus result, string status )
{ 
	if( result != XN_STATUS_OK ) 
		cerr << status << " Error: " << xnGetStatusString( result ) << endl;
}

int main( int argc, char** argv )
{
	
 
  XnStatus result = XN_STATUS_OK;
	xn::DepthMetaData depthMD;
	xn::ImageMetaData imageMD;

	//OpenCV
	Mat imgDepth16u(Size(640,480),CV_16UC1);
	Mat imgRGB8u(Size(640,480),CV_8UC3);
	Mat depthShow(Size(640,480),CV_16UC1);
	Mat imageShow(Size(640,480),CV_8UC3);
	cvNamedWindow("depth",1);
	cvNamedWindow("image",1);
	char key=0;

	//【2】
	// context 
	xn::Context context; 
	result = context.Init(); 
	CheckOpenNIError( result, "initialize context" );  

	// creategenerator  
	xn::DepthGenerator depthGenerator;  
	result = depthGenerator.Create( context ); 
	CheckOpenNIError( result, "Create depth generator" );  
	xn::ImageGenerator imageGenerator;
	result = imageGenerator.Create( context ); 
	CheckOpenNIError( result, "Create image generator" );

	//【3】
	//map mode  
	XnMapOutputMode mapMode; 
	mapMode.nXRes = 640;
	mapMode.nYRes = 480;
	mapMode.nFPS = 1; //设置后也无效
	result = depthGenerator.SetMapOutputMode( mapMode ); 
	result = imageGenerator.SetMapOutputMode( mapMode );  

	//【4】
	// correct view port  
	depthGenerator.GetAlternativeViewPointCap().SetViewPoint( imageGenerator ); 

	//【5】
	//read data
	result = context.StartGeneratingAll();  
	//【6】
	result = context.WaitNoneUpdateAll();  
 char Saverbg[256],Savedepth[256];
 int imagecount=0;

  //Check settings file
   cv::FileStorage fsSettings("/home/bobo/code/kinect1_drive/config/kinect1.yaml", cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
           cerr << "Failed to open settings file at: " << argv[2]<< endl;
           exit(-1);
        }
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = fsSettings["Camera.cx"];
    camera.cy = fsSettings["Camera.cy"];
    camera.fx = fsSettings["Camera.fx"];
    camera.fy = fsSettings["Camera.fy"];
    camera.scale = fsSettings["DepthMapFactor"];

   //PointCloud::Ptr newCloud ( new PointCloud() );
   //pcl::visualization::CloudViewer viewer("viewer");

	while( (key!=27) && !(result = context.WaitNoneUpdateAll( ))  ) 
	{  
		//get meta data
		depthGenerator.GetMetaData(depthMD); 
		imageGenerator.GetMetaData(imageMD);
                /****测试设置fps是否有效***
                imageGenerator.GetMapOutputMode( mapMode );
                cout<<"fps:"<<mapMode.nFPS<<endl;*/
		//【7】
		//OpenCV output
		memcpy(imgDepth16u.data,depthMD.Data(),640*480*2);
		//imgDepth16u.convertTo(depthShow,CV_8UC1,255/4096.0);
		memcpy(imgRGB8u.data,imageMD.Data(),640*480*3);
		cvtColor(imgRGB8u,imageShow,CV_RGB2BGR);
               
	        imshow("depth", imgDepth16u);
		imshow("image",imageShow);

  // newCloud = image2PointCloud(imageShow,imgDepth16u,camera);
   //viewer.showCloud( newCloud);
		key=cvWaitKey(20);
                if (key == 13)
                {
                   imagecount++;
                   sprintf(Saverbg,"../data/rgb_png/%01d.png",imagecount);
                   sprintf(Savedepth,"../data/depth_png/%01d.png",imagecount);
                   imwrite(Saverbg,imageShow);
                   imwrite(Savedepth,imgDepth16u);
                }
               

	}

	//destroy
	context.StopGeneratingAll();
	context.Shutdown();
	return 0;
}

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}
