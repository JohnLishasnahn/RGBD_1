// C++ 标准库
#include <iostream>
#include <string>
// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;
// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 
// 相机内参
const double camera_factor = 1000;		// 深度图的缩放因子
const double camera_cx = 325.5;			// 内参矩阵参数fx,fy,cx,cy
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

// 主函数 
int main( int argc, char** argv )
{
    // 读取./data/rgb.png和./data/depth.png，并转化为点云
    // 图像矩阵
    cv::Mat rgb, depth;
    // 使用cv::imread()来读取图像
    // API: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html?highlight=imread#cv2.imread
    rgb = cv::imread( "../data/11.png" );
    cout<<rgb.size()<<endl;							//输出结果:	[640 x 480]
    cout<<depth.size()<<endl;						//输出结果:	[0 x 0]
    // rgb 图像是8UC3的彩色图像
    // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
    depth = cv::imread( "../data/22.png", -1 );

    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr cloud (new PointCloud);
    // 遍历深度图			//像素坐标[u,v,d]这里用变量m,n分别代替u,v遍历深度图
    for (int m = 0; m < depth.rows; m++)	// 像素坐标系，横坐标(向右)为y,纵坐标(向下)为x,
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
            p.z = double(d) / camera_factor;						// z = d/s
            p.x = (n - camera_cx) * p.z / camera_fx;			// x = z*(u-cx)/fx  这里u为n,v为m
            p.y = (m - camera_cy) * p.z / camera_fy;			// y = x*(v-cy)/fy
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中(这里的cloud是创建的空云)
            cloud->points.push_back(p);
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "./cloud.pcd", *cloud );					//实际不在src下，而是在build下
    // 清除数据并退出
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}
