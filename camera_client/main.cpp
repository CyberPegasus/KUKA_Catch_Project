#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#define theta_comp 1.8911
#define PORT 8888

using namespace std;
using namespace cv;

const bool enable_rgb = true;
const bool enable_depth = true;
int center_cx[10];
int center_cy[10];
const int hr = 148;//仿射变换后图像高度
const int wr = 602;//放射变换后图像宽度

//图像检测定位主函数
int camera_detect_main(double *world_xyt);
//图像检测定位相关函数
int img_correct(cv::Mat& sourceImg,cv::Mat& showImg,cv::Mat& rgbd);
int img_segment(cv::Mat& rgb,cv::Mat& rgbd,int h,int w,double far,double close);
int img_canny(cv::Mat& sImg,double low_threshold,double high_threshold,double seg_threshold);
int img_rec_find(cv::Mat& sImg,double *corner_x,double *corner_y);
int object_center_locate(double *center_xy, double *world_xy, double *circle_center_xy, int h, int w);
int get_circle_center_xy(double *circle_center_xy);
//通讯主函数
int client(double *world_xyt);

int main(){
    //目前运行方式为一次性运行
    //获取工件的世界坐标系
    double world_xyt[3];
    camera_detect_main(world_xyt);
    cout << "待抓取物块的的世界坐标为（" << world_xyt[0] <<","<< world_xyt[1] <<"）,转动角度为:"<<world_xyt[2]<< endl;
    //确认是否进行抓取
    char c;
    std::cout<<"Please enter to confirm the location, or to quit."<<endl;
    scanf("%c",&c);
    if (c=='\n')
        client(world_xyt);
    else
        std::cout<<"quit..."<<endl;
    return 0;
}
//工件检测主程序
int camera_detect_main(double *world_xyt){
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;
    libfreenect2::PacketPipeline *pipeline = nullptr;

    // 检测是否有设备连接并列举所有设备--------------
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    string serial = freenect2.getDefaultDeviceSerialNumber();
    if (serial.empty()){
        std::cout<<"Can't get the serial."<<endl;
        return -1;
    }
    std::cout<<"Begin to connect the kinect v2 camera,The serial number is :"<<serial<<endl;
    //-----------------------------------------

    //定义处理pipeline串流的方式-----------------------------
    pipeline = new libfreenect2::OpenCLPacketPipeline();//使用开源并行计算库，比较快
    //---------------------------------------------------

    //打开并设置设备-------------------------------------------
    dev = freenect2.openDevice(serial, pipeline);
    if (dev == 0){
        std::cout<<"Open the device failed"<<endl;
        return -1;
    }
    int types = 0;
    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    //---------------------------------------------------

    //设备开始采集深度图像----------------------------------
    if (enable_rgb && enable_depth)
    {
        if (!dev->start()){
            std::cout<<"device can't start"<<endl;
            return -1;
        }
    }
    else
    {
        if (!dev->startStreams(enable_rgb, enable_depth)){
            std::cout<<"device can't start"<<endl;
            return -1;
        }
    }
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;


    //Registration Setup--------------------------------
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    //depth2rgb的上下两行为空白行
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),depth2rgb(1920, 1080 + 2, 4);
    //--------------------------------------------------

    ////////////////////////////////////////////////////
    //使用OpenCV处理相机数据--------------------------------
    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
    listener.waitForNewFrame(frames);
    //获取Frame对应数据
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    //转为CV MAT格式
    cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
    Mat rgb3;
    cv::cvtColor(rgbmat,rgb3,CV_BGRA2BGR);
    //水平翻转
    cv::flip(rgb3,rgb3,1);
    cv::imshow("rgb", rgb3);
    //registration
    registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
    cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
    //水平翻转
    cv::flip(rgbd2,rgbd2,1);
    cv::imshow("depth2RGB", rgbd2 / 4500.0f);
    //---------------------------------------------------


    //Image_process-------------------------------------

    std::cout<<"The size of the graph:"<<rgbd2.cols<<"*"<<rgbd2.rows<<endl;
    std::cout<<"Test:"<<rgbd2.at<float>(1,0)<<","<<rgbd2.at<float>(540,960)<<endl;
    Mat img_color = rgb3;
    Mat show_Img = rgb3;

    img_segment(show_Img,rgbd2,1080,1920,1300,1000);
    imshow("rgb segment",show_Img);

    Mat img_cor = show_Img;
    cvtColor(img_cor, img_cor, COLOR_BGR2GRAY);
    imshow("gray",img_cor);
    int loop=0;
    while(img_correct(img_cor,show_Img,rgbd2)){
        loop++;
        if(loop==10){
            waitKey(0);
            listener.release(frames);
            dev->stop();
            dev->close();
            delete registration;
            return 0;
        }
    }
    imshow("rgb Corrected",show_Img);
    imshow("rgbd Corrected",rgbd2 / 4500.0f);


    cvtColor(show_Img,show_Img,CV_BGR2GRAY);
    threshold(show_Img,show_Img,85,255,THRESH_BINARY);
    imshow("gray",show_Img);

    img_canny(show_Img,50,100,80);
    imshow("After Canny",show_Img);

    //locate
    double corner_x[5];//第一～四个数组元素是角点x坐标,第五个是中心x坐标
    double corner_y[5];//第一～四个数组元素是角点y坐标,第五个是中心y坐标
    if(!img_rec_find(show_Img,corner_x,corner_y))
        imshow("After Locate",show_Img);
    else{
        std::cout<<"Can't find the object, please check the img_rec_find function."<<endl;
        listener.release(frames);
        dev->stop();
        dev->close();
        delete registration;
        return 0;
    }
    for(int i=0;i<4;i++){
        cout << "待抓取物块的第" << i+1<<"个角点的坐标为（"<< corner_x[i]<<","<< corner_y[i]<<"）"<< endl;
    }
    cout << "待抓取物块的的中心坐标为（" << corner_x[4] <<","<< corner_y[4] <<"）"<< endl;
    //--------------------------------------------------


    //对抓取位姿进行估计-------------------------------------
    double catch_theta = 0.0;
    double x_left = corner_x[0];
    double x_right = corner_x[0];
    int most_left;
    int most_right;
    for (int i = 0; i < 4; ++i) {
        if (x_left>=corner_x[i])
            most_left=i;
        if (x_right<=corner_x[i])
            most_right=i;
    }
    //对角线与图像坐标系的夹角
    double tan_theta = (corner_y[most_left]-corner_y[most_right])/(corner_x[most_left]-corner_x[most_right]);
    catch_theta = atan(tan_theta);
    //两个对边垂线与图像坐标系的夹角（理想的夹持角度）
    catch_theta += 45;
    //加上图像坐标系顺势针旋转1.89度得到机械臂基座坐标系的影响,逆时针还原，对于机械臂A6角是加上该角度
    catch_theta += theta_comp;
    //换算出实际A6角度，因为A6垂直方向是-170度，水平方向是-80度
    catch_theta -= 80;
    //考虑到夹爪导线的缠绕影响，获取实际的夹持角度
    while (catch_theta < -170)
        catch_theta+=90;
    while (catch_theta > -80)
        catch_theta-=90;
    //得到最终角度
    world_xyt[2]=catch_theta;

    // ----------------------------------------------------

    //物块中心坐标，第一个为x，第二个为y
    double center_xy[2];center_xy[0]=corner_x[4];center_xy[1]=corner_y[4];
    //xy交叉排序，存储参照圆的圆心在机械臂基座坐标系下的坐标
    double circle_center_xy[8];
    get_circle_center_xy(circle_center_xy);
    double world_xy[2];
    object_center_locate(center_xy, world_xy, circle_center_xy, show_Img.rows, show_Img.cols);
    world_xyt[0]=world_xy[0];
    world_xyt[1]=world_xy[1];

    //结束--------------------------------------------------
    listener.release(frames);
    dev->stop();
    dev->close();
    delete registration;
    std::cout<<"Camera Quit..."<<endl;
    //------------------------------------------------------
    return 0;
}
//定位参考圆，并标出
int img_correct(cv::Mat& sourceImg,cv::Mat& showImg,cv::Mat& rgbd){

    vector<Vec3f> circles;
    HoughCircles(sourceImg, circles, CV_HOUGH_GRADIENT, 1, 60,90, 40, 20, 40  );
    for (size_t i = 0; i < circles.size(); i++){
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        center_cx[i]=center.x;
        center_cy[i]=center.y;
        cout << "第"<<i+1<<"个圆的x坐标为" << center_cx[i] <<" ,y坐标为"<< center_cy[i]<< endl;
        int radius = cvRound(circles[i][2]);
        circle(showImg, center, 3, Scalar(155, 50, 255), -1, 8, 0);
        circle(showImg, center, radius, Scalar(155, 50, 255), 3, 8, 0);
    }
    imshow("Circle",showImg);
    if((center_cx[3]!=0)&&(center_cx[4]==0)){
        std::cout<<"基于参照圆的参考系建立成功"<<endl;
    }
    else{
        std::cout<<"参照圆识别错误，请更改标定参数！"<<endl;
        cvWaitKey(0);
        return 1;
    }
    //标定圆中心坐标排序
    cout<<"正在进行角点排序......"<<endl;
    int zs=0,zx=0,ys=0,yx=0,summax=center_cx[0]+center_cy[0],summin=center_cx[0]+center_cy[0];
    for(int i=0;i<4;i++){
        if(summax<(center_cx[i]+center_cy[i])){
            yx=i;
            summax=center_cx[i]+center_cy[i];
        }
        if(summin>(center_cx[i]+center_cy[i])){
            zs=i;
            summin=center_cx[i]+center_cy[i];
        }
    }
    summax=0;summin=100000;
    for(int i=0;i<4;i++){
        if(i!=zs&&i!=yx&&summax<center_cx[i]){
            ys=i;
            summax=center_cx[i];
        }
        if(i!=zs&&i!=yx&&summin>center_cx[i]){
            zx=i;
            summin=center_cx[i];
        }
    }
    cout<<"左上角坐标为("<<center_cx[zs]<<","<< center_cy[zs]<<")"<<endl;
    cout<<"左下角坐标为("<<center_cx[zx]<<","<< center_cy[zx]<<")"<<endl;
    cout<<"右下角坐标为("<<center_cx[yx]<<","<< center_cy[yx]<<")"<<endl;
    cout<<"右上角坐标为("<<center_cx[ys]<<","<< center_cy[ys]<<")"<<endl;
    //放射变换

    Point2f srcPoints[4], dstPoints[4];
    dstPoints[0]=Point2f(0,0);
    dstPoints[1]=Point2f(0,hr);
    dstPoints[2]=Point2f(wr,hr);
    dstPoints[3]=Point2f(wr,0);

    srcPoints[0]=Point2f(center_cx[zs],center_cy[zs]);
    srcPoints[1]=Point2f(center_cx[zx],center_cy[zx]);
    srcPoints[2]=Point2f(center_cx[yx],center_cy[yx]);
    srcPoints[3]=Point2f(center_cx[ys],center_cy[ys]);

    Mat transMat = getPerspectiveTransform(srcPoints, dstPoints);
    Mat roiImg_rgb(hr, wr, CV_8UC3);
    warpPerspective(showImg, roiImg_rgb, transMat, roiImg_rgb.size());
    Mat roiImg_rgbd(hr,wr,CV_32FC1);
    warpPerspective(rgbd, roiImg_rgbd, transMat, roiImg_rgbd.size());

    showImg = roiImg_rgb;
    rgbd = roiImg_rgbd;
    return 0;
}
//按深度进行分割
//h为图片高度，w为图片宽度，far为远距离阈值，close为近距离阈值，函数保留从近距离到远距离的深度图像对应的Color图像部分
int img_segment(cv::Mat& rgb,cv::Mat& rgbd,int h,int w,double far,double close){
    for(int i=0;i<h;i++)
        for(int j=0;j<w;j++)
            //注意深度图像为1920*1082，其中第一行和最后一行为无效行
            if(rgbd.at<float>(i+1,j)<close||rgbd.at<float>(i+1,j)>far){
                rgb.at<Vec3b>(i,j)[0]=0;
                rgb.at<Vec3b>(i,j)[1]=0;
                rgb.at<Vec3b>(i,j)[2]=0;
            }
}
//canny边缘检测
//seg_threshold为灰度阈值分割参数，low和high threshold为canny高低阈值
int img_canny(cv::Mat& sImg,double low_threshold,double high_threshold,double seg_threshold){

    //threshold(sImg,sImg,seg_threshold,255,THRESH_BINARY);
    Canny(sImg, sImg, low_threshold, high_threshold,3, false);
}
//定位矩形工件表面
int img_rec_find(cv::Mat& sImg,double *corner_x,double *corner_y){
    vector<vector<Point>> contours;
    Mat profile_line, polyPic;
    cv::findContours(sImg, contours, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    vector<vector<Point>> polyContours(2);

    int maxArea = 0;
    for (unsigned int index = 0; index < contours.size(); index++) {
        if (contourArea(contours[index]) > contourArea(contours[maxArea]))
            maxArea = index;
    }
    approxPolyDP(contours[maxArea], polyContours[0], arcLength(contours[maxArea],true)*0.1, true);

    polyPic = Mat::zeros(sImg.size(), CV_8UC3);
    drawContours(polyPic, polyContours, 0, Scalar(0, 0, 255), 1);

    vector<int>  hull;
    convexHull(polyContours[0], hull);
    double obj_center_x=0;
    double obj_center_y=0;
    int hull_size=hull.size();
    if(hull_size>5)
        return -1;
    for (int i = 0; i < hull_size; ++i) {
        circle(polyPic, polyContours[0][i], 5, Scalar(0, 255, 0), 3);
        obj_center_x=obj_center_x+polyContours[0][i].x;
        obj_center_y=obj_center_y+polyContours[0][i].y;
        corner_x[i]=polyContours[0][i].x;
        corner_y[i]=polyContours[0][i].y;
        //cout << "待抓取物块的第" << i+1<<"个角点的坐标为（"<< polyContours[0][i].x<<","<< polyContours[0][i].y<<"）"<< endl;
    }
    obj_center_x=(obj_center_x)/hull_size;
    obj_center_y=(obj_center_y)/hull_size;
    corner_x[4]=obj_center_x;
    corner_y[4]=obj_center_y;
    //cout << "待抓取物块的的中心坐标为（" << obj_center_x<<","<< obj_center_y<<"）"<< endl;
    circle(polyPic, Point(obj_center_x,obj_center_y), 3, Scalar(0, 255, 0), 2);
    sImg = polyPic;
    return 0;
}
//对工件中心进行定位
int object_center_locate(double *center_xy, double *world_xy, double *circle_center_xy, int h, int w){
    //图像坐标系以左上圆为原点，从左向右为x正方向，从上向下为y正方向
    //double world_x_ave=0;
    //double world_y_ave=0;
    //左上角标定圆的世界坐标
    //机械臂坐标系，从左到右为y，从上到下为x；
    double circle_x=536.117;double circle_y=-296.244;
    /*
    world_x_ave+=(center_xy[0]/w)*(circle_center_xy[2]-circle_center_xy[0])+circle_center_xy[0];
    world_x_ave+=(center_xy[0]/w)*(circle_center_xy[4]-circle_center_xy[6])+circle_center_xy[6];
    world_x_ave=world_x_ave/2.0;
    world_xyt[0]=world_x_ave;

    world_y_ave+=(center_xy[1]/h)*(circle_center_xy[7]-circle_center_xy[1])+circle_center_xy[1];
    world_y_ave+=(center_xy[1]/h)*(circle_center_xy[5]-circle_center_xy[3])+circle_center_xy[3];
    world_y_ave=world_y_ave/2.0;
    world_xyt[1]=world_y_ave;
    */
    //mm为单位的图像距离h_r与w_r
    //以下为图像坐标系
    double h_r=center_xy[1]/h*148;//或者改为154.8
    double w_r=center_xy[0]/w*602;//或者改为629.7
    double sin=0.033;
    double cos=0.999;
    //以下为机械臂坐标系
    double deltay=h_r*sin+w_r*cos;
    double deltax=h_r*cos-w_r*sin;
    world_xy[0]= deltax + circle_x;
    world_xy[1]= deltay + circle_y;
}
//对四个参照圆的中心坐标手动赋值
int get_circle_center_xy(double *circle_center_xy){

    //实际的坐标要以机械臂工具坐标系为准
    //左上
    circle_center_xy[0]=536.117;//x
    circle_center_xy[1]=-296.224;//y
    //右上
    circle_center_xy[2]=0;
    circle_center_xy[3]=0;
    //右下
    circle_center_xy[4]=0;
    circle_center_xy[5]=0;
    //左下
    circle_center_xy[6]=0;
    circle_center_xy[7]=0;
}
//发起TCP客户端通讯
int client(double *world_xyt){
    double send_raw_xyt[2];send_raw_xyt[0]=world_xyt[0];send_raw_xyt[1]=world_xyt[1];send_raw_xyt[2]=world_xyt[2];
    std::cout << "打开套接字" << endl;
    int socket_fd = socket(AF_INET,SOCK_STREAM,0);
    if (socket_fd == -1)
    {
        std::cout << "socket 创建失败： "<< endl;
        exit(1);
    }
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);//将一个无符号短整型的主机数值转换为网络字节顺序，即大尾顺序(big-endian)
    addr.sin_addr.s_addr = inet_addr("192.168.1.5");;//INADDR_ANY;
    bzero(&(addr.sin_zero), 8);
    int struct_len = sizeof(struct sockaddr_in);
    std::cout << "正在连接客户端...." << endl;
    while (connect(socket_fd,(struct sockaddr*)&addr,struct_len)==-1);
    std::cout << "连接到客户端" << endl;
    char bufferx[10]={};
    char buffery[10]={};
    char buffert[10]={};
    sprintf(bufferx, "%f", send_raw_xyt[0]);
    sprintf(buffery, "%f", send_raw_xyt[1]);
    sprintf(buffert, "%f", send_raw_xyt[2]);
    //依次发送x和y
    write(socket_fd,bufferx,sizeof(bufferx));
    write(socket_fd,buffery,sizeof(buffery));
    write(socket_fd,buffert,sizeof(buffert));
    char buffer_read[20]={};
    read(socket_fd,buffer_read,sizeof(buffer_read));
    std::cout<< "内容： " << buffer_read << endl;
    close(socket_fd);
}