#include "AutoAim.h"
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <string>
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstdlib>
#include<math.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace std::chrono;
using namespace cv;

static bool debug = true;

const int winHeight = 480;
const int winWidth = 640;

string num(string n){   //坐标三位数字补0
			int a = 3 - n.size();

			if(a==0) return n;
			else if(a==1) return ("0" + n);
			else if(a==2) return ("00" + n);
			else if(a==3) return ("000" );
		}

string num2(int n){  //设置标志位
	if(n<0){
		string res = "0" + to_string(abs(n));
		return res;	
	}
	else return "1" + to_string(n);	
	
}

// 创建一个Kalman滤波器
KalmanFilter CreateKF(double sys_noise)		
{
    RNG rng;
    //1.kalman filter setup
    const int stateNum = 4;                                      //状态值4×1向量(x,y,△x,△y)
    const int measureNum = 2;                                    //测量值2×1向量(x,y)
    KalmanFilter KF(stateNum, measureNum, 0);	// 构建卡尔曼滤波器模型

    KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 
						0, 1, 0, 1, 
						0, 0, 1, 0, 
						0, 0, 0, 1);  //转移矩阵A
    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(sys_noise));                            //系统噪声方差矩阵Q 
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2));                        //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
    rng.fill(KF.statePost, RNG::UNIFORM, 0, winHeight > winWidth ? winWidth : winHeight);   //初始状态值x(0)
    return KF;
}

Point Kalman_Predict(KalmanFilter KF)
{
    Mat prediction = KF.predict();
    Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //预测值(x',y')
    return predict_pt;
}


[[noreturn]] void AutoAim::detection_run(const std::string &onnx_file) {
    /*
     * 识别器
     */

    if (debug) std::cout << "============ detection_run ===========" << std::endl;
    TRTModule model(onnx_file);


	//kalman滤波
	Point predict_pt;
	KalmanFilter KF = CreateKF(1e-5);	// 创建一个Kalman Filter
	Mat measurement_A = Mat::zeros(2, 1, CV_32F);

	


    int fd;
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) /* Error Checking */
        printf("\n  Error! in Opening ttyUSB0  ");
    else
	printf("\n  ttyUSB0 Opened Successfully ");

    struct termios SerialPortSettings;

    tcgetattr(fd, &SerialPortSettings);

    //设置波特率
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    //设置没有校验
    SerialPortSettings.c_cflag &= ~PARENB;

    //停止位 = 1
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;

    //设置数据位 = 8
    SerialPortSettings.c_cflag |= CS8;

    SerialPortSettings.c_cflag &= ~CRTSCTS;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;

    //关闭软件流动控制
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    //设置操作模式
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    SerialPortSettings.c_oflag &= ~OPOST;

    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0)
        printf("\n  ERROR ! in Setting attributes");
    else
        printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none");





//    auto webview_checkbox = umt::ObjManager<CheckBox>::find_or_create("show detections");
//
//    umt::Subscriber<SensorsData> sensor_sub("sensors_data");
//    umt::Publisher<cv::Mat> webview_detections("detections");
//
//    umt::Publisher<Detection_pack> detections_pub("detections_pack");


 	std::cout << "empty" << std::endl;

    int fps = 0, fps_count = 1;
    auto t1 = system_clock::now();
    int cnt_useless = -1;
    const cv::Scalar colors[3] = {{255, 0,   0},
                                  {0,   0,   255},
                                  {0,   255, 0}};
	
    //cv::VideoCapture capture("/home/jimmy/视频/video.mp4");
    cv::VideoCapture capture(0);
    capture.set(cv::CAP_PROP_FRAME_WIDTH,640);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    //capture.set(cv::CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'));
    cv::namedWindow( "img", cv::WINDOW_NORMAL );

	

    while (true) {
        //const auto& [img, q, timestamp] = sensor_sub.pop();
        cv::Mat img;
        capture >> img;
        //cv::Mat img = cv::imread("/home/hfut/tansor/CVRM2022/AutoAim/202202003.jpg");
        auto detections = model(img);

        /* publish detection results */
        if (!detections.empty()) {
            std::cout << "detected" << std::endl;
            //detections_pub.push(Detection_pack{detections, img, q, timestamp});  // 打包数据
        } else {
            std::cout << "empty" << std::endl;
            if (++cnt_useless == 50) {  // 避免输出太多
                //fmt::print(fmt::fg(fmt::color::blue), "No enemy detected!");
                std::cout << std::endl;
                cnt_useless = -1;
            }
            //webview_detections.push(img);
            //continue;
        }
	
	
        /* show detections */
        if (true) {
	    double Max_area = 0;
	    vector<cv::Point2f> p_max;
	    //S_area.clear();
            cv::Mat im2show = img.clone();
            for (const auto &b: detections) {
		
		if(b.color_id == 0){  // 0 blue 1 red
                	cv::line(im2show, b.pts[0], b.pts[1], colors[2], 2);
                	cv::line(im2show, b.pts[1], b.pts[2], colors[2], 2);
                	cv::line(im2show, b.pts[2], b.pts[3], colors[2], 2);
                	cv::line(im2show, b.pts[3], b.pts[0], colors[2], 2);
                	cv::putText(im2show, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1,colors[b.color_id]);

			
			double area = cv::contourArea(vector<cv::Point2f>(b.pts, b.pts+4));
			if(area > Max_area){
				Max_area = area;
				p_max = vector<cv::Point2f>(b.pts, b.pts+4);
			}
		}
		//cout<< b.pts[0]<< b.pts[1]<<b.pts[2]<<b.pts[3]<<b.tag_id<<std::endl;
            }
	    if(Max_area){
	    	cv::Point2f p1 = cv::Point2f((p_max[0].x+p_max[1].x)/2, (p_max[0].y+p_max[1].y)/2);
	    	cv::Point2f p2 = cv::Point2f((p_max[2].x+p_max[3].x)/2, (p_max[2].y+p_max[3].y)/2);

	    	cv::Point2f center = cv::Point2f((p1.x+p2.x)/2, (p1.y+p2.y)/2);
		int X_c = center.x, Y_c = center.y;
		
		//预测
		predict_pt = Kalman_Predict(KF);
		measurement_A.at<float>(0) = X_c;
        	measurement_A.at<float>(1) = Y_c;
		
            	KF.correct(measurement_A);
		cv::circle(im2show, center, 10,colors[2],5);
		cv::circle(im2show, predict_pt, 10,colors[1],5);
		


		string write_buffer = "A1" + num(to_string(X_c)) + " " + num(to_string(Y_c)) + "B";

		int bytes_written = write(fd, write_buffer.c_str(), write_buffer.size());//传出坐标
		cout << write_buffer<<endl;

		//计算角度
		//double fx=2.734929096203664e+03;
		//double fy=2.728616564280733e+03;
		//double rx=(X_c-320)/fx;
		//double ry=(Y_c-220)/fy;
		//cout<< "angx: "<<to_string(atan(rx)/CV_PI*180)<<endl;
    		//cout<< "angy: "<<to_string(atan(ry)/CV_PI*180)<<endl;

		//string write_buffer2 = "A1 "+ num2(atan(rx)/CV_PI*180) +" "+ num2(atan(ry)/CV_PI*180)+"B";
		//cout << write_buffer2<<endl;
		//int bytes_written = write(fd, write_buffer2.c_str(), write_buffer2.size());//传出角度
		
		
	    	
		//cout << Max_area <<endl;
            }else{
		//string write_buffer2 = "A0 00 00B";
		string write_buffer2 = "A0000 000B";
		cout << write_buffer2<<endl;
		int bytes_written = write(fd, write_buffer2.c_str(), write_buffer2.size());//传出角度
	     }
	    fps_count++;
	    
	    
	    
            auto t2 = system_clock::now();
            if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                fps = fps_count;
                fps_count = 0;
                t1 = t2;
            }
            cv::putText(im2show, "fps=" + fps, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
            std::cout << fps << "\n";
            cv::imshow("img",im2show);
            cv::waitKey(1);
            
        }
    }
    close(fd);
}
