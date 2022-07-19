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

#include "kalmanfilter.h"
using namespace std;
using namespace std::chrono;
using namespace cv;

#define ENEMY 0   // 0 blue 1 red

static bool debug = true;


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




[[noreturn]] void AutoAim::detection_run(const std::string &onnx_file) {
    /*
     * 识别器
     */

    if (debug) std::cout << "============ detection_run ===========" << std::endl;
    TRTModule model(onnx_file);


    // 4-dimensional state, 2-dimensional measurements
    typedef ::KalmanFilter<4, 2> KF;
    typedef KF::MeasurementSpaceVector Measurement;

    // transition matrix
    KF::StateMatrix F =
            (KF::StateMatrix() <<
                    1, 0, 1, 0,
                    0, 1, 0, 1,
                    0, 0, 1, 0,
                    0, 0, 0, 1).finished();

    // sensor model
    KF::MeasurementStateConversionMatrix H =
            ( KF::MeasurementStateConversionMatrix() <<
            1, 0, 0, 0,
            0, 1, 0, 0).finished();

    // process noise covariance
    KF::StateMatrix Q = 4. *
            (KF::StateMatrix() <<
            1, .5, 0, 0,
            .5, 1, 0, 0,
            0, 0, 1, .5,
            0, 0, .5, 1).finished();

    // measurement noise covariance
    KF::MeasurementMatrix R = 4. *
            (KF::MeasurementMatrix() <<
            1., .5,
            .5, 1.).finished();

    
    int X_c = 320, Y_c = 240;;
    KF filter = KF(Measurement(0, 0), F, Q, R, H);




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


    int cnt_useless = -1;
    const cv::Scalar colors[3] = {{255, 0,   0},
                                  {0,   0,   255},
                                  {0,   255, 0}};
	
    unsigned char *yuv422frame = NULL;
    unsigned long yuvframeSize = 0;
    
    VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);	
 
    Mat img;
    
    cv::namedWindow( "img", cv::WINDOW_NORMAL );

	
    
    while (true) {
        //const auto& [img, q, timestamp] = sensor_sub.pop();
	
    	double t = static_cast<double>(getTickCount());
        cap >> img;
 	
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
		
		if(b.color_id == ENEMY){  // 0 blue 1 red
                	cv::line(im2show, b.pts[0], b.pts[1], colors[2], 2);
                	cv::line(im2show, b.pts[1], b.pts[2], colors[2], 2);
                	cv::line(im2show, b.pts[2], b.pts[3], colors[2], 2);
                	cv::line(im2show, b.pts[3], b.pts[0], colors[2], 2);
                	cv::putText(im2show, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1,colors[b.color_id]);

			double area = cv::contourArea(vector<cv::Point2f>(b.pts, b.pts+4));
			if(area > Max_area && b.tag_id!=2){   //不打2号工程
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
		X_c = center.x;
		Y_c = center.y;
		
		//预测
		const Measurement x(X_c, Y_c);
		filter.update(x);
		
            	const KF::StateSpaceVector s = filter.state();
            	const KF::StateSpaceVector p = filter.prediction();
		Point p_p = Point(p(0), p(1));

		cv::circle(im2show, p_p, 10, colors[1],5);

		cv::circle(im2show, center, 10,colors[2],5);


		string write_buffer = "A1" + to_string(p_p.x) + " " + to_string(p_p.y) + "B";

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
		int bytes_written = write(fd, write_buffer2.c_str(), write_buffer2.size());
	     }

	    t = ((double)getTickCount() - t) / getTickFrequency();//求输入帧后经过的周期数/每秒系统计的周期数=一帧用时多少秒
            double fps = 1.0 / t;//求倒数得到每秒经过多少帧，即帧率
	    
	    
            string s = "FPS:";
            s += to_string(fps);//得到字符串FPS:XXX
            putText(im2show, s, Point(5, 20), FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 0, 0));//输入到帧frame上
            imshow("img", im2show);
            std::cout << s << "\n";

            cv::waitKey(1);
            
        }
    }
    close(fd);
}
