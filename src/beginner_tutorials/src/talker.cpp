/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
//ros header
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include <sensor_msgs/LaserScan.h>//laserScan

#include <tf/transform_broadcaster.h>
//std header
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
//laser detect header
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include "lsd_opencv.hpp"
using namespace std;
using namespace cv;
//control bar
int threshold_value1 = 150;
int threshold_value2 = 255;
int threshold_value3 = 80;
int threshold_value4 = 100;
int threshold_value5 = 750;
char* trackbar_value1 = "H_low Value";
char* trackbar_value2 = "H_high Value";
char* trackbar_value3 = "angle_low Value";
char* trackbar_value4 = "angle_high Value";
char* trackbar_value5 = "HIGH";
Mat cameraMatrix, distMatrix, warp3D, warp3DInv;
//
double cameraPara[9] = { 387.9441, 0, 307.5401,
0, 385.1973, 269.5151,
0, 0, 1.0000 };
double distorPara[4] = { 0.1420, -0.2269, 0.0017, -0.0034 };
double para[5] = { 0.376445490967450 ,- 0.926358114094578	,0.0108593356748642 ,- 0.00561352029593190	,1010.59051752313-259 };
double a[640], b[640], c[640];
void Threshold_Demo(int, void*)
{}
string num2str(int i){
	stringstream s;
	s << i;
	return s.str();
}
int my_cmp(double p1, double  p2)
{
	return p1> p2;
}
void updatePara(){

	Mat(3, 3, CV_64FC1, cameraPara).copyTo(cameraMatrix);
	Mat(4, 1, CV_64FC1, distorPara).copyTo(distMatrix);//深复制

	double q0 = para[0], q1 = para[1], q2 = para[2], q3 = para[3], h = para[4];
	double R[9] = {
		2 * q0*q0 + 2 * q1*q1 - 1, 2 * q1*q2 - 2 * q0*q3, 2 * q1*q3 + 2 * q0*q2,
		2 * q1*q2 + 2 * q0*q3, 2 * q0*q0 + 2 * q2*q2 - 1, 2 * q2*q3 - 2 * q0*q1,
		2 * q1*q3 - 2 * q0*q2, 2 * q2*q3 + 2 * q0*q1, 2 * q0*q0 + 2 * q3*q3 - 1
	};

	Mat R_M(3, 3, CV_64F, (void *)R);
	warp3D = R_M*cameraMatrix.inv();
	warp3D.row(0) *= -h;
	warp3D.row(1) *= -h;

	warp3DInv = warp3D.inv();
	cout << cameraMatrix << endl << distMatrix << endl;
	//warp3DBias = warp3D;
	//warp3DBias.row(0) += 500;
	//warp3DBias.row(1) -= 1000;
}
void getPoint3D(double u, double v, double &x, double &y){
	double axisInImage[3] = { u, v, 1 };
	cv::Mat axisImage = cv::Mat(3, 1, CV_64FC1, axisInImage);
	cv::Mat_<double> divder = warp3D.row(2)*axisImage; double divderD = divder(0, 0);
	cv::Mat_<double> x_ = warp3D.row(0)*axisImage / divderD;
	cv::Mat_<double> y_ = warp3D.row(1)*axisImage / divderD;
	x = x_(0, 0);
	y = y_(0, 0);
}
void wrongPointDetect(double p[])
{
	
	for (int i = 1; i < 640-1; i++)
	{
		if (fabs(p[i - 1] - p[i + 1])<10 && fabs(p[i] - p[i - 1])>50)
			p[i] = (p[i + 1] + p[i - 1]) / 2;

	}
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  vector<double> p[640];
	vector<double> depth[1000];
	vector<double> worldx[1000];
	cv::namedWindow("BarValueThres");
	cv::namedWindow("video");
	string ss("");
	cv::VideoCapture videoCapture(0);
	videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	videoCapture.set(CV_CAP_PROP_EXPOSURE, -6);
	Mat frame;
	Mat distortframe;
	int index = 0;
	//显示视屏
	char c = 0;
	updatePara();
	
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::LaserScan>("laser", 1000);//const std::string& topic, uint32_t queue_size
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);//Rate(double frequency)
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
    
    videoCapture >> frame;
		
		//create bar
		createTrackbar(trackbar_value1,
			"BarValueThres", &threshold_value1,
			255, Threshold_Demo);

		createTrackbar(trackbar_value2,
			"BarValueThres", &threshold_value2,
			255, Threshold_Demo);
		createTrackbar(trackbar_value3,
			"BarValueThres", &threshold_value3,
			180, Threshold_Demo);

		createTrackbar(trackbar_value4,
			"BarValueThres", &threshold_value4,
			180, Threshold_Demo);
		//updatePara();
		//
		if (!frame.data)
			continue;
		Mat gray,gray1;
		cvtColor(frame, gray, CV_RGB2GRAY);
		inRange(gray, threshold_value1, threshold_value2, gray1);
	    //line detect
		Mat roiImageH;
		//cvtColor(gray1, roiImageH, CV_RGB2HSV);//just input your image here
		cvtColor(gray1, gray1, CV_GRAY2RGB);
		cvtColor(gray1, roiImageH, CV_RGB2HSV);
		vector<Mat> splited;
		split(roiImageH, splited);
		Mat image = splited[2];
		Ptr<LineSegmentDetector> lsd_std = createLineSegmentDetectorPtr(LSD_REFINE_STD);
		//double start = double(getTickCount());
		vector<Vec4i> lines_std,out_line;
		
		
		lsd_std->detect(image, lines_std);
		//filter lines
		lsd_std->filterOutAngle(lines_std, out_line,threshold_value3, threshold_value4);
		//
		for (int k = 0; k < 640; k++)
		{
			vector<double> getmin;
			p[k].clear();
			double x = k, y;
			for (int i = 0; i < out_line.size(); i++)
			{
				double x1 = out_line[i][0];
				double y1 = out_line[i][1];
				double x2 = out_line[i][2];
				double y2 = out_line[i][3];
				if (k >= x1&&k <= x2 || k >= x2&&k <= x1)
				{
					
					y = (x - x2) / (x1 - x2)*(y1 - y2) + y2;
					getmin.push_back(y);
				}
				

				//circle(frame, Point2f(x1, y1), 3, Scalar(255, 0, 0));
				//circle(frame, Point2f(x2, y2), 3, Scalar(255, 0, 0));
			}
			sort(getmin.begin(), getmin.end(), my_cmp);
			if (getmin.size() >= 4)
			{
				y = (getmin[0] + getmin[1]) / 2;
				p[k].push_back(y);
				y = (getmin[2] + getmin[3]) / 2;
				p[k].push_back(y);
			}
			else if (getmin.size() > 1)
			{
				y = (getmin[0] + getmin[1]) / 2;
				p[k].push_back(y);
			}
			else if (getmin.size() == 1)
			{
				p[k].push_back(getmin[0]);
			}
			else
				p[k].push_back(0);

		}
		//wrongPointDetect(p);
		Mat pic(1000, 1000, CV_8UC3);
		int count1 = 0;
		for (int k = 0; k < 640; k++)
		{
			
			depth[k].clear();
			worldx[k].clear();
			for (int i = 0; i < p[k].size(); i++)
			{
				double a, b;
				circle(frame, Point2f(k, p[k][i]), 3, Scalar(255, 0, 0));
				Mat p_origin, p_after;
				double aa[2] = { k, p[k][i] };
				p_origin.push_back(Mat(1, 1, CV_64FC2, aa));
				undistortPoints(p_origin, p_after, cameraMatrix, distMatrix);
				//cout << cameraMatrix << endl << distMatrix << endl;
				//cout << p_origin << endl;

				double x0 = p_after.at<double>(0, 0);
				double y0 = p_after.at<double>(0, 1);
				double fx = 388.2391, cx = 307.5625, fy = 385.5123, cy = 269.5769;
				x0 = x0*fx + cx;
				y0 = y0*fy + cy;
				//cout << x0<<" "<<y0 << endl;

				getPoint3D(x0, y0, a, b);
				depth[k].push_back(b);
				worldx[k].push_back(a);
				count1++;
			}
			    
				
				
				
				//cout << a << endl << b << endl;
				/*if (p[k] == 0)
					continue;
				circle(pic, Point2f(a/10+500, b/10), 3, Scalar(255, 0, 0));*/
				
				/*if (k % 50 == 0)
					cout << b << " ";*/
			
		}
		/*imshow("map", pic);*/

		//利用线性优化系数进行优化
		Mat x = Mat(1, 640, CV_64FC1, depth);
		//Mat result;
		//Mat A, B;
		//result = aa.mul(x).mul(x) + bb.mul(x) + cc;
		//result = aa.mul(x) + bb;
		for (int j = 0; j < 640; j += 50)
		{
			//cout << result.at<double>(0, j) << " ";
			for (int i = 0; i < p[j].size(); i++)
			{
				cout << "[" << worldx[j][i] << "," << depth[j][i] << "] ";
				circle(frame, Point2f(j, p[j][i]), 5, Scalar(0, 255, 0));
			}
			
		}
		for (int j = 0; j < 640; j++)
		{
			if (p[j][0] == 0)
				continue;
			for (int i = 0; i < worldx[j].size(); i++)
			{
				//circle(pic, Point2f(worldx[j][i] / 5 +600, result.at<double>(0, j) / 5+300), 3, Scalar(255, 0, 0));
				circle(pic, Point2f(worldx[j][i] / 5 + 600, depth[j][i] / 5 + 300), 3, Scalar(255, 0, 0));
			}
			
		}
		circle(pic, Point2f( 0 + 600, 0 + 300), 10, Scalar(0, 0, 255));
		flip(pic, pic, 0);
		imshow("map", pic);
		cout << endl;
		//cout << endl;
		
		//show line
		Mat drawnLines(frame);
		//lsd_std->drawSegments(drawnLines, out_line);
		//imshow("Standard refinement", drawnLines);

		imshow("rgb", frame);
		//imshow("video", gray);
		//imshow("extract", gray1);
		//imshow("disvideo", distortframe);
		if (c == 32){ //c==32
			/*	imwrite((num2str(index) + ".jpg").c_str(), frame);
				cout << index << endl;
				++index;
				Sleep(2000);*/
			static ofstream outfile;
			if (!outfile.is_open()) {
				cout << "not open" << endl;
				outfile.open("depth.txt", ios::out);//文件名改成自己的
			}
			for (int i = 0; i < 639; i++)
			{
				outfile << depth[i][0] << '\t';
			}
			outfile << depth[639][0] << endl;
		}

		c = cvWaitKey(30);
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;
    sensor_msgs::LaserScan scan_msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    scan_msg.header.frame_id="laser";
  scan_msg.angle_min = -3.1415926/4;
  scan_msg.angle_max = 3.1415926/4;
  scan_msg.angle_increment = 3.1415926/2/540;
  scan_msg.time_increment = 1;
  scan_msg.scan_time = 1;
  scan_msg.range_min = 0.5;
  scan_msg.range_max = 10;
  scan_msg.ranges.assign(540, std::numeric_limits<float>::quiet_NaN());
  for(int i=0;i<540;i++)
  {
    scan_msg.ranges[i]=100;
  }
  for(int i=0;i<640;i++)
  {
    
    for(int j=0;j<depth[i].size();j++)
    {
      double c=sqrt(depth[i][j]*depth[i][j]+worldx[i][j]*worldx[i][j]);
      double angle=3.1415926/2-acos(worldx[i][j]/c);
      //cout<<"angle:"<<angle<<endl;
      angle=angle+3.1415926/4;
      int k=angle/scan_msg.angle_increment;
      if(scan_msg.ranges[k]>c/1000&&k>=0&&k<540)
      scan_msg.ranges[540-k-1]=c/1000;  
    }
      
  }
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(scan_msg);
// %EndTag(PUBLISH)%
    static tf::TransformBroadcaster laser_broadcaster;
      tf::Transform laser_transform;
       laser_transform.setOrigin( tf::Vector3(0.3, 0, 0.05) );
        tf::Quaternion q;
       q.setRPY(0, 0, 0);
       laser_transform.setRotation(q);
       laser_broadcaster.sendTransform(tf::StampedTransform(laser_transform, ros::Time::now(), "base_link", "laser"));
     static tf::TransformBroadcaster map_broadcaster;
      tf::Transform map_transform;   
      map_transform.setOrigin( tf::Vector3(0, 0, 0) );
       map_transform.setRotation(q);
       map_broadcaster.sendTransform(tf::StampedTransform(laser_transform, ros::Time::now(), "map", "base_link"));
        //ROS_INFO("%s", "111");
// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%

