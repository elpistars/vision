// Threshold CIE.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <algorithm>
#include "opencv-3.3.1-dev/opencv2/highgui.hpp"
#include "opencv-3.3.1-dev/opencv2/imgproc.hpp"
#include "ros/ros.h"

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
using namespace std;
using namespace cv;
void thresh_callback(int, void*,Mat src );
void hough_draw(Mat src, Mat &dst);
int main( int argc, char** argv ){
    int iLowL = 0;
    int iHighL = 256;
    int iLowa = 0; 
    int iHigha = 256;
    int iLowb = 0;
    int iHighb = 256;
    ros::init(argc, argv, "tuning_node");

    VideoCapture cap(0); //capture the video from web cam
    cap.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"



	//Create trackbars in "Control" window
	createTrackbar("LowL", "Control", &iLowL, 256); //L (-128 - 128)
	createTrackbar("HighL", "Control",	&iHighL, 256 );

	createTrackbar("Lowa", "Control", &iLowa, 256); //a (-128 - 255)
	createTrackbar("Higha", "Control", &iHigha, 256 );
	createTrackbar("Lowb", "Control", &iLowb, 256); //b (-128 - 255)
	createTrackbar("Highb", "Control", &iHighb, 256 );
    

    while (1)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

	    Mat imgHSV, imgLAB;
	    cvtColor(imgOriginal, imgHSV, COLOR_BGR2Lab);
        imgLAB=imgHSV.clone();
        Rect Rec((FRAME_WIDTH/2)-100,380,200,100);
        rectangle(imgHSV,Rec,Scalar(100,100,100),1,8,0);
        Mat imgThresholded;
        Mat imgOutput=imgOriginal.clone();
        imgHSV=imgHSV(Rec);
        inRange(imgHSV, Scalar(iLowL, iLowa, iLowb), Scalar(iHighL, iHigha, iHighb), imgThresholded); //Threshold the image
      
  //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

   //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      
        
        rectangle(imgOutput,Rec,Scalar(255,0,0),1,8,0);
        imgOutput=imgOutput(Rec);
        hough_draw(imgThresholded,imgOutput);
        line(imgOutput,Point(imgOutput.size().width/2,0),Point(imgOutput.size().width/2,FRAME_HEIGHT),Scalar(255,0,0),3,CV_AA);
        
        //thresh_callback(0,0,imgThresholded);
        //Mat imcontrol;
        Mat imcontrol_LAB_H(100,300,CV_8UC3,Scalar(iHighL,iHigha,iHighb));
        //Mat imcontrol_LAB_L(100,300,CV_8UC3,Scalar(iLowL,iLowa,iLowb));
        //cvtColor(imcontrol_LAB,imcontrol,COLOR_Lab2BGR);
        imshow("Control", imcontrol_LAB_H);
        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        imshow("LAB Image",imgLAB);        
        imshow("Output", imgOutput); //show the original image

        //imshow("Control", imcontrol_LAB_L);
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break; 
        }
    }
}

void hough_draw(Mat src,Mat &dst){
  Mat canny_output;
  Point line_pos[2];
  int tengah;
  int line_count=0;
  Canny(src,canny_output,50,200,3);
  vector<Vec2f> lines;
  HoughLines(canny_output,lines, 1,CV_PI/180,50,0,0);
  cout<<"lines size= "<<lines.size()<<endl;
  for (size_t i=0; i<lines.size();i++){
      float rho = lines[i][0], theta=lines[i][1];
      Point pt1, pt2;
      double a=cos(theta), b=sin(theta);
      double x0=a*rho, y0 =b*rho;
      pt1.x=cvRound(x0+120*(-b));
      pt1.y=cvRound(y0+120*(a));
      pt2.x=cvRound(x0-120*(-b));
      pt2.y=cvRound(y0-120*(a));
   //   cout<<"i="<<i<<" "<<"pt1 : "<<pt1.x<<endl<<"pt2 : "<<pt2.x<<endl<<endl;
      line(dst,pt1,pt2,Scalar(0,0,255),3,CV_AA);
      if(max(pt1.x,pt2.x)<(src.size().width/2)){
        line_pos[0].x=max(max(pt1.x,pt2.x),line_pos[0].x);
      }
      else if(min(pt1.x,pt2.x)>(src.size().width/2)){
        line_pos[1].x=max(min(pt1.x,pt2.x),line_pos[1].x);
      }
  }

/*   cout<<"0 :"<<line_pos[0]<<endl<<"1 :"<<line_pos[1]<<endl;
  if(line_pos[0].x<src.size().width/2){
      line_pos[1]=line_pos[0];
      line_pos[0].x=0;
  }
  else{}
  if(line_pos[1].x>src.size().width/2){
      line_pos[0]=line_pos[1];
      line_pos[1].x=0;
  }
  else{} */
  cout<<"kiri :"<<line_pos[0]<<endl<<"kanan :"<<line_pos[1]<<endl;
  if((line_pos[0].x)==0&&(line_pos[1].x!=0)) tengah=1;  
  else if((line_pos[1].x==0)&&(line_pos[0].x!=0)) tengah=-1;
  else tengah=(line_pos[0].x+line_pos[1].x)/2;
  cout<<tengah<<endl;
  line(dst,Point(tengah,0),Point(tengah,dst.size().height),Scalar(0,255,0),3,8);

  return;
}