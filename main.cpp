#include <time.h>
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iostream>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


int main()
{
    clock_t start,finish;
    double totaltime;
    VideoCapture capture("/home/einstain/桌面/步兵蓝2.avi");
    //VideoCapture capture(0);
    Mat frame;
    Mat binary;


    for(;;)
    {
        start = clock();
        capture>> frame;
        frame.copyTo(binary);

        cvtColor(frame,frame,COLOR_BGR2GRAY);

        //threshold(frame, frame, 200, 255, THRESH_BINARY | THRESH_OTSU);
        threshold(frame, frame, 230, 255,cv::THRESH_BINARY);//用老摄像头就调220左右，黑色的就调180左右
        // Find all the contours in the thresholded image
        vector<vector<Point>> contours;
        findContours(frame, contours, RETR_LIST, CHAIN_APPROX_NONE);


        for (size_t i = 0; i < contours.size(); i++){

            vector<Point> points;
            double area = contourArea(contours[i]);
            if (area < 20 || 1e3 < area) continue;
            drawContours(frame, contours, static_cast<int>(i), Scalar(0), 2);

            double high;
            points = contours[i];

            RotatedRect rrect = fitEllipse(points);
            cv::Point2f* vertices = new cv::Point2f[4];
                rrect.points(vertices);

                for (int j = 0; j < 4; j++)
                {
                    cv::line(binary, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),4);
                }

             //ellipse(binary,rrect,Scalar(0));
             high = rrect.size.height;


             for(size_t j = 1;j < contours.size();j++){

                 vector<Point> pointsA;
                 double area = contourArea(contours[j]);
                 if (area < 20 || 1e3 < area) continue;


                 double highA, distance;
                 double slop ;
                 pointsA = contours[j];

                 RotatedRect rrectA = fitEllipse(pointsA);

                 slop = abs(rrect.angle - rrectA.angle);
                 highA = rrectA.size.height;
                 distance  =sqrt((rrect.center.x-rrectA.center.x)*(rrect.center.x-rrectA.center.x)+
                                 (rrect.center.y-rrectA.center.y)*(rrect.center.y-rrectA.center.y));


                 double aim =   distance/((highA+high)/2);
                 double height_equal = rrect.size.height/rrectA.size.height;
                 double self =  (double)(rrect.size.height+rrect.size.height)/
                         (double)(rrectA.size.width+rrectA.size.width);


                 //cout << rrect.angle<<"  "<<rrectA.angle <<" "<< aim<<endl;
                 if(aim < 2.5 && aim > 2.0  && slop < 1 && height_equal > 0.95 && height_equal < 1.05){
                     //cv::circle(binary,rrect.center,15,cv::Scalar(0),4);
                     cv::circle(binary,Point((rrect.center.x+rrectA.center.x)/2,
                                (rrect.center.y+rrectA.center.y)/2),
                                15,cv::Scalar(0,0,255),4);
                     cout <<"恭喜恭喜恭喜恭喜恭喜恭喜恭喜恭喜恭喜恭喜恭喜"<<endl;
                 }
             }
        }
        imshow("Prossess image ",frame);
        imshow("Binary image ",binary);
        waitKey(2);

        finish = clock();
        totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
        cout<<"Time whole"<<totaltime<<"秒！"<<endl;
    }
}
