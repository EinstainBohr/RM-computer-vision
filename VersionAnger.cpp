#include <time.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <math.h>

#define PI 3.1415926

using namespace std;
using namespace cv;


int main()
{
    clock_t start,finish;
    double totaltime, heights[16];
    int hi = 0;
    VideoCapture capture("/home/einstein/桌面/save_12.avi");
    //VideoCapture capture(1);
    Mat frame, binary;
    RotatedRect RA[16], R[16];


    for(;;)
    {
        start = clock();
        capture>> frame;
        frame.copyTo(binary);

        cvtColor(frame,frame,COLOR_BGR2GRAY);

        threshold(frame, frame, 160, 255,cv::THRESH_BINARY);//调阈值就差不多了
        // Find all the contours in the thresholded image
        vector<vector<Point>> contours;
        imshow("sdfa",frame);
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


                 double max_height, min_height;
                 if(rrect.size.height > rrectA.size.height){
                     max_height = rrect.size.height;
                     min_height = rrectA.size.height;
                 }
                 else{
                     max_height = rrectA.size.height;
                     min_height = rrect.size.height;
                 }

                 double line_x = abs(rrect.center.x-rrectA.center.x);
                 double difference = max_height - min_height;
                 double aim =   distance/((highA+high)/2);
                 double difference3 = abs(rrect.size.width -rrectA.size.width);
                 double height = (rrect.size.height+rrectA.size.height)/200;
                 double slop_low = abs(rrect.angle + rrectA.angle)/2;


                 if((aim < 3.0 - height && aim > 2.0 - height     //小装甲板
                       && slop <= 5 && difference <=8 && difference3 <= 5
                      &&(slop_low <= 30 || slop_low >=150) && line_x >0.6*distance)
                      || (aim < 5.0-height && aim > 3.2 - height  //大装甲板
                          && slop <= 7 && difference <=15 && difference3 <= 8
                         &&(slop_low <= 30 || slop_low >=150) && line_x >0.7*distance)){

                     heights[hi] = (rrect.size.height+rrectA.size.height)/2;
                     R[hi] = rrect;
                     RA[hi] = rrectA;
                     hi++;
                 }
             }
        }


        double max = 0;
        int mark;
        for(int i = 0;i < hi;i++){     //多个目标存在，打更近装甲板
            if(heights[i]  >= max){
                max = heights[i];
                mark = i;
            }
        }
        if(hi != 0){
            cv::circle(binary,Point((R[mark].center.x+RA[mark].center.x)/2,
                       (R[mark].center.y+RA[mark].center.y)/2),
                       abs((R[mark].size.height+RA[mark].size.height)/4),cv::Scalar(0,0,255),4);

           double center_x = (R[mark].center.x+RA[mark].center.x)/2;
           double center_y = (R[mark].center.y+RA[mark].center.y)/2;
           double height_equal = abs(R[mark].center.x-RA[mark].center.x)/2;
           double width_equal =  abs((R[mark].size.height+RA[mark].size.height)/4);

           Mat cameraMatrix;
           Mat cameraDistCoeffs;
           FileStorage fs("/home/einstein/桌面/OpenCV3Cookbook-master/src/Chapter11/calib.xml", cv::FileStorage::READ);
           fs["Intrinsic"] >> cameraMatrix;
           fs["Distortion"] >> cameraDistCoeffs;
           //std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;

           vector<cv::Point3f> objectPoints;                //小装甲板空间坐标
           objectPoints.push_back(cv::Point3f(0, 0, 0));
           objectPoints.push_back(cv::Point3f(13.5, 0, 0));
           objectPoints.push_back(cv::Point3f(13.5, 5.6, 0));
           objectPoints.push_back(cv::Point3f(0, 5.6, 0));
           objectPoints.push_back(cv::Point3f(6.75,2.8, 0));

           vector<cv::Point3f> objectPoints2;                //大装甲板空间坐标
           objectPoints2.push_back(cv::Point3f(0, 0, 0));
           objectPoints2.push_back(cv::Point3f(23, 0, 0));
           objectPoints2.push_back(cv::Point3f(23, 5.6, 0));
           objectPoints2.push_back(cv::Point3f(0, 5.6, 0));
           objectPoints2.push_back(cv::Point3f(11.5,2.8, 0));


           vector<cv::Point2f> imagePoints;                 //装甲板像素坐标

           float x1 =  center_x - height_equal, y1= center_y - width_equal;
           float x2 =  center_x +  height_equal, y2= center_y + width_equal;

           imagePoints.push_back(Point2f(x1, y1));
           imagePoints.push_back(Point2f(x2, y1));
           imagePoints.push_back(Point2f(x2, y2));
           imagePoints.push_back(Point2f(x1, y2));
           imagePoints.push_back(Point2f(center_x, center_y));


           line(binary,Point(x1, y1),Point(x2, y1),cv::Scalar(0, 255, 0),4);
           line(binary,Point(x1, y2),Point(x2, y2),cv::Scalar(0, 255, 0),4);

           float whichone = abs(x1-x2)/abs(y1-y2);

           Mat rvec, tvec;
           if( whichone > 3.2)
              solvePnP(objectPoints2, imagePoints,cameraMatrix, cameraDistCoeffs, rvec, tvec);
           else
              solvePnP(objectPoints, imagePoints, cameraMatrix, cameraDistCoeffs, rvec, tvec);


           Mat rotation;
           // convert vector-3 rotation
           // to a 3x3 rotation matrix
           Rodrigues(rvec, rotation);

           float theta_x ,theta_y,theta_z;
           //根据旋转矩阵求出坐标旋转角
            theta_x = atan2(rotation.at<double>(2, 1), rotation.at<double>(2, 2));
            theta_y = atan2(-rotation.at<double>(2, 0),
            sqrt(rotation.at<double>(2, 1)*rotation.at<double>(2, 1) +rotation.at<double>(2, 2)*rotation.at<double>(2, 2)));
            theta_z = atan2(rotation.at<double>(1, 0), rotation.at<double>(0, 0));

               //将弧度转化为角度
            theta_x = theta_x * (180 / PI);
            theta_y = theta_y * (180 / PI);
            theta_z = theta_z * (180 / PI);


           cout << "x 角度 =  " << theta_x << endl;
           cout << "y 角度 =  " << theta_y << endl;
           cout << "z 角度 =  " << theta_z << endl <<endl;

        }

        imshow("okey",binary);
        waitKey(2);

        finish = clock();
        totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
        cout<<"Time whole"<<totaltime<<"秒！"<<endl;
        hi = 0;
    }

}
