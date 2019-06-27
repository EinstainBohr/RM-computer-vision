#include <iostream>
#include <opencv2/opencv.hpp>
#include <string.h>
#include <sstream>

using namespace cv;
using namespace std;


int main()
{
    VideoCapture cap(0);
    Mat image;
    int i = 0;
    namedWindow("nihao");
    for(;;){
        cap.read(image);
        cvtColor(image,image,COLOR_RGB2GRAY);
        imshow("nihao",image);
        char c = waitKey(33);
        if (c == 'c') {
            i++;
            stringstream ss;
            string convert;
            ss << i;
            ss >>convert;
            imwrite("/home/einstein/桌面/vision4/carmera_blue" + convert +".jpg",image);
        }
    }
    return 0;
}
