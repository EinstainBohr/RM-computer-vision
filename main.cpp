#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


using namespace  cv;
using namespace std;

#define armorW 135

#define DEVICE "/dev/ttyTHS2" //这里改成linux里dev 的ttyxxx串口项
                                //妙算UART2串口
int serial_fd = 0;

void f(const int &a)
{
    cout<< a <<endl;
}
char init_serial(void)
{
    serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);  //是以读写方式、不把该文件作为终端设备、无延时模式打开串口1.
                                // O_NOCTTY 如果p a t h n a m e指的是终端设备，则不将此设备分配作为此进程的控制终端。
                                //http://zhidao.baidu.com/link?url=Oy8DY3HhPz09ZryPITubleqm2_jL5cAChlFyRnPxry_awKk6HpoicQGmyBEmNlNN6JLtEIamLoLwS3uvI_86Aa
    if (serial_fd < 0) {
        perror("open");
        return -1;
    }
    struct termios options;
    tcgetattr(serial_fd, &options);  //成功返回零；失败返回非零

    options.c_cflag |= (CLOCAL | CREAD);//忽略解制解调器状态行；  启用接收装置
    options.c_cflag &= ~CSIZE;		//字符大小屏蔽
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CS8;		//http://www.cnblogs.com/dartagnan/archive/2013/04/25/3042417.html
    options.c_cflag &= ~CSTOPB;//送两个停止位，否则为1位；
    options.c_iflag |= IGNPAR;//忽略奇偶错字符

    options.c_oflag = 0;
    options.c_lflag = 0;
    cfsetospeed(&options, B115200);
    tcflush(serial_fd, TCIFLUSH);
    tcsetattr(serial_fd, TCSANOW, &options);
    return 0;
}



int uart_recv(int fd, char *data, int datalen)
{
    int len = 0, ret = 0;
    fd_set fs_read;
    struct timeval tv_timeout;
    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);
    tv_timeout.tv_sec = (10 * 20 / 9600 + 2);
    tv_timeout.tv_usec = 0;
    ret = select(fd + 1, &fs_read, NULL, NULL, &tv_timeout);
    printf("ret = %d\n", ret);
    if (FD_ISSET(fd, &fs_read))
    {
        len = read(fd, data, datalen);
        printf("len = %d\n", len);
        return len;
    }
    else
    {
        perror("select");
        return -1;
    }
    return 0;
}
/********************************************************
xx	xx	xx	xx	xx
|	|	|	|	|
|	|	|	|	|
|	|	|	|	|
|	|	|	|	|
|	|	|	|	|
|	|	|	|	|--------------------checksum=sum%255
|	|	|	|------------------------value_L
|	|	|----------------------------value_H
|	|--------------------------------data
|------------------------------------fixed=0xff
********************************************************/
unsigned char check_sum(unsigned char *const pbuf, unsigned int num)
{
    unsigned int ii, sum;
    sum = 0;
    for (ii = 0; ii < num; ii++)
    {
        sum += pbuf[ii];
        sum %= 255;
    }
    return sum;
}

void SendAngle(double data1,double data2)
{
    unsigned char buf[255],len;
    buf[8]=(int)data1+(int)data2;
    //cout<<"sum="<<(int)data1+(int)data2<<endl;
    memcpy(&buf,&data1,sizeof(data1));
    memcpy(&buf[sizeof(data1)],&data2,sizeof(data2));
    write(serial_fd, buf, sizeof(data1)+sizeof(data2)+1);//write在出错的时候返回-1,调用成功但并没有写入数据返回0,大于0表示实际写入的字节数。
}
//*******************************//
//*********以上是串口初始化*******//
//*******************************//



int main()
{
    init_serial();
    clock_t start,finish;
    double totaltime, heights[16];
    int hi = 0;
    //VideoCapture capture("/home/einstein/桌面/save_10.avi");
    VideoCapture capture(0);
    Mat frame, binary;
    RotatedRect RA[16], R[16];


    for(;;)
    {
        start = clock();
        capture>> frame;
        frame.copyTo(binary);

        cvtColor(frame,frame,COLOR_BGR2GRAY);

        threshold(frame, frame, 210, 255,cv::THRESH_BINARY);//用老摄像头就调220左右，黑色的就调180左右
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

                 double difference = max_height - min_height;
                 double width_q = 2.43*max_height;
                 double difference1 = sqrt(abs(width_q*width_q-distance*distance))/6;
                 double aim =   distance/((highA+high)/2);
                 double difference3 = abs(rrect.size.width -rrectA.size.width);
                 double height = (rrect.size.height+rrectA.size.height)/200;
                 double slop_low = abs(rrect.angle + rrectA.angle)/2;


                 if(aim < 3.0 - height && aim > 2.0 - height  && slop <= 5 && difference <=difference1
                       && difference3 < 5 &&(slop_low <= 30 || slop_low >=150)){
                     heights[hi] = (rrect.size.height+rrectA.size.height)/2;
                     R[hi] = rrect;
                     RA[hi] = rrectA;
//                     cout << "height_diffrence  "<<difference<< "    " << difference1 <<endl <<endl;
//                     cout <<"slop_low  "  << slop_low<<"  " <<endl;
//                     cout <<"width_difference  " <<difference3 <<endl;
//                     cout <<"aim  " <<aim <<endl;
//                     cout <<"slop" <<slop <<endl;
//                     cout <<"angle" <<rrect.angle <<" " <<rrectA.angle <<endl;
//                     cout <<"height" <<height <<endl;
                     hi++;
                 }
             }
        }


        double max = 0;
        int mark;
        for(int i = 0;i < hi;i++){
            if(heights[i]  >= max){
                max = heights[i];
                mark = i;
            }
        }
        if(hi != 0){
            cv::circle(binary,Point((R[mark].center.x+RA[mark].center.x)/2,
                       (R[mark].center.y+RA[mark].center.y)/2),
                       15,cv::Scalar(0,0,255),4);

           //cout  << abs(R[mark].center.x-RA[mark].center.x);
           double lessx  =  320 - (R[mark].center.x+RA[mark].center.x)/2;
           double lessy =   240 - (R[mark].center.y+RA[mark].center.y)/2;
           //cout <<  lessx << "  " << lessy << " " << down <<endl;

           SendAngle(lessx,lessy);
        }

        imshow("okey",binary);
        waitKey(2);

        finish = clock();
        totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
        //cout<<"Time whole"<<totaltime<<"秒！"<<endl;
        hi = 0;
    }
    close(serial_fd);
}

