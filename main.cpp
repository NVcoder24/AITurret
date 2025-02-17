#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "SimpleSerial.h"
#include <windows.h>
#include "utils.hpp"
using namespace std;
using namespace cv;
using namespace cv::ml;

const wchar_t* GetWC(const char* c) {
    const size_t cSize = strlen(c) + 1;
    wchar_t* wc = new wchar_t[cSize];
    mbstowcs(wc, c, cSize);

    return wc;
}

int W = 600;
int H = 400;

float posx = 0;
float posy = 0;

void Draw(int event, int x, int y, int flags, void* param) {
    posx = std::ceil(((W / 2 - (float)x) / (W / 2) * (float)-30) * 100.0) / 100.0;
    posy = std::ceil(((H / 2 - (float)y) / (H / 2) * (float)-30) * 100.0) / 100.0;
}

float scalefactor = 1.5;

float min_conf = .6;

int main() {
    auto net = cv::dnn::DetectionModel("C:/cocods2020/frozen_inference_graph.pb", "C:/cocods2020/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt");
    net.setInputSize(320, 320);
    net.setInputScale(1.0 / 127.5);
    net.setInputMean(Scalar(127.5, 127.5, 127.5));
    net.setInputSwapRB(true);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    const WCHAR* PortSpecifier = GetWC("COM7");

    DCB dcb;
    DWORD byteswritten;
    HANDLE hPort = CreateFile(
        PortSpecifier,
        GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );
    if (!GetCommState(hPort, &dcb))
        return -1;

    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;

    if (!SetCommState(hPort, &dcb))
        return -2;

    PurgeComm(hPort, PURGE_RXCLEAR | PURGE_TXCLEAR);

    int frameswithoutdetection = 0;

    int mul = 1;

    cv::Mat img = Mat::zeros(cv::Size(W, H), CV_8SC1);
    cv::Mat frame1;

    cv::VideoCapture camera(1);

    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

    vector<Point> track;

    namedWindow("AITurret", WINDOW_AUTOSIZE);
    namedWindow("AITurret VISION", WINDOW_AUTOSIZE);
    camera >> frame1;

    bool lastAccel = false;
    bool lastPush = false;
    bool pushPos = false;

    int lastFireFrames = 0;

    while (true) {
        bool needAccel = false;
        bool needToFire = false;

        camera >> frame1;

        Mat frame = frame1.clone();
        
        cv::resize(frame, frame, cv::Size(int(640 / scalefactor), int(480 / scalefactor)));

        vector<Rect> found;
        vector<int> weights;
        vector<float> conf;

        net.detect(frame, weights, conf, found);

        Point lastpoint = Point(int(640 / 2), int(480 / 2));
        Point defpoint = Point(int(640 / 2), int(480 / 2));

        bool islockedin = false;

        detectdata dd = GetDetection(found, weights, conf, min_conf);

        bool detected = dd.detected;
        int maxconfi = dd.maxconfi;
        float maxconf = dd.maxconf;

        float len = 0;

        float addX = 0;
        float addY = 0;

        if (detected) {
            int i = maxconfi;
            rectangle(frame, found[i], cv::Scalar(0, 0, 255), 2);
            rectangle(frame1, Rect2i(found[i].x * scalefactor, found[i].y * scalefactor, found[i].width * scalefactor, found[i].height * scalefactor), cv::Scalar(0, 0, 255), 2);
            stringstream temp;
            temp << conf[i];
            putText(frame1, temp.str(), Point(found[i].x * scalefactor, found[i].y * scalefactor + 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
            lastpoint = Point(int(found[i].x * scalefactor + found[i].width * scalefactor / 2), int(found[i].y * scalefactor + found[i].height * scalefactor * .1));
            circle(frame1, lastpoint, 3, Scalar(255, 0, 0), 2);

            float xshift = lastpoint.x - defpoint.x;
            float yshift = lastpoint.y - defpoint.y;

            len = std::sqrt(pow(xshift, 2) + pow(yshift, 2));

            if (len < 40) {
                islockedin = true;
                putText(frame1, "LOCKED", defpoint, FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 3);
            }

            float xnorm = xshift / len;
            float ynorm = yshift / len;

            line(frame1, defpoint, lastpoint, Scalar(0, 255, 0));
            line(frame1, defpoint, Point(int(defpoint.x + xnorm * 20), int(defpoint.y + ynorm * 20)), Scalar(255, 0, 0), 2);

            addX = xnorm * pow(max(min(len / 50, 1), -1), 1.5) * .5;
            addY = ynorm * pow(max(min(len / 50, 1), -1), 1.5) * .5;

            if (len > 150) {
                addX = xnorm * pow(max(min(len / 50, 1), -1), 1.5) * .9;
                addY = ynorm * pow(max(min(len / 50, 1), -1), 1.5) * .9;
            }

            if (len < 20) {
                addX = 0;
                addY = 0;
            }

            needAccel = true;
            if (len < 20) {
                needToFire = true;
            }

            if (abs(addX) > 5) {
                addX = 0;
            }

            if (abs(addY) > 5) {
                addY = 0;
            }

            posx += addX;
            posy += addY;

            if (needToFire) {
                if (lastFireFrames > 10) {
                    pushPos = !pushPos;
                    lastFireFrames = 0;
                }
                lastFireFrames++;
            }

            frameswithoutdetection = 0;
        }
        else {
            pushPos = false;
            lastFireFrames = 0;

            if (frameswithoutdetection == 0) {

            }
            if (frameswithoutdetection == 10) {
                posx = 0;
                posy = 0;
            }
            if (frameswithoutdetection > 10) {
                posx += .5 * (float)mul;
                if (posx > 30 || posx < -30) {
                    mul = mul * -1;
                }
            }

            frameswithoutdetection += 1;
        }

        cv::imshow("AITurret", frame1);
        cv::imshow("AITurret VISION", frame);

        posx = min(max(posx, -50), 50);
        posy = min(max(posy, -40), 40);

        Mat df(300, 500, CV_8UC3, Scalar(0));

        putText(df, "X: " + to_string(posx) + " Y: " + to_string(posy), Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        putText(df, "Len: " + to_string(len), Point(10, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        if (islockedin) {
            putText(df, "LOCKED", Point(10, 90), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        if (needAccel) {
            putText(df, "ACCEL", Point(10, 120), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        if (needToFire) {
            putText(df, "!FIRE", Point(10, 150), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        if (pushPos) {
            putText(df, "PUSHED IN", Point(10, 180), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
        }
        putText(df, "addX: " + to_string(addX) + " addY: " + to_string(addY), Point(10, 210), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);

        cv::imshow("DEBUG", df);

        std::string data = "1," + std::to_string(posx) + "," + std::to_string(posy) + ";";
        const char* data1 = data.data();
        bool retVal = WriteFile(hPort, data1, strlen(data1), &byteswritten, NULL);

        if (needAccel != lastAccel) {
            std::string data = "3,0;";
            if (needAccel) {
                data = "3,1;";
            }
            const char* data1 = data.data();
            bool retVal = WriteFile(hPort, data1, strlen(data1), &byteswritten, NULL);
        }
        lastAccel = needAccel;

        if (pushPos != lastPush) {
            std::string data = "2,160;";
            if (pushPos) {
                data = "2,20;";
            }
            const char* data1 = data.data();
            bool retVal = WriteFile(hPort, data1, strlen(data1), &byteswritten, NULL);
        }
        lastPush = pushPos;

        if ((cv::waitKey(1) & 0xFF) == 27) {
            break;
        }
    }

    cv::destroyAllWindows();

    CloseHandle(hPort);

    return 0;
}
