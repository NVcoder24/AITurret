#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "SimpleSerial.h"
#include <windows.h>
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

float min_conf = .7;

int main() {
    auto net = cv::dnn::DetectionModel("C:/cocods2020/frozen_inference_graph.pb", "C:/cocods2020/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt");
    net.setInputSize(320, 320);
    net.setInputScale(1.0 / 127.5);
    net.setInputMean(Scalar(127.5, 127.5, 127.5));
    net.setInputSwapRB(true);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    const WCHAR* PortSpecifier = GetWC("COM9");

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

    while (true) {
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
        bool detected = false;

        int maxconfi = -1;
        float maxconf = 0;

        for (size_t i = 0; i < found.size(); i++) {
            if (weights[i] == 1 && conf[i] > min_conf) {
                if (conf[i] > maxconf) {
                    maxconf = conf[i];
                    maxconfi = i;
                }
                detected = true;
            }
        }

        if (detected) {
            int i = maxconfi;
            rectangle(frame, found[i], cv::Scalar(0, 0, 255), 2);
            rectangle(frame1, Rect2i(found[i].x * scalefactor, found[i].y * scalefactor, found[i].width * scalefactor, found[i].height * scalefactor), cv::Scalar(0, 0, 255), 2);
            stringstream temp;
            temp << conf[i];
            putText(frame1, temp.str(), Point(found[i].x * scalefactor, found[i].y * scalefactor + 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255));
            lastpoint = Point(int(found[i].x * scalefactor + found[i].width * scalefactor / 2), int(found[i].y * scalefactor + found[i].height * scalefactor * .3));
            circle(frame1, lastpoint, 3, Scalar(255, 0, 0), 2);

            float xshift = lastpoint.x - defpoint.x;
            float yshift = lastpoint.y - defpoint.y;

            float len = std::sqrt(pow(xshift, 2) + pow(yshift, 2));

            if (len < 40) {
                islockedin = true;
                putText(frame1, "LOCKED", defpoint, FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 3);
            }

            float xnorm = xshift / len;
            float ynorm = yshift / len;

            line(frame1, defpoint, lastpoint, Scalar(0, 255, 0));
            line(frame1, defpoint, Point(int(defpoint.x + xnorm * 20), int(defpoint.y + ynorm * 20)), Scalar(255, 0, 0), 2);

            posx += xnorm * len / 100;
            posy += ynorm * len / 100;

            frameswithoutdetection = 0;
        } else {
            if (frameswithoutdetection == 0) {
                
            }
            if (frameswithoutdetection == 10) {
                posx = 0;
                posy = 0;
            }   
            if (frameswithoutdetection > 10) {
                posx += .5 * (float)mul;
                if (posx > 30 || posx < - 30) {
                    mul = mul * -1;
                }
            }

            frameswithoutdetection += 1;
        }

        cv::imshow("AITurret", frame1);
        cv::imshow("AITurret VISION", frame);

        posx = min(max(posx, -50), 50);
        posy = min(max(posy, -40), 40);

        std::string data = std::to_string(posx) + "," + std::to_string(posy) + ";";
        const char* data1 = data.data();
        bool retVal = WriteFile(hPort, data1, strlen(data1), &byteswritten, NULL);

        if ((cv::waitKey(1) & 0xFF) == 27) {
            break;
        }
    }

    cv::destroyAllWindows();

    CloseHandle(hPort);

    return 0;
}