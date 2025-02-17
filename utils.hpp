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

struct detectdata {
	bool detected;
	int maxconfi;
	float maxconf;
};

detectdata GetDetection(vector<Rect> &found, vector<int> &weights, vector<float> &conf, float min_conf) {
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

    detectdata dd;
    dd.detected = detected;
    dd.maxconfi = maxconfi;
    dd.maxconf = maxconf;

    return dd;
}
