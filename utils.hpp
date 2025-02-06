using namespace std;

struct detectdata {
	bool detected;
	int maxconfi;
	float maxconf;
};

detectdata GetDetection(vector<Rect> &found, vector<int> &weights, vector<float> &conf) {
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
