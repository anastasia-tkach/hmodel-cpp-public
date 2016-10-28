#pragma once
#include <queue>
#include <fstream>
#include "tracker/Types.h"

class TrackingMonitor{
private:
   std::queue<float> failure_metric_history;
   float failure_metric_sum = 0;
   int moving_window_size = 10;
   

public:
	bool is_failure_frame(float pull_error, float push_error, bool fit2D_enable) {
		float metric, threshold;
		/*if (fit2D_enable) {
			Vector3 gamma = Vector3(23.6f, 8.20843f, -4.07201f); // gamma(0) = 30 for DeepPrior, gamma = 23.6
			Vector3 x = Vector3(1.0, push_error, pull_error);
			/// Failure: 0 , Success: 1
			metric = 1.0 / (1.0 + pow((float)exp(1.0), -x.transpose() * gamma));
			threshold = 0.5;
			if (metric < threshold) metric = 0;
			else metric = 1;
		}
		else {*/
			metric = pull_error;
			threshold = 6.0;
			if(metric > threshold) metric = 0;
			else metric = 1;
		//
	
        // ofstream errors_file;
        //if (!errors_file.is_open()) errors_file.open("...", ios::app);
        //errors_file << push_error << " " << pull_error << " " << metric << endl;

        failure_metric_history.push(metric);
        failure_metric_sum = failure_metric_sum + metric;
        if (failure_metric_history.size() > moving_window_size) {
            failure_metric_sum = failure_metric_sum - failure_metric_history.front();
            failure_metric_history.pop();
        }		
        if ((failure_metric_history.size() == moving_window_size) && (failure_metric_sum == 0))
            return true;
        return false;
    }
};

