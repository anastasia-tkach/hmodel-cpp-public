#include "DataStream.h"
#include <QFileDialog>
#include <QByteArray>
#include <QFile>

#include "util/qt2eigen.h"
#include "util/mylogger.h"
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <iomanip>

DataStream::DataStream(Camera *camera) : _camera(camera){
    assert( camera != NULL);
}

DataStream::~DataStream(){
    for(uint i=0; i<frames.size(); i++)
        delete frames.at(i); 
}

int DataStream::add_frame(const void* color_buffer, const void* depth_buffer, const void* full_color_buffer) {
    frames.push_back( new DataFrame(frames.size()) );
    DataFrame& frame = *frames.back();
    
    /// Clone the data
    if(color_buffer) frame.color = cv::Mat(height(), width(), CV_8UC3, (void*) color_buffer).clone();
    if(depth_buffer) frame.depth = cv::Mat(height(), width(), CV_16UC1, (void*) depth_buffer).clone();
	if (full_color_buffer) frame.full_color = cv::Mat(height() * 2, width() * 2, CV_8UC3, (void*)full_color_buffer).clone();
    if(!color_buffer) qDebug() << "warning: null color buffer?";
    if(!depth_buffer) qDebug() << "warning: null depth buffer?";
    
    /// Signal system to update GUI
    return (frames.size()-1);
}

void DataStream::save_as_images(std::string path) {	

	for (size_t i = 0; i < frames.size(); i++) {
		std::ostringstream stringstream;
		stringstream << std::setw(7) << std::setfill('0') << i;
		std::string filename;
		filename = path + "depth-" + stringstream.str() + ".png";
		cv::imwrite(filename, frames.at(i)->depth);
		filename = path + "color-" + stringstream.str() + ".png";
		cv::imwrite(filename, frames.at(i)->color);
		if (frames.at(i)->full_color.data) {
			filename = path + "full_color-" + stringstream.str() + ".png";
			cv::imwrite(filename, frames.at(i)->full_color);
		}
		//filename = path + "mask-" + stringstream.str() + ".png";
		//cv::imwrite(filename, frames.at(i)->silhouette);
		cout << filename << endl;
	}
	
	/*for (size_t i = 0; i < current.rows; i++) {
		for (size_t j = 0; j < current.cols; j++) {
			uint16_t a = current.at<uint16_t>(i, j);
			cout << a << " ";
		}
	}*/
}
