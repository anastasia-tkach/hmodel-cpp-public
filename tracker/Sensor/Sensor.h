#pragma once
#include <QObject>


#include "tracker/HandFinder/HandFinder.h"

struct DataFrame;
class Camera;

class Sensor{
protected:
    bool initialized;
	bool real_color;
    const Camera * camera;
public:
	HandFinder * handfinder;


public:   
    Sensor(Camera* camera): initialized(false), camera(camera) {}
	Sensor(Camera* camera, bool real_color) : initialized(false), camera(camera) {}
    virtual ~Sensor(){}
    virtual bool spin_wait_for_data(float timeout_seconds) = 0;
    virtual bool fetch_streams(DataFrame& frame) = 0;
	virtual bool concurrent_fetch_streams(DataFrame &frame, HandFinder & handfinder, cv::Mat & full_color) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
private:
    virtual int initialize() = 0;
};

class SensorOpenNI : public Sensor{
public:
    SensorOpenNI(Camera* camera);
    virtual ~SensorOpenNI();
    bool spin_wait_for_data(float timeout_seconds);
    bool fetch_streams(DataFrame& frame);
	bool concurrent_fetch_streams(DataFrame &frame, HandFinder & handfinder, cv::Mat & full_color) { return 0; }
    void start(){}
    void stop(){}
private:
    int initialize();
};

class SensorSoftKin : public Sensor{
public:
    SensorSoftKin(Camera* camera);
    virtual ~SensorSoftKin();
    bool spin_wait_for_data(float timeout_seconds);
    bool fetch_streams(DataFrame& frame);
	bool concurrent_fetch_streams(DataFrame &frame, HandFinder & handfinder, cv::Mat & full_color) { return 0; }
    void start(); ///< calls initialize
    void stop();
private:
    int initialize();
};

class SensorRealSense : public Sensor {
public:
	SensorRealSense(Camera* camera);
	SensorRealSense(Camera* camera, bool real_color);
	virtual ~SensorRealSense();
    bool spin_wait_for_data(float timeout_seconds);
	bool fetch_streams(DataFrame& frame); 
	bool concurrent_fetch_streams(DataFrame &frame, HandFinder & handfinder, cv::Mat & full_color);
	bool run();
	void start(); ///< calls initialize
	void stop();
private:
	int initialize();
};

