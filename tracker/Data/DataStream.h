#pragma once
#include <QList>

#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "Camera.h"
#include "DataFrame.h"
#include <QString>

class DataStream{       
private:
    QList<DataFrame*> frames;
private:
    Camera* _camera;
public:
    Camera& camera(){ return *_camera; }
public:
    int width() const { return _camera->width(); }
    int height() const { return _camera->height(); }
    int size() const{ return frames.size(); }
public:
	int add_frame(const void* color_buffer, const void* depth_buffer, const void* full_color_buffer);
public:
    DataStream(Camera* camera);
    ~DataStream();
public:
	void save_as_images(std::string path);
};



