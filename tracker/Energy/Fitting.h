#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "tracker/Energy/Fitting/Settings.h"
#include "Fitting/DistanceTransform.h"
struct MappedResource;

namespace energy{
class Fitting {
protected:
    Camera* camera = NULL;
    DepthTexture16UC1* sensor_depth_texture = NULL;
    HandFinder* handfinder = NULL;
	Model * model = NULL;

public:
    fitting::Settings _settings;
    fitting::Settings*const settings = &_settings;
	DistanceTransform distance_transform;

public:
#ifdef WITH_CUDA
    void track(DataFrame &frame, LinearSystem &sys, bool rigid_only, bool eval_error, float &push_error, float &pull_error, int iter);
    void init(Worker* worker);
    void cleanup();
	~Fitting();
#else
    void track(DataFrame &frame, LinearSystem &sys, bool rigid_only, bool eval_error, float &push_error, float &pull_error){}
    void init(Worker* worker){}
    void cleanup(){}
#endif
};
} /// energy::
