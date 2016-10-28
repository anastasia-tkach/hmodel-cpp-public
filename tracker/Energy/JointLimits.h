#pragma once
#include "tracker/ForwardDeclarations.h"
#include "Energy.h"
#include "tracker/Types.h"
#include <vector>

namespace energy{

class JointLimits : public Energy{
    SkeletonSerializer* skeleton = NULL;
	Model * model = NULL;
	bool htrack;
public:
    bool jointlimits_enable = false;
private:
    float jointlimits_weight = 10000000;
	//float jointlimits_weight = 100000;
    bool jointlimits_show_constraints = false;

public:
    void init(Model * model);
    void track(LinearSystem& sys, const std::vector<Scalar> &theta_0);
};

}
