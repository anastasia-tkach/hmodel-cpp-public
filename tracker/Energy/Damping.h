#pragma once
#include "tracker/ForwardDeclarations.h"
#include "Energy.h"
#include "tracker/Types.h"
#include <vector>

namespace energy{

class Damping : public Energy{
	Model * model = NULL;

public:
    struct Settings {
		Scalar translation_damping = 1;
		Scalar rotation_damping = 3000;
		Scalar abduction_damping = 1500000; 
		Scalar top_phalange_damping = 10000;
    } _settings;
    Settings*const settings = &_settings;


    void init(Model * model);
    void track(LinearSystem& system);
};

}
