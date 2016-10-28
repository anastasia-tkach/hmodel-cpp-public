#pragma once
#include <vector>
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "tracker/Energy/Energy.h"
class SolutionQueue;

namespace energy{
class Temporal : public Energy{
	Model * model = NULL;	

    SolutionQueue* solution_queue = NULL;
    std::vector<int> joint_ids;
	std::vector<int> center_ids;
	std::vector<int> phalange_ids;
    std::vector<Vector3> pos_prev1;
    std::vector<Vector3> pos_prev2;
    int fid_curr = -1;

public:
    struct Settings{
        bool temporal_coherence1_enable = true;
        bool temporal_coherence2_enable = true;
        float temporal_coherence1_weight = 0.05;
        float temporal_coherence2_weight = 0.05;
    } _settings;
    Settings*const settings = &_settings;

public:
	void init(Model * model);
    ~Temporal();
    void track(LinearSystem& system, DataFrame& frame);
    void update(int frame_id, const std::vector<Scalar>& Solution);
private:
	void track(LinearSystem& system, int fid, bool first_order);
	void temporal_coherence_init();
};

} /// energy::
