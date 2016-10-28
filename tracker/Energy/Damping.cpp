#include "Damping.h"
#include "tracker/TwSettings.h"
#include "tracker/HModel/Model.h"

namespace energy {

	void Damping::init(Model * model) {
		this->model = model;
		tw_settings->tw_add(settings->translation_damping, "translations", "group=Damping");
		tw_settings->tw_add(settings->rotation_damping, "rotations", "group=Damping");
	}

	void Damping::track(LinearSystem &system) {

		Eigen::Matrix<Scalar, num_thetas, num_thetas> D = Eigen::Matrix<Scalar, num_thetas, num_thetas>::Identity(system.lhs.rows(), system.lhs.cols());
		Eigen::Matrix<Scalar, num_thetas, 1>  d = Eigen::Matrix<Scalar, num_thetas, 1>::Ones(system.lhs.rows(), 1);

		float max_JtJ = 0;
		/*for (int i = 0; i < num_thetas; ++i) {
			if (max_JtJ < abs(system.lhs(i, i)))
				max_JtJ = abs(system.lhs(i, i));
		}*/
	
		for (int i = 0; i < num_thetas; ++i) {
			if (model->dofs[i].type == TRANSLATION_AXIS) 
				d(i) = settings->translation_damping;
			else 
				d(i) = settings->rotation_damping;
			if (i == 13 || i == 17 || i == 21 || i == 25) d(i) = settings->abduction_damping;
			if (i == 16 || i == 20 || i == 24 || i == 28) d(i) = settings->top_phalange_damping;
			
			//d(i) = d(i) * system.lhs(i, i) / max_JtJ;			
		}
		//cout << d.transpose() << endl;
		D.diagonal() = d;
		system.lhs = system.lhs + D;

		if (Energy::safety_check) Energy::has_nan(system);
	}

} 
