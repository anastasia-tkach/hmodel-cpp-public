#include "JointLimits.h"
#include "tracker/Hmodel/Model.h"

void energy::JointLimits::init(Model * model) {
	this->model = model;
}

void energy::JointLimits::track(LinearSystem& sys, const std::vector<Scalar>& theta_0)
{
	//cout << "LOW JOINTLIMITS" << endl;
    if(!jointlimits_enable) return;
    
    Eigen::Matrix<Scalar, num_thetas, num_thetas> LHS = Eigen::Matrix<Scalar, num_thetas, num_thetas>::Identity(sys.lhs.rows(), sys.lhs.cols());
    Eigen::Matrix<Scalar, num_thetas, 1>  rhs = Eigen::Matrix<Scalar, num_thetas, 1>::Ones( sys.lhs.rows(),1 );
            
    
    bool show_constraints = jointlimits_show_constraints;    
   
    for(size_t i = 0; i < theta_0.size(); ++i) {

		float qmin = model->dofs[i].min;
		float qmax = model->dofs[i].max;

        if(theta_0[i] > qmax) {	
			//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
            rhs(i)=(qmax - theta_0[i]) - std::numeric_limits<Scalar>::epsilon();           
            LHS(i, i) = 1;
        }
        else if(theta_0[i] < qmin) {	
			//cout << "i = " << i << ", theta = " << theta_0[i] << endl;
            rhs(i) = (qmin - theta_0[i]) + std::numeric_limits<Scalar>::epsilon();
            LHS(i, i) = 1;
        }
        else {
            LHS(i, i) = 0;
            rhs(i) = 0;
        }
    }   
    
    ///--- Add constraints to the solve
    Scalar omega = jointlimits_weight;
    sys.lhs.noalias() += omega * LHS.transpose() * LHS;
    sys.rhs.noalias() += omega * LHS.transpose() * rhs;
    
    ///--- Check
    if(Energy::safety_check)
        Energy::has_nan(sys);
}
