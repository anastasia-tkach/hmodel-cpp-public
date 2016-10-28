#include "util/gl_wrapper.h" ///< for cuda_gl_interop
#include <cuda_gl_interop.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>
#include <thrust/sequence.h>

#include "cudax/kernel.h"
#include "cudax/CudaTimer.h"
#include "cudax/helper_cuda.h" ///< SDK error checking
#include "cudax/CublasHelper.h"
#include "cudax/CudaHelper.h"
#include "cudax/KinectCamera.h"
#include "cudax/kernel_init.h"
#include "cudax/kernel_upload.h"
#include "cudax/kernel_debug.h"
#include "cudax/PixelIndexer.h"

#include "cudax/functors/IsSilhouette.h"
#include "cudax/functors/ComputeJacobianSilhouette.h"
#include "cudax/functors/ComputeJacobianData.h"

#include <iostream>
#include <fstream>
#include <string>

using namespace cudax;

struct absolute_value : public thrust::unary_function <float, float > {
	__host__ __device__
		float operator()(float x) const {
		return (x>=0) ? x : -x;
	}
};

void kernel_bind()
{
    if(cudax::sensor_depth)   CHECK_CUDA(cudaBindTextureToArray(depth_tex, cudax::sensor_depth));
}

void kernel_unbind(){
    if(cudax::sensor_depth)   CHECK_CUDA(cudaUnbindTexture(depth_tex));
    cudax::sensor_depth=NULL;
}

void kernel(float* eigen_JtJ, float* eigen_JtF, float & push_error, float & pull_error, bool eval_metric, bool reweight, int id, int iter, 
	int num_sensor_points, int num_rendered_points) {    
    
    // CUDA_TIMED_BLOCK(timer,"indexing constraints")

	int n_pull, n_push;

	if (settings->fit2D_enable) {
		n_push = 2 * num_rendered_points;
		pixel_indexer->clear_counters_memory();
		pixel_indexer->assign_pull_constraints_indices(num_sensor_points);
		n_pull = 1 * num_sensor_points; // point to plane
	}
	if (!settings->fit2D_enable) {
		pixel_indexer->clear_counters_memory();			
		pixel_indexer->assign_pull_constraints_indices(num_sensor_points);
		n_pull = 1 * num_sensor_points; // point to plane
		n_push = 0;
	}

	int n_total = n_pull + n_push;	
      
    // CUDA_TIMED_BLOCK(timer,"memory resize + zero (J+e)")
    { 
        const J_row zeros = {};
		thrust::fill(J->begin(), J->begin() + n_total, zeros);
		thrust::fill(F->begin(), F->begin() + n_total, 0.0f);
        
        if (n_total==0) return;
    }
    J_row* J_push = thrust::raw_pointer_cast(J->data());
    J_row* J_pull = J_push + n_push;    
    
	float* F_push = thrust::raw_pointer_cast(F->data());
    float* F_pull = F_push + n_push;

	ComputeJacobianSilhouette functor_push(J_push, F_push);
	ComputeJacobianData functor_data_model(J_pull, F_pull, reweight);

	int factor = 6;
    if(store_corresps) {
		hmodel_correspondences = new thrust::device_vector<float>(n_pull * factor, -111);
		functor_data_model.store_data(thrust::raw_pointer_cast(hmodel_correspondences->data()));
    }
    
	thrust::sequence(push_indices->begin(), push_indices->begin() + num_rendered_points);

    //CUDA_TIMED_BLOCK(timer,"Assemble Jacobian")
	{
		if (settings->fit2D_enable) {
			//thrust::for_each(_rendered_indicator->begin(), _rendered_indicator->begin() + num_rendered_points, functor_push);
			thrust::for_each(push_indices->begin(), push_indices->begin() + num_rendered_points, functor_push);
		}
		if (settings->fit3D_enable) 		
			thrust::for_each(_sensor_indicator->begin(), _sensor_indicator->begin() + num_sensor_points, functor_data_model);				
	} 

    // CUDA_TIMED_BLOCK(timer, "Jt*J and Jt*e + CPU Transfer")
	{
		CublasHelper::outer_product_J(*J, *JtJ, n_total, NUM_THETAS);
		CublasHelper::vector_product_J(*J, *F, *JtF, n_total, NUM_THETAS);
		thrust::copy(JtF->begin(), JtF->end(), eigen_JtF);
		thrust::copy(JtJ->begin(), JtJ->end(), eigen_JtJ);
	}

	// Multiply with CPU
	if (_test) {
		thrust::host_vector<float> F_host(n_total);
		thrust::copy(F->begin(), F->begin() + n_total, F_host.begin());
		thrust::host_vector<J_row> J_host(n_total);
		thrust::copy(J->begin(), J->begin() + n_total, J_host.begin());

		for (size_t i = 0; i < NUM_THETAS; i++) {
			double result = 0;
			for (size_t k = 0; k < n_total; k++) {				
				J_row a = J_host[k];
				result = result + (double)a.data[i] * (double)F_host[k];				
			}
			eigen_JtF[i] = result;
		}	
		for (size_t i = 0; i < NUM_THETAS; i++) {
			for (size_t j = 0; j < NUM_THETAS; j++) {
				double result = 0;
				for (size_t k = 0; k < n_total; k++) {
					J_row a = J_host[k];
					result = result + (double)a.data[i] * (double)a.data[j];
				}
				eigen_JtJ[i * NUM_THETAS + j] = result;
			}
		}
    }

    /// Only need evaluate metric on the last iteration
    if (eval_metric) {
		thrust::device_vector<float> f_pull(n_pull);
		thrust::transform(F->begin() + n_push, F->begin() + n_push + n_pull, f_pull.begin(), absolute_value());
		pull_error = thrust::reduce(f_pull.begin(), f_pull.end());
		pull_error = pull_error / n_pull;
		//std::cout << pull_error << std::endl;

		thrust::device_vector<float> f_push(n_push);
		thrust::transform(F->begin(), F->begin() + n_push, f_push.begin(), absolute_value());
		push_error = thrust::reduce(f_push.begin(), f_push.end());
		push_error = push_error / n_push;
	}		
	
	//Write the correspondences	
	if (store_corresps) {
		std::ofstream output_file;
		std::string data_path = "...";
		thrust::host_vector<float> output(n_pull * factor);
		
		thrust::copy(hmodel_correspondences->begin(), hmodel_correspondences->begin() + n_pull * factor, output.begin());
		output_file.open(data_path + "corresp-" + std::to_string(id) + ".txt");
		for (size_t i = 0; i < n_pull * factor; i++) {
			output_file << output[i] << " ";
		}
		output_file.close();
		
	}	
    return;
}
