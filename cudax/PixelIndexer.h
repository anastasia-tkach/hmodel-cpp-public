#pragma once
#include "kernel.h"
#include <thrust/device_vector.h>
#include "cudax/functors/IsSilhouette.h"


namespace cudax {

struct PixelIndexer {
    const int INVALID;
    thrust::device_vector<int> counters_memory; 
    int*  counters;       ///< raw device pointer!! 
    int2* cnstr_indexes;  ///< raw device pointer!!
    
public:  
    PixelIndexer(thrust::device_vector<int2>& indexes) : INVALID(-1), counters_memory(PixelType::SIZE, 0 /*init*/)    {
        this->counters = thrust::raw_pointer_cast(&counters_memory[0]);    
        this->cnstr_indexes  = thrust::raw_pointer_cast(&indexes[0]);
    }
    
	void clear_counters_memory() {
        thrust::fill(counters_memory.begin(), counters_memory.end(), 0);
    }
    
    int num_extra_pull_constraints(){ return counters_memory[PixelType::CONSTRAINT_EXTRA_PULL]; }
    int num_extra_push_constraints(){ return 2*counters_memory[PixelType::CONSTRAINT_EXTRA_PUSH]; }
    
    int num_silho_sensor(){ return counters_memory[PixelType::SENSOR_SILHOUETTE]; }
    int num_silho_render(){ return counters_memory[PixelType::RENDER_SILHOUETTE]; }
   
	void assign_pull_constraints_indices(int num_sensor_points) {
		PullFunctor pull_functor(counters, cnstr_indexes);
		thrust::for_each(_sensor_indicator->begin(), _sensor_indicator->begin() + num_sensor_points, pull_functor);
	}

	/*void assign_push_constraints_indices(int num_rendered_points) {
		PushFunctor push_functor(counters, cnstr_indexes);
		thrust::for_each(_rendered_indicator->begin(), _rendered_indicator->begin() + num_rendered_points, push_functor);
	}*/
    
    struct IsExtraPullConstraint{
        IsSensorSilhouette is_sensor_silhouette;
        IsExtraPullConstraint() : is_sensor_silhouette(*silhouette_sensor){}

        __device__
        bool operator()(const int4& off){
            return is_sensor_silhouette(off); 
        }
    };
    
    /*struct IsExtraPushConstraint{
        //IsRenderedSilhouette is_r_silho;
        IsSensorSilhouette is_s_silho;
        IsExtraPushConstraint() : is_s_silho(*silhouette_sensor){}

        __device__
        bool operator()(const int4& off){
            return is_r_silho(off) && !is_s_silho(off);
        }
    };*/

    struct Functor{
        //IsRenderedSilhouette    is_rendered_silhouette;
        IsSensorSilhouette      is_sensor_silhouette;
        IsExtraPullConstraint   is_extra_pull_constraint;
        //IsExtraPushConstraint   is_extra_push_constraint;
        
        int*  counters;       ///< global memory holding pixels in 
        int2* cnstr_indexes;
        
		Functor(int* counters, int2* cnstr_indexes) : is_sensor_silhouette(*silhouette_sensor)
        {
            this->counters = counters;   
            this->cnstr_indexes  = cnstr_indexes; 
        }
    };

	struct PullFunctor {
		int*  counters;      
		int2* cnstr_indexes;	

		PullFunctor(int* counters, int2* cnstr_indexes) {
			this->counters = counters;
			this->cnstr_indexes = cnstr_indexes;
		}
		__device__
			void operator()(int index) {
			int offset_y = index / width;
			int offset_x = index - width * offset_y;
			offset_y = height - 1 - offset_y;
			int offset_z = offset_y * width + offset_x;

			int constraint_index = atomicAdd(&counters[PixelType::CONSTRAINT_EXTRA_PULL], 1);

			cnstr_indexes[offset_z] = make_int2(PixelType::CONSTRAINT_EXTRA_PULL, constraint_index);
			
			//printf("offset_z = %d constraint_index = %d, indices_memory = %d \n", offset_z, constraint_index, cnstr_indexes[offset_z].y);

		}
	};

	struct PushFunctor {
		int*  counters;
		int2* cnstr_indexes;

		PushFunctor(int* counters, int2* cnstr_indexes) {
			this->counters = counters;
			this->cnstr_indexes = cnstr_indexes;
		}
		__device__
			void operator()(int index) {
			int offset_y = index / width;
			int offset_x = index - width * offset_y;
			offset_y = height - 1 - offset_y;
			int offset_z = offset_y * width + offset_x;

			int constraint_index = atomicAdd(&counters[PixelType::CONSTRAINT_EXTRA_PUSH], 1);

			cnstr_indexes[offset_z] = make_int2(PixelType::CONSTRAINT_EXTRA_PUSH, constraint_index);

			//printf("offset_z = %d constraint_index = %d, indices_memory = %d \n", offset_z, constraint_index, cnstr_indexes[offset_z].y);

		}
	};
};


} 
