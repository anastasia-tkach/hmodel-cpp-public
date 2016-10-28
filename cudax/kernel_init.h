#pragma once
#include "kernel.h"
#include "cudax/Kinematic.h"
#include "cudax/PixelIndexer.h"

//=============================================================================
/// Constructor
//=============================================================================
void kernel_init(energy::fitting::Settings* _settings, int H_width, int H_height, int thetas_size, float H_focal_length_x, float H_focal_length_y, const float* H_inv_proj_matrix,
	int d, int num_centers, int num_blocks, int max_num_outlines, int num_tangent_fields, int num_outline_fields, bool htrack, bool test, int model_type) {
    cudax::settings = _settings;

    using namespace cudax;
    bool is_init = false;
    if(is_init){ std::cout << "!!!ERROR initialized cuda kernel twice" << std::endl; exit(0); }
    is_init = true;
            
    t = new cudax::CudaTimer();
    t->restart("Kernel Initialization");
        std::cout << "Init cuda: " << H_width << " " << H_height << std::endl;

        ///--- allocate linear system
        J = new thrust::device_vector<cudax::J_row>();
        F = new thrust::device_vector<float>();		
         
        //J->reserve(upper_bound_num_constraints);
        //e->reserve(upper_bound_num_constraints);
		int upper_bound_num_sensor_points = 80000;
		J->resize(upper_bound_num_sensor_points);
		F->resize(upper_bound_num_sensor_points);

		_sensor_indicator = new thrust::device_vector<int>();
		_sensor_indicator->resize(upper_bound_num_sensor_points);

		int upper_bound_num_outlines = 5000;
		push_indices = new thrust::device_vector<int>(upper_bound_num_outlines);
		_rendered_pixels = new thrust::device_vector<int>(upper_bound_num_outlines);
		_rendered_points = new thrust::device_vector<float>(3 * upper_bound_num_outlines);
		_rendered_block_ids = new thrust::device_vector<int>(upper_bound_num_outlines);

        JtJ = new thrust::device_vector<float>(thetas_size*thetas_size);
        JtF = new thrust::device_vector<float>(thetas_size);

        kinematic = new Kinematic();

        silhouette_sensor = new thrust::device_vector<uchar>(H_width*H_height);

        ///--- copy  floats to GPU
        cudax::H_width = H_width;
        cudax::H_height = H_height;
        cudaMemcpyToSymbol(focal_length_x, &H_focal_length_x, sizeof(float));
        cudaMemcpyToSymbol(focal_length_y, &H_focal_length_y, sizeof(float));
        cudaMemcpyToSymbol(width, &H_width, sizeof(int));
        cudaMemcpyToSymbol(height, &H_height, sizeof(int));

        ///--- copy camera matrix
        camera_matrix = new KinectCamera(H_inv_proj_matrix);

        indexes_memory = new thrust::device_vector<int2>(H_width*H_height, make_int2(-1,-1));

        pixel_indexer = new PixelIndexer(*indexes_memory);

        sensor_dtform_idxs = new thrust::device_vector<int>(H_width*H_height, -1);

		///-- allocate memory for Hmodel data
		cudaMemcpyToSymbol(D, &d, sizeof(int));
		cudaMemcpyToSymbol(NUM_CENTERS, &num_centers, sizeof(int));
		cudaMemcpyToSymbol(NUM_BLOCKS, &num_blocks, sizeof(int));	
		cudaMemcpyToSymbol(NUM_TANGENT_FIELDS, &num_tangent_fields, sizeof(int));
		cudaMemcpyToSymbol(NUM_OUTLINE_FIELDS, &num_outline_fields, sizeof(int));
		cudaMemcpyToSymbol(_model_type, &model_type, sizeof(int));
		cudaMemcpyToSymbol(_htrack_device, &htrack, sizeof(bool));
		device_pointer_centers = new thrust::device_vector<float>(d * num_centers);
		device_pointer_radii = new thrust::device_vector<float>(num_centers);
		device_pointer_blocks = new thrust::device_vector<int>(d * num_blocks);
		device_pointer_tangent_points = new thrust::device_vector<float>(d * num_tangent_fields * num_blocks);
		device_pointer_outline = new thrust::device_vector<float>(d * num_outline_fields * max_num_outlines);
		device_pointer_blockid_to_jointid_map = new thrust::device_vector<int>(num_centers);
		cudax::_htrack = htrack;
		cudax::_test = test;

    t->display();
    // t->set_prefix(" + ");
    is_init = true;
}

//=============================================================================
/// Destructor
//=============================================================================
void kernel_cleanup(){
    std::cout << "kernel_cleanup()" << std::endl;
    using namespace cudax;    
    delete t;
    delete J;
    delete F;
    delete JtJ;
    delete JtF;

	delete kinematic;
	delete indexes_memory;

	delete silhouette_sensor;
	delete camera_matrix;	
	delete pixel_indexer;

	delete _sensor_indicator;
	delete push_indices;
	delete _rendered_pixels;
	delete _rendered_points;
	delete _rendered_block_ids;
    
    delete sensor_dtform_idxs;
	delete device_pointer_centers;
	delete device_pointer_radii;
	delete device_pointer_blocks;
	delete device_pointer_tangent_points;
	delete device_pointer_outline;
	delete device_pointer_blockid_to_jointid_map;
}

//=============================================================================
/// Checks
//=============================================================================
void kernel_memory_tests(){
    /// Make sure data alignment is not f'd when we pass data to cuda
    
    if( !(sizeof(glm::mat4x4) == (16*sizeof(float))) ){    
        printf("!!! Memory alignment error");
        exit(0);       
    }
    
    if( !(sizeof(cudax::J_row)==(sizeof(float)*NUM_THETAS)) ){
        printf("!!! Memory alignment error");
        exit(0);       
    }
    
    /// Make sure memory transfers are done correctly
    if(!(sizeof(cudax::J_row)==(sizeof(float)*NUM_THETAS))){
        printf("!!! Memory alignment error");
        exit(0);
    }
}

void kernel_delete() {

}
