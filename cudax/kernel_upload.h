#pragma once
#include "kernel.h"
#include "Kinematic.h"

using namespace cudax;

void kernel_upload_dtform_idxs(int* H_dtform_idxs){
    thrust::copy(H_dtform_idxs, H_dtform_idxs+H_width*H_height, sensor_dtform_idxs->begin());    
}

void kernel_upload_sensor_silhouette(uchar* H_silhouette_sensor)
{
   thrust::copy(H_silhouette_sensor, H_silhouette_sensor+H_width*H_height, silhouette_sensor->begin());
}

void kernel_upload_sensor_indicator(int * sensor_indicator, int num_sensor_points) {
	thrust::copy(sensor_indicator, sensor_indicator + num_sensor_points, _sensor_indicator->begin());
	cudaMemcpyToSymbol(_num_sensor_points, &num_sensor_points, sizeof(int));
}


void kernel_upload_rendered_indicator(int * rendered_pixels, float * rendered_points, int * rendered_block_ids, int num_rendered_points) {
	thrust::copy(rendered_pixels, rendered_pixels + num_rendered_points, _rendered_pixels->begin());
	thrust::copy(rendered_points, rendered_points + 3 * num_rendered_points, _rendered_points->begin());
	thrust::copy(rendered_block_ids, rendered_block_ids + num_rendered_points, _rendered_block_ids->begin());
	cudaMemcpyToSymbol(_num_rendered_points, &num_rendered_points, sizeof(int));
}

void kernel_upload_kinematic(const JointTransformations& H_jointinfos, const KinematicChain& H_kinchains)
{
    kinematic->D_jointinfos.resize(H_jointinfos.size());
    thrust::copy(H_jointinfos.begin(), H_jointinfos.end(), kinematic->D_jointinfos.begin()); 
    kinematic->D_chains.resize(H_kinchains.size());	
    thrust::copy(H_kinchains.begin(), H_kinchains.end(), kinematic->D_chains.begin());

    // printf("#jointinfos: %ld\n", kinematic->D_jointinfos.size());
    // printf("#kinematic: %ld\n", kinematic->D_chains.size());
    
    /// Fetch raw pointers that can be accessed in the kernels
    kinematic->jointinfos = thrust::raw_pointer_cast(&kinematic->D_jointinfos[0]);
    kinematic->chains = thrust::raw_pointer_cast(&kinematic->D_chains[0]);
}

void kernel_upload_model(int d, int num_centers, int num_blocks, int num_outlines, int num_tangent_fields, int num_outline_fields, 
	const float * host_pointer_centers, const float * host_pointer_radii, const int * host_pointer_blocks,
	const float * host_pointer_tangent_points, const float * host_pointer_outline, const int * host_pointer_blockid_to_jointid_map) {

	/*printf("D = %d\n", d);
	printf("num_centers = %d\n", num_centers);
	printf("num_blocks = %d\n", num_blocks);
	printf("num_outlines = %d\n", num_outlines);
	printf("num_tangent_fields = %d\n", num_tangent_fields);
	printf("num_outline_fields = %d\n", num_outline_fields);*/

	/*printf("\n CENTERS: \n");
	for (size_t i = 0; i < NUM_CENTERS; i++) {
		printf("%f, %f, %f\n", host_pointer_centers[d * i], host_pointer_centers[d * i + 1], host_pointer_centers[d * i + 2]);
	}
	printf("\n RADII: \n");
	for (size_t i = 0; i < NUM_CENTERS; i++) {
		printf("%f\n", host_pointer_radii[i]);
	}
	printf("\n BLOCKS: \n");
	for (size_t i = 0; i < num_blocks; i++) {
		printf("%d, %d, %d\n", host_pointer_blocks[d * i], host_pointer_blocks[d * i + 1], host_pointer_blocks[d * i + 2]);
	}*/

	cudaMemcpyToSymbol(NUM_OUTLINES, &num_outlines, sizeof(int));
	thrust::copy(host_pointer_centers, host_pointer_centers + d * num_centers, device_pointer_centers->begin());
	thrust::copy(host_pointer_radii, host_pointer_radii + num_centers, device_pointer_radii->begin());
	thrust::copy(host_pointer_blocks, host_pointer_blocks + d * num_blocks, device_pointer_blocks->begin());
	thrust::copy(host_pointer_tangent_points, host_pointer_tangent_points + d * num_tangent_fields * num_blocks, device_pointer_tangent_points->begin());
	thrust::copy(host_pointer_outline, host_pointer_outline + d * num_outline_fields * num_outlines, device_pointer_outline->begin());
	thrust::copy(host_pointer_blockid_to_jointid_map, host_pointer_blockid_to_jointid_map + num_centers, device_pointer_blockid_to_jointid_map->begin());
}

