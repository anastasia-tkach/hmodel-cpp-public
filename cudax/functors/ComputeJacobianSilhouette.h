#pragma once
#include "ComputeJacobianRow.h"

namespace cudax {

class ComputeJacobianSilhouette : public ComputeJacobianRow{
private:
    ///--- These are for the extra_push
    int* sensor_dtform_idxs;
	int * blockid_to_jointid_map;
    float weight;

	int * rendered_pixels;
	float * rendered_points;
	int * rendered_block_ids;
public:
	ComputeJacobianSilhouette(J_row* J_raw, float* e_raw) : ComputeJacobianRow(J_raw, e_raw) {
        this->sensor_dtform_idxs = thrust::raw_pointer_cast(::sensor_dtform_idxs->data());
		this->blockid_to_jointid_map = thrust::raw_pointer_cast(device_pointer_blockid_to_jointid_map->data());

        weight = settings->fit2D_weight;        
      
		this->rendered_pixels = thrust::raw_pointer_cast(_rendered_pixels->data());
		this->rendered_points = thrust::raw_pointer_cast(_rendered_points->data());
		this->rendered_block_ids = thrust::raw_pointer_cast(_rendered_block_ids->data());
    }
        
	__device__
		void assemble_linear_system(int constraint_index, int b, glm::vec2 p_diff, glm::vec3 p_rend_3D) {
		J_row* J_sub = J_raw + 2 * constraint_index;
		float* e_sub = e_raw + 2 * constraint_index;

		int joint_id;
		if (_htrack_device) joint_id = b;
		else joint_id = blockid_to_jointid_map[b];
	
		glm::mat3x2 J_proj = projection_jacobian(p_rend_3D);

		/*if (off.z == 52962) {
		p_rend_3D = glm::vec3(2.708164, 60.960911, 389.143860);
		J_proj = projection_jacobian(p_rend_3D);
		printf("silhouette %d\n", off.z);
		printf("joint_id = %d\n", joint_id);
		printf("p_rend = %f %f\n", p_rend[0], p_rend[1]);
		printf("p_sens = %f %f\n", p_sens[0], p_sens[1]);
		printf("p_diff = %f %f\n", p_diff[0], p_diff[1]);
		printf("p_rend_3D = %f %f %f\n", p_rend_3D[0], p_rend_3D[1], p_rend_3D[2]);
		printf("J_proj = %f %f %f %f %f %f\n", J_proj[0][1], J_proj[0][2], J_proj[1][0], J_proj[1][1], J_proj[2][0], J_proj[2][1]);
		}*/
		// skeleton_jacobian(joint_id, p_rend_3D, J_sub, projector, true);


		///--- Compute LHS
		for (int i_column = 0; i_column<CHAIN_MAX_LENGTH; i_column++) {
			int jointinfo_id = chains[joint_id].data[i_column];
			if (jointinfo_id == -1) break;
			const CustomJointInfo& jinfo = jointinfos[jointinfo_id];
			glm::vec3& axis = jointinfos[jointinfo_id].axis;

			switch (jinfo.type) {
			case 1:
			{
				glm::vec3 col = glm::vec3(jointinfos[jointinfo_id].mat * glm::vec4(axis, 1));
				glm::vec2 jcol = J_proj * col;
				(J_sub + 0)->data[jinfo.index] = weight * jcol.x;
				(J_sub + 1)->data[jinfo.index] = weight * jcol.y;

				/*if (off.z == 52962) {
				printf("\nindex = %d\n", jinfo.index);
				printf("axis = %f %f %f\n", axis[0], axis[1], axis[2]);
				printf("mat = ");
				for (size_t u = 0; u < 16; u++) printf("%f ", jointinfos[jointinfo_id].mat[u]); printf("\n");
				printf("col = %f %f %f\n", col[0], col[1], col[2]);
				printf("jcol = %f %f\n", jcol[0], jcol[1]);
				printf("weight = %f\n", weight);
				printf("(J_sub+0) = %f\n", (J_sub + 0)->data[jinfo.index]);
				printf("(J_sub+1) = %f\n", (J_sub + 1)->data[jinfo.index]);
				}*/

				break;
			}
			case 0: // ROT
			{
				glm::vec3 t(jointinfos[jointinfo_id].mat[3][0], jointinfos[jointinfo_id].mat[3][1], jointinfos[jointinfo_id].mat[3][2]);
				glm::vec3 a = glm::normalize(glm::vec3(jointinfos[jointinfo_id].mat * glm::vec4(axis, 1)) - t);
				glm::vec3 col = glm::cross(a, p_rend_3D - t);
				glm::vec2 jcol = J_proj * col;
				(J_sub + 0)->data[jinfo.index] = weight * jcol.x;
				(J_sub + 1)->data[jinfo.index] = weight * jcol.y;

				/*if (off.z == 52962) {
				printf("\nindex = %d\n", jinfo.index);
				printf("axis = %f %f %f\n", axis[0], axis[1], axis[2]);
				printf("mat = ");
				for (size_t u = 0; u < 16; u++) printf("%f ", jointinfos[jointinfo_id].mat[u]); printf("\n");
				printf("a = %f %f %f\n", a[0], a[1], a[2]);
				printf("col = %f %f %f\n", col[0], col[1], col[2]);
				printf("jcol = %f %f\n", jcol[0], jcol[1]);
				printf("weight = %f\n", weight);
				printf("(J_sub+0) = %f\n", (J_sub + 0)->data[jinfo.index]);
				printf("(J_sub+1) = %f\n", (J_sub + 1)->data[jinfo.index]);
				}*/

				break;
			}
			}
		}

		/// Fills RHS
		*(e_sub + 0) = weight * p_diff.x;
		*(e_sub + 1) = weight * p_diff.y;
	}

public:

	/*__device__
		void operator()(int index) {
		int offset_y = index / width;
		int offset_x = index - width * offset_y;
		offset_y = height - 1 - offset_y;
		int offset_z = offset_y * width + offset_x;

		int joint_id = (int)tex2D(color_tex, offset_x, offset_y);

		int constraint_index = cnstr_indexes[offset_z].y;

		// Fetch closest point on sensor data
		int closest_idx = sensor_dtform_idxs[offset_z];
		int row = closest_idx / width;
		int col = closest_idx - width*row;
		glm::vec2 p_rend(offset_x, offset_y);
		glm::vec2 p_sens(col, row);
		glm::vec2 p_diff = p_sens - p_rend;

		// Fetch rendered 3D point        
		glm::vec3 p_rend_3D;
		p_rend_3D[0] = tex2D(extra_tex, offset_x, offset_y).x;
		p_rend_3D[1] = tex2D(extra_tex, offset_x, offset_y).y;
		p_rend_3D[2] = tex2D(extra_tex, offset_x, offset_y).z;

		assemble_linear_system(constraint_index, joint_id, p_diff, p_rend_3D);
	} */

	__device__
		void operator()(int index) {

		glm::vec3 p_rend_3D;
		p_rend_3D[0] = rendered_points[3 * index];
		p_rend_3D[1] = rendered_points[3 * index + 1];
		p_rend_3D[2] = rendered_points[3 * index + 2];
		int linear_index = rendered_pixels[index];
		int block_id = rendered_block_ids[index];
		int constraint_index = index;

		int offset_y = linear_index / width;
		int offset_x = linear_index - width * offset_y;
		offset_y = height - 1 - offset_y;
		int offset_z = offset_y * width + offset_x;

		// Fetch closest point on sensor data
		int closest_idx = sensor_dtform_idxs[offset_z];
		int row = closest_idx / width;
		int col = closest_idx - width*row;
		glm::vec2 p_rend(offset_x, offset_y);
		glm::vec2 p_sens(col, row);
		glm::vec2 p_diff = p_sens - p_rend;

		/*if (index == 100) {
			printf("closest_idx =  %d\n", closest_idx);
			printf("p_rend =  %f %f\n", p_rend.x, p_rend.y);
			printf("p_sens =  %f %f\n", p_sens.x, p_sens.y);			
		}*/

		assemble_linear_system(constraint_index, block_id, p_diff, p_rend_3D);
	}
};


} 
