#include "cudax/kernel.h"
#include "cudax/PixelIndexer.h"
#include "CorrespondencesFinder.h"

//== NAMESPACE ================================================================
namespace cudax {
	//=============================================================================

	struct ComputeJacobianData : public ComputeJacobianRow {
		const glm::mat3x3& iproj; ///< to compute cloud from depth
		CorrespondencesFinder correspondences_finder;

		bool debug_store;
		float * correspondences;

		float weight;
		bool point_to_plane;
		bool reweight;

		float * centers;
		float * radii;
		int * blocks;
		float * tangent_points;
		float * outline;
		int * blockid_to_jointid_map;

		float * _JtJ;
		float * _Jte;

	public:
		ComputeJacobianData(J_row* J_raw, float* e_raw, bool reweight) :
			ComputeJacobianRow(J_raw, e_raw),
			iproj(cudax::camera_matrix->D_inv_proj_matrix()) {
			point_to_plane = settings->fit3D_point2plane;
			weight = settings->fit3D_weight;
			this->reweight = reweight;
			debug_store = false;
			correspondences = NULL;

			this->centers = thrust::raw_pointer_cast(device_pointer_centers->data());
			this->radii = thrust::raw_pointer_cast(device_pointer_radii->data());
			this->blocks = thrust::raw_pointer_cast(device_pointer_blocks->data());
			this->tangent_points = thrust::raw_pointer_cast(device_pointer_tangent_points->data());
			this->outline = thrust::raw_pointer_cast(device_pointer_outline->data());
			this->blockid_to_jointid_map = thrust::raw_pointer_cast(device_pointer_blockid_to_jointid_map->data());
		}

		void store_data(float *debug_correspondences) {
			if (debug_correspondences != NULL) { debug_store = true; correspondences = debug_correspondences; }
			else { debug_store = false; correspondences = NULL; }
		}
		
		__device__
			void skeleton_jacobian(const int joint_id, const glm::vec3& pos, J_row* sub_J, glm::vec3 nrm = glm::vec3(0), bool project = false) {

			//float j_buffer[CHAIN_MAX_LENGTH];
			for (int i_column = 0; i_column < CHAIN_MAX_LENGTH; i_column++) {
				int jointinfo_id = chains[joint_id].data[i_column];
				if (jointinfo_id == -1) break;
				const CustomJointInfo& jinfo = jointinfos[jointinfo_id];
				glm::vec3& axis = jointinfos[jointinfo_id].axis;

				glm::vec3 col;
				switch (jinfo.type) {
				case 1: {
					col = glm::vec3(jointinfos[jointinfo_id].mat * glm::vec4(axis, 1));
					break;
				}
				case 0: {
					glm::vec3 t(jointinfos[jointinfo_id].mat[3][0], jointinfos[jointinfo_id].mat[3][1], jointinfos[jointinfo_id].mat[3][2]);
					glm::vec3 a = glm::normalize(glm::vec3(jointinfos[jointinfo_id].mat * glm::vec4(axis, 1)) - t);
					col = glm::cross(a, pos - t);
					break;
				}
				}
				
				if (point_to_plane || project) {
					sub_J->data[jinfo.index] = weight * glm::dot(col, nrm);					

					///					
					//j_buffer[jinfo.index] = weight * glm::dot(col, nrm);
					///
				}
				else {
					(sub_J + 0)->data[jinfo.index] = weight * col[0];
					(sub_J + 1)->data[jinfo.index] = weight * col[1];
					(sub_J + 2)->data[jinfo.index] = weight * col[2];
				}
			}

			/*for (int u = 0; u < CHAIN_MAX_LENGTH; u++) {
				int i_id = chains[joint_id].data[u];
				if (i_id == -1) break;
				int i = jointinfos[i_id].index;
				atomicAdd(_Jte + i, e * j_buffer[i]);				
				for (int v = 0; v < CHAIN_MAX_LENGTH; v++) {
					int j_id = chains[joint_id].data[v];
					if (j_id == -1) break;
					int j = jointinfos[j_id].index;
					int k = i * NUM_THETAS + j;
					atomicAdd(_JtJ + k, j_buffer[i] * j_buffer[j]);
				}
			}*/
		}

		__device__ void assemble_linear_system(int constraint_index, float4 p_sensor) {
			glm::vec3 p = glm::vec3(p_sensor.x, p_sensor.y, p_sensor.z);
			glm::vec3 q, s;
			glm::ivec3 index;
			int b;

			correspondences_finder.find(p, b, q, s, index);

			if (debug_store) {
				correspondences[6 * constraint_index + 0] = p.x;
				correspondences[6 * constraint_index + 1] = p.y;
				correspondences[6 * constraint_index + 2] = p.z;
				correspondences[6 * constraint_index + 3] = q.x;
				correspondences[6 * constraint_index + 4] = q.y;
				correspondences[6 * constraint_index + 5] = q.z;
			}

			if (length(p - q) < 1e-5) return;
			glm::vec3 n = (p - q) / length(p - q);
			if (isnan(n[0]) || isnan(n[1]) || isnan(n[2])) return;

			int joint_id = blockid_to_jointid_map[b];

			if (reweight) {
				float d = length(p - q);
				float w = rsqrt(d + 1e-3);
				if (d > 1e-3) weight *= w * 3.5f; // factor 3.5 compensates for residual magnitude change
			}
			///--- Access to a 3xN block of Jacobian
			J_row* J_sub = J_raw + constraint_index;
			float* e_sub = e_raw + constraint_index;
			
			*e_sub = weight * glm::dot(p - q, n);
			skeleton_jacobian(joint_id, p, J_sub, n);		
			
		}

		__device__
			void operator()(int index) {

			int offset_y = index / width;        
			int offset_x = index - width * offset_y;
			offset_y = height - 1 - offset_y;
			int offset_z = offset_y * width + offset_x;

			float depth = (float)tex2D(depth_tex, offset_x, height - 1 - offset_y).x;
			glm::vec3 wrld = iproj * glm::vec3(offset_x * depth, offset_y * depth, depth);
			float4 p_sensor = make_float4(wrld[0], wrld[1], wrld[2], 0);
			
			int constraint_index = cnstr_indexes[offset_z].y;
			assemble_linear_system(constraint_index, p_sensor);
	
			/*if (offset_x == 80 && offset_y == 150) {			
				printf("NEW, depth = %f, index = %d, off_x = %d, off_y = %d\n", depth, offset_z, offset_x, offset_y);
			}*/
		}

	};
}