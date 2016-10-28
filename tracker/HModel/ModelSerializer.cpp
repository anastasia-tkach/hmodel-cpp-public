#include "ModelSerializer.h"
#include "Model.h"

void ModelSerializer::serialize_centers() {
	if (!model->host_pointer_centers)
		model->host_pointer_centers = new float[d * model->centers.size()];
	for (size_t i = 0; i < model->centers.size(); i++) {
		for (size_t j = 0; j < d; j++) {
			model->host_pointer_centers[d * i + j] = model->centers[i][j];
		}
	}
}

void ModelSerializer::serialize_radii() {
	if (!model->host_pointer_radii)
		model->host_pointer_radii = new float[model->radii.size()];
	for (size_t i = 0; i < model->radii.size(); i++) {
		model->host_pointer_radii[i] = model->radii[i];
	}
}

void ModelSerializer::serialize_blocks() {
	if (!model->host_pointer_blocks)
		model->host_pointer_blocks = new int[d * model->blocks.size()];
	for (size_t i = 0; i < model->blocks.size(); i++) {
		for (size_t j = 0; j < d; j++) {
			model->host_pointer_blocks[d * i + j] = model->blocks[i][j];
		}
	}
}

void ModelSerializer::serialize_tangent_points() {
	if (!model->host_pointer_tangent_points)
		model->host_pointer_tangent_points = new float[d * model->num_tangent_fields * model->tangent_points.size()];
	for (size_t i = 0; i < model->tangent_points.size(); i++) {
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + j] = model->tangent_points[i].v1[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 3 + j] = model->tangent_points[i].v2[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 6 + j] = model->tangent_points[i].v3[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 9 + j] = model->tangent_points[i].n[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 12 + j] = model->tangent_points[i].u1[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 15 + j] = model->tangent_points[i].u2[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 18 + j] = model->tangent_points[i].u3[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 21 + j] = model->tangent_points[i].m[j];
	}
}

void ModelSerializer::serialize_outline() {
	if (!model->host_pointer_outline)
		model->host_pointer_outline = new float[d * model->num_outline_fields * model->max_num_outlines];
	for (size_t i = 0; i < model->outline_finder.outline3D.size(); i++) {
		for (size_t j = 0; j < d; j++)
			model->host_pointer_outline[d * model->num_outline_fields * i + j] = model->outline_finder.outline3D[i].start[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_outline[d * model->num_outline_fields * i + 3 + j] = model->outline_finder.outline3D[i].end[j];
		for (size_t j = 0; j < 2; j++)
			model->host_pointer_outline[d * model->num_outline_fields * i + 6 + j] = model->outline_finder.outline3D[i].indices[j];
		model->host_pointer_outline[d * model->num_outline_fields * i + 8] = model->outline_finder.outline3D[i].block;
	}
}

void ModelSerializer::serialize_blockid_to_jointid_map() {
	if (!model->host_pointer_blockid_to_jointid_map)
		model->host_pointer_blockid_to_jointid_map = new int[model->blockid_to_jointid_map.size()];
	for (size_t i = 0; i < model->blockid_to_jointid_map.size(); i++) {
		model->host_pointer_blockid_to_jointid_map[i] = model->blockid_to_jointid_map[i];
	}
}

void ModelSerializer::serialize_transformations() {
	model->transformations.resize(num_thetas);
	for (size_t i = 0; i < model->dofs.size(); i++) {
		Eigen::Map<Vector3>(model->transformations[i].axis) = model->dofs[i].axis;
		model->transformations[i].type = model->dofs[i].type;
		model->transformations[i].index = model->dofs[i].joint_id;
		if (model->dofs[i].phalange_id != -1) {
			switch (model->dofs[i].type) {
			case TRANSLATION_AXIS: {
				Matrix3 mat = Matrix3::Identity(3, 3);
				Eigen::Map<Matrix4>(model->transformations[i].mat) = Transform3f(mat).matrix();
				break;
			}
			case ROTATION_AXIS:
			default: {
				Matrix4 mat = model->phalanges[model->dofs[i].phalange_id].global;
				Eigen::Map<Matrix4>(model->transformations[i].mat) = Transform3f(mat).matrix();
				break;
			}
			}
		}
	}
	/*
	for (int i = 1; i < num_thetas; ++i) {
		CustomJointInfo custom_joint_info = model->transformations[i];
		cout << custom_joint_info << endl;
	}
	*/
}

ModelSerializer::ModelSerializer(Model * _model) : model(_model) {}

void ModelSerializer::serialize_model() {
	serialize_centers();
	serialize_radii();
	serialize_blocks();
	serialize_tangent_points();
	serialize_outline();
	serialize_blockid_to_jointid_map();
	serialize_transformations();
}

