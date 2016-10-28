#pragma once

class Model;

class ModelSerializer {
	
	void serialize_centers();

	void serialize_radii();

	void serialize_blocks();

	void serialize_tangent_points();

	void serialize_outline();

	void serialize_blockid_to_jointid_map();

	void serialize_transformations();

public:
	Model * model;

	ModelSerializer(Model * _model);

	void serialize_model();
	
};
