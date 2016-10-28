#pragma once

class Model;
class Cylinders;
class Skeleton;

class ModelSemantics {

	Model * model;

public:

	ModelSemantics(Model * _model);

	void setup_kinematic_chain();

	// Hmodel

	void setup_outline();

	void setup_centers_name_to_id_map();

	void setup_phalanges_to_centers_map();

	void setup_jointid_to_centerid_map();

	void setup_blockid_to_jointid_map();

	void setup_topology();

	void setup_dofs();

	void setup_phalanges();

};
