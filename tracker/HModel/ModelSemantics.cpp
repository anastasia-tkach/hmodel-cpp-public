#include "ModelSemantics.h";
#include "Model.h"

ModelSemantics::ModelSemantics(Model * _model) : model(_model) {}

void ModelSemantics::setup_kinematic_chain() {
	size_t index = 0;
	size_t length_kinematic_chain = 25;
	model->kinematic_chain.resize(length_kinematic_chain);

	int root[CHAIN_MAX_LENGTH] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = root[i];
	index++;
	int pose[CHAIN_MAX_LENGTH] = { 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = pose[i];
	index++;
	int scale[CHAIN_MAX_LENGTH] = { 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = scale[i];
	index++;
	int Hand[CHAIN_MAX_LENGTH] = { 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = Hand[i];
	index++;
	int HandThumb1[CHAIN_MAX_LENGTH] = { 9, 10, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandThumb1[i];
	index++;
	int HandThumb2[CHAIN_MAX_LENGTH] = { 11, 9, 10, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandThumb2[i];
	index++;
	int HandThumb3[CHAIN_MAX_LENGTH] = { 12, 11, 9, 10, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandThumb3[i];
	index++;
	int HandThumb4[CHAIN_MAX_LENGTH] = { 12, 11, 9, 10, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandThumb4[i];
	index++;
	int HandPinky1[CHAIN_MAX_LENGTH] = { 25, 26, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandPinky1[i];
	index++;
	int HandPinky2[CHAIN_MAX_LENGTH] = { 27, 25, 26, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandPinky2[i];
	index++;
	int HandPinky3[CHAIN_MAX_LENGTH] = { 28, 27, 25, 26, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandPinky3[i];
	index++;
	int HandPinky4[CHAIN_MAX_LENGTH] = { 28, 27, 25, 26, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandPinky4[i];
	index++;
	int HandRing1[CHAIN_MAX_LENGTH] = { 21, 22, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandRing1[i];
	index++;
	int HandRing2[CHAIN_MAX_LENGTH] = { 23, 21, 22, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandRing2[i];
	index++;
	int HandRing3[CHAIN_MAX_LENGTH] = { 24, 23, 21, 22, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandRing3[i];
	index++;
	int HandRing4[CHAIN_MAX_LENGTH] = { 24, 23, 21, 22, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandRing4[i];
	index++;
	int HandMiddle1[CHAIN_MAX_LENGTH] = { 17, 18, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandMiddle1[i];
	index++;
	int HandMiddle2[CHAIN_MAX_LENGTH] = { 19, 17, 18, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandMiddle2[i];
	index++;
	int HandMiddle3[CHAIN_MAX_LENGTH] = { 20, 19, 17, 18, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandMiddle3[i];
	index++;
	int HandMiddle4[CHAIN_MAX_LENGTH] = { 20, 19, 17, 18, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandMiddle4[i];
	index++;
	int HandIndex1[CHAIN_MAX_LENGTH] = { 13, 14, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandIndex1[i];
	index++;
	int HandIndex2[CHAIN_MAX_LENGTH] = { 15, 13, 14, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandIndex2[i];
	index++;
	int HandIndex3[CHAIN_MAX_LENGTH] = { 16, 15, 13, 14, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandIndex3[i];
	index++;
	int HandIndex4[CHAIN_MAX_LENGTH] = { 16, 15, 13, 14, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = HandIndex4[i];
	index++;
	int Wrist[CHAIN_MAX_LENGTH] = { 7, 8, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1 };
	for (size_t i = 0; i < CHAIN_MAX_LENGTH; i++) model->kinematic_chain[index].data[i] = Wrist[i];
	index++;	
}

// Hmodel

void ModelSemantics::setup_outline() {

	if (model->model_type == 2 || model->model_type == 1) {
		std::vector<int> finger_block_indices;
		finger_block_indices.push_back(0);
		finger_block_indices.push_back(1);
		model->fingers_block_indices.push_back(finger_block_indices);
		finger_block_indices.clear();
		finger_block_indices.push_back(3);
		finger_block_indices.push_back(4);
		model->fingers_block_indices.push_back(finger_block_indices);
		finger_block_indices.clear();
		finger_block_indices.push_back(6);
		finger_block_indices.push_back(7);
		model->fingers_block_indices.push_back(finger_block_indices);
		finger_block_indices.clear();
		finger_block_indices.push_back(9);
		finger_block_indices.push_back(10);
		model->fingers_block_indices.push_back(finger_block_indices);
		finger_block_indices.clear();
		finger_block_indices.push_back(12);
		finger_block_indices.push_back(13);
		finger_block_indices.push_back(14);
		model->fingers_block_indices.push_back(finger_block_indices);

		model->fingers_base_centers.push_back(model->centers_name_to_id_map["pinky_bottom"]);
		model->fingers_base_centers.push_back(model->centers_name_to_id_map["ring_bottom"]);
		model->fingers_base_centers.push_back(model->centers_name_to_id_map["middle_bottom"]);
		model->fingers_base_centers.push_back(model->centers_name_to_id_map["index_bottom"]);
		model->fingers_base_centers.push_back(model->centers_name_to_id_map["thumb_bottom"]);

		model->crop_indices_thumb.push_back(glm::ivec2(model->centers_name_to_id_map["thumb_bottom"], model->centers_name_to_id_map["thumb_middle"]));
		model->limit_indices_thumb.push_back(glm::ivec2(model->centers_name_to_id_map["thumb_bottom"], model->centers_name_to_id_map["thumb_fold"]));

		model->crop_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["pinky_base"], model->centers_name_to_id_map["pinky_bottom"]));
		model->crop_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["ring_base"], model->centers_name_to_id_map["ring_bottom"]));
		model->crop_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["middle_base"], model->centers_name_to_id_map["middle_bottom"]));
		model->crop_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["index_base"], model->centers_name_to_id_map["index_bottom"]));

		model->limit_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["pinky_membrane"], model->centers_name_to_id_map["ring_membrane"]));
		model->limit_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["ring_membrane"], model->centers_name_to_id_map["middle_membrane"]));
		model->limit_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["middle_membrane"], model->centers_name_to_id_map["index_membrane"]));

		model->adjuct_block_indices_fingers.push_back(2);
		model->adjuct_block_indices_fingers.push_back(5);
		model->adjuct_block_indices_fingers.push_back(8);
		model->adjuct_block_indices_fingers.push_back(11);

		if (model->model_type == 1) {
			model->palm_block_indices.push_back(15);
			model->palm_block_indices.push_back(16);
			model->palm_block_indices.push_back(17);
			model->palm_block_indices.push_back(18);
			model->palm_block_indices.push_back(19);
			model->palm_block_indices.push_back(20);
			model->palm_block_indices.push_back(21);
			model->palm_block_indices.push_back(22);
			model->palm_block_indices.push_back(23);
			model->palm_block_indices.push_back(24);
			model->palm_block_indices.push_back(25);
			model->palm_block_indices.push_back(26);
			model->palm_block_indices.push_back(27);
			model->palm_block_indices.push_back(28);
			model->palm_block_indices.push_back(2);
			model->palm_block_indices.push_back(5);
			model->palm_block_indices.push_back(8);
			model->palm_block_indices.push_back(11);
		}
		if (model->model_type == 2) {
			model->fingers_block_indices[4][2] = 27;

			model->palm_block_indices.push_back(14);
			model->palm_block_indices.push_back(15);
			model->palm_block_indices.push_back(16);
			model->palm_block_indices.push_back(17);
			model->palm_block_indices.push_back(18);
			model->palm_block_indices.push_back(19);
			model->palm_block_indices.push_back(20);
			model->palm_block_indices.push_back(21);
			model->palm_block_indices.push_back(22);
			model->palm_block_indices.push_back(23);
			model->palm_block_indices.push_back(24);
			model->palm_block_indices.push_back(25);
			model->palm_block_indices.push_back(26);
			model->palm_block_indices.push_back(2);
			model->palm_block_indices.push_back(5);
			model->palm_block_indices.push_back(8);
			model->palm_block_indices.push_back(11);
			model->palm_block_indices.push_back(28);
			model->palm_block_indices.push_back(29);
		}
	}
	if (model->model_type == 0) {
		model->palm_block_indices.push_back(2);
		model->palm_block_indices.push_back(5);
		model->palm_block_indices.push_back(8);
		model->palm_block_indices.push_back(11);
		model->palm_block_indices.push_back(14);
		model->palm_block_indices.push_back(15);
		model->palm_block_indices.push_back(16);

		std::vector<int> finger_block_indices;
		finger_block_indices.push_back(0);
		finger_block_indices.push_back(1);
		model->fingers_block_indices.push_back(finger_block_indices);
		finger_block_indices.clear();
		finger_block_indices.push_back(3);
		finger_block_indices.push_back(4);
		model->fingers_block_indices.push_back(finger_block_indices);
		finger_block_indices.clear();
		finger_block_indices.push_back(6);
		finger_block_indices.push_back(7);
		model->fingers_block_indices.push_back(finger_block_indices);
		finger_block_indices.clear();
		finger_block_indices.push_back(9);
		finger_block_indices.push_back(10);
		model->fingers_block_indices.push_back(finger_block_indices);
		finger_block_indices.clear();
		finger_block_indices.push_back(12);
		finger_block_indices.push_back(13);
		model->fingers_block_indices.push_back(finger_block_indices);

		model->fingers_base_centers.push_back(2);
		model->fingers_base_centers.push_back(6);
		model->fingers_base_centers.push_back(10);
		model->fingers_base_centers.push_back(14);
		model->fingers_base_centers.push_back(18);
	}
}

void ModelSemantics::setup_centers_name_to_id_map() {

	model->centers_name_to_id_map = std::map<string, size_t>();
	if (model->model_type == 1) {
		model->centers_name_to_id_map["index_base"] = 15;
		model->centers_name_to_id_map["index_bottom"] = 14;
		model->centers_name_to_id_map["index_membrane"] = 32;
		model->centers_name_to_id_map["index_middle"] = 13;
		model->centers_name_to_id_map["index_top"] = 12;
		model->centers_name_to_id_map["middle_base"] = 11;
		model->centers_name_to_id_map["middle_bottom"] = 10;
		model->centers_name_to_id_map["middle_membrane"] = 31;
		model->centers_name_to_id_map["middle_middle"] = 9;
		model->centers_name_to_id_map["middle_top"] = 8;
		model->centers_name_to_id_map["palm_attachment"] = 26;
		model->centers_name_to_id_map["palm_back"] = 25;
		model->centers_name_to_id_map["palm_index"] = 23;
		model->centers_name_to_id_map["palm_left"] = 28;
		model->centers_name_to_id_map["palm_middle"] = 22;
		model->centers_name_to_id_map["palm_pinky"] = 20;
		model->centers_name_to_id_map["palm_right"] = 27;
		model->centers_name_to_id_map["palm_ring"] = 21;
		model->centers_name_to_id_map["palm_thumb"] = 24;
		model->centers_name_to_id_map["pinky_base"] = 3;
		model->centers_name_to_id_map["pinky_bottom"] = 2;
		model->centers_name_to_id_map["pinky_membrane"] = 29;
		model->centers_name_to_id_map["pinky_middle"] = 1;
		model->centers_name_to_id_map["pinky_top"] = 0;
		model->centers_name_to_id_map["ring_base"] = 7;
		model->centers_name_to_id_map["ring_bottom"] = 6;
		model->centers_name_to_id_map["ring_membrane"] = 30;
		model->centers_name_to_id_map["ring_middle"] = 5;
		model->centers_name_to_id_map["ring_top"] = 4;
		model->centers_name_to_id_map["thumb_additional"] = 34;
		model->centers_name_to_id_map["thumb_base"] = 19;
		model->centers_name_to_id_map["thumb_bottom"] = 18;
		model->centers_name_to_id_map["thumb_fold"] = 35;
		model->centers_name_to_id_map["thumb_membrane"] = 33;
		model->centers_name_to_id_map["thumb_middle"] = 17;
		model->centers_name_to_id_map["thumb_top"] = 16;
	}
	if (model->model_type == 2) {
		model->centers_name_to_id_map["index_base"] = 15;
		model->centers_name_to_id_map["index_bottom"] = 14;
		model->centers_name_to_id_map["index_membrane"] = 31;
		model->centers_name_to_id_map["index_middle"] = 13;
		model->centers_name_to_id_map["index_top"] = 12;
		model->centers_name_to_id_map["middle_base"] = 11;
		model->centers_name_to_id_map["middle_bottom"] = 10;
		model->centers_name_to_id_map["middle_membrane"] = 30;
		model->centers_name_to_id_map["middle_middle"] = 9;
		model->centers_name_to_id_map["middle_top"] = 8;
		model->centers_name_to_id_map["palm_back"] = 25;
		model->centers_name_to_id_map["palm_index"] = 23;
		model->centers_name_to_id_map["palm_left"] = 27;
		model->centers_name_to_id_map["palm_middle"] = 22;
		model->centers_name_to_id_map["palm_pinky"] = 20;
		model->centers_name_to_id_map["palm_right"] = 26;
		model->centers_name_to_id_map["palm_ring"] = 21;
		model->centers_name_to_id_map["palm_thumb"] = 24;
		model->centers_name_to_id_map["pinky_base"] = 3;
		model->centers_name_to_id_map["pinky_bottom"] = 2;
		model->centers_name_to_id_map["pinky_membrane"] = 28;
		model->centers_name_to_id_map["pinky_middle"] = 1;
		model->centers_name_to_id_map["pinky_top"] = 0;
		model->centers_name_to_id_map["ring_base"] = 7;
		model->centers_name_to_id_map["ring_bottom"] = 6;
		model->centers_name_to_id_map["ring_membrane"] = 29;
		model->centers_name_to_id_map["ring_middle"] = 5;
		model->centers_name_to_id_map["ring_top"] = 4;
		model->centers_name_to_id_map["thumb_additional"] = 32;
		model->centers_name_to_id_map["thumb_base"] = 19;
		model->centers_name_to_id_map["thumb_bottom"] = 18;
		model->centers_name_to_id_map["thumb_fold"] = 33;
		model->centers_name_to_id_map["thumb_middle"] = 17;
		model->centers_name_to_id_map["thumb_top"] = 16;

		model->centers_name_to_id_map["wrist_bottom_left"] = 36;
		model->centers_name_to_id_map["wrist_bottom_right"] = 37;
		model->centers_name_to_id_map["wrist_top_left"] = 34;
		model->centers_name_to_id_map["wrist_top_right"] = 35;
	}
}

void ModelSemantics::setup_phalanges_to_centers_map() {
	cout << "Initialize center_ids" << endl;
	{
		// 3 : Hand
		model->phalanges[0].center_id = model->centers_name_to_id_map["palm_back"];
		model->phalanges[0].attachments.push_back(model->centers_name_to_id_map["palm_back"]);
		model->phalanges[0].attachments.push_back(model->centers_name_to_id_map["palm_index"]);
		model->phalanges[0].attachments.push_back(model->centers_name_to_id_map["palm_left"]);
		model->phalanges[0].attachments.push_back(model->centers_name_to_id_map["palm_middle"]);
		model->phalanges[0].attachments.push_back(model->centers_name_to_id_map["palm_pinky"]);
		model->phalanges[0].attachments.push_back(model->centers_name_to_id_map["palm_right"]);
		model->phalanges[0].attachments.push_back(model->centers_name_to_id_map["palm_ring"]);
		model->phalanges[0].attachments.push_back(model->centers_name_to_id_map["palm_thumb"]);
		if (model->model_type == 1)
			model->phalanges[0].attachments.push_back(model->centers_name_to_id_map["thumb_fold"]);

		// 4 : HandThumb1
		model->phalanges[1].center_id = model->centers_name_to_id_map["thumb_base"];
		if (model->model_type == 1)
			model->phalanges[1].attachments.push_back(model->centers_name_to_id_map["thumb_membrane"]);
		if (model->model_type == 2)
			model->phalanges[1].attachments.push_back(model->centers_name_to_id_map["thumb_fold"]);

		// 5 : HandThumb2
		model->phalanges[2].center_id = model->centers_name_to_id_map["thumb_bottom"];

		// 6 : HandThumb3
		model->phalanges[3].center_id = model->centers_name_to_id_map["thumb_middle"];
		model->phalanges[3].attachments.push_back(model->centers_name_to_id_map["thumb_top"]);
		model->phalanges[3].attachments.push_back(model->centers_name_to_id_map["thumb_additional"]);

		// 8 : HandPinky1
		model->phalanges[4].center_id = model->centers_name_to_id_map["pinky_base"];
		model->phalanges[4].attachments.push_back(model->centers_name_to_id_map["pinky_membrane"]);

		// 9 : HandPinky2
		model->phalanges[5].center_id = model->centers_name_to_id_map["pinky_bottom"];

		// 10 : HandPinky3
		model->phalanges[6].center_id = model->centers_name_to_id_map["pinky_middle"];
		model->phalanges[6].attachments.push_back(model->centers_name_to_id_map["pinky_top"]);

		// 12 : HandRing1
		model->phalanges[7].center_id = model->centers_name_to_id_map["ring_base"];
		model->phalanges[7].attachments.push_back(model->centers_name_to_id_map["ring_membrane"]);

		// 13 : HandRing2
		model->phalanges[8].center_id = model->centers_name_to_id_map["ring_bottom"];

		// 14 : HandRing3
		model->phalanges[9].center_id = model->centers_name_to_id_map["ring_middle"];
		model->phalanges[9].attachments.push_back(model->centers_name_to_id_map["ring_top"]);

		// 16 : HandMiddle1
		model->phalanges[10].center_id = model->centers_name_to_id_map["middle_base"];
		model->phalanges[10].attachments.push_back(model->centers_name_to_id_map["middle_membrane"]);

		// 17 : HandMiddle2
		model->phalanges[11].center_id = model->centers_name_to_id_map["middle_bottom"];

		// 18 : HandMiddle3
		model->phalanges[12].center_id = model->centers_name_to_id_map["middle_middle"];
		model->phalanges[12].attachments.push_back(model->centers_name_to_id_map["middle_top"]);

		// 20 : HandIndex1
		model->phalanges[13].center_id = model->centers_name_to_id_map["index_base"];
		model->phalanges[13].attachments.push_back(model->centers_name_to_id_map["index_membrane"]);

		// 21 : HandIndex2
		model->phalanges[14].center_id = model->centers_name_to_id_map["index_bottom"];

		// 22 : HandIndex3
		model->phalanges[15].center_id = model->centers_name_to_id_map["index_middle"];
		model->phalanges[15].attachments.push_back(model->centers_name_to_id_map["index_top"]);

		model->phalanges[16].center_id = model->centers_name_to_id_map["palm_back"];
		model->phalanges[16].attachments.push_back(model->centers_name_to_id_map["wrist_bottom_left"]);
		model->phalanges[16].attachments.push_back(model->centers_name_to_id_map["wrist_bottom_right"]);
		model->phalanges[16].attachments.push_back(model->centers_name_to_id_map["wrist_top_left"]);
		model->phalanges[16].attachments.push_back(model->centers_name_to_id_map["wrist_top_right"]);
	}

}

void ModelSemantics::setup_jointid_to_centerid_map() {
	model->jointid_to_centerid_map[0] = -1; // root
	model->jointid_to_centerid_map[1] = -1; // pose
	model->jointid_to_centerid_map[2] = -1; // scale
	model->jointid_to_centerid_map[3] = model->centers_name_to_id_map["palm_back"]; // Hand
	model->jointid_to_centerid_map[4] = model->centers_name_to_id_map["thumb_base"]; // HandThumb1
	model->jointid_to_centerid_map[5] = model->centers_name_to_id_map["thumb_bottom"]; // HandThumb2
	model->jointid_to_centerid_map[6] = model->centers_name_to_id_map["thumb_middle"]; // HandThumb3
	model->jointid_to_centerid_map[7] = model->centers_name_to_id_map["thumb_top"]; // HandThumb4
	model->jointid_to_centerid_map[8] = model->centers_name_to_id_map["pinky_base"]; // HandPinky1
	model->jointid_to_centerid_map[9] = model->centers_name_to_id_map["pinky_bottom"]; // HandPinky2
	model->jointid_to_centerid_map[10] = model->centers_name_to_id_map["pinky_middle"]; // HandPinky3
	model->jointid_to_centerid_map[11] = model->centers_name_to_id_map["pinky_top"]; // HandPinky4
	model->jointid_to_centerid_map[12] = model->centers_name_to_id_map["ring_base"]; // HandRing1
	model->jointid_to_centerid_map[13] = model->centers_name_to_id_map["ring_bottom"]; // HandRing2
	model->jointid_to_centerid_map[14] = model->centers_name_to_id_map["ring_middle"]; // HandRing3
	model->jointid_to_centerid_map[15] = model->centers_name_to_id_map["ring_top"]; // HandRing4
	model->jointid_to_centerid_map[16] = model->centers_name_to_id_map["middle_base"]; // HandMiddle1
	model->jointid_to_centerid_map[17] = model->centers_name_to_id_map["middle_bottom"]; // HandMiddle2
	model->jointid_to_centerid_map[18] = model->centers_name_to_id_map["middle_middle"]; // HandMiddle3
	model->jointid_to_centerid_map[19] = model->centers_name_to_id_map["middle_top"]; // HandMiddle4
	model->jointid_to_centerid_map[20] = model->centers_name_to_id_map["index_base"]; // HandIndex1
	model->jointid_to_centerid_map[21] = model->centers_name_to_id_map["index_bottom"]; // HandIndex2
	model->jointid_to_centerid_map[22] = model->centers_name_to_id_map["index_middle"]; // HandIndex3
	model->jointid_to_centerid_map[23] = model->centers_name_to_id_map["index_top"]; // HandIndex4
}

void ModelSemantics::setup_blockid_to_jointid_map() {
	model->blockid_to_jointid_map = vector<int>(model->centers.size(), 3);

	if (model->model_type == 1) {
		// 10: HandPinky3
		model->blockid_to_jointid_map[0] = 10; // pinky_middle, pinky_top
		// 9: HandPinky2
		model->blockid_to_jointid_map[1] = 9; // pinky_bottom, pinky_middle
		// 8: HandPinky1
		model->blockid_to_jointid_map[2] = 8; // pinky_base, pinky_bottom
		// 14: HandRing3
		model->blockid_to_jointid_map[3] = 14; // ring_top, ring_middle
		// 13: HandRing2
		model->blockid_to_jointid_map[4] = 13; // ring_bottom, ring_middle
		// 12 : HandRing1
		model->blockid_to_jointid_map[5] = 12; // ring_bottom, ring_base
		// 18: HandMiddle3
		model->blockid_to_jointid_map[6] = 18; // middle_top, middle_middle
		// 17: HandMiddle2
		model->blockid_to_jointid_map[7] = 17; // middle_bottom, middle_middle
		// 16: HandMiddle1
		model->blockid_to_jointid_map[8] = 16; // middle_bottom, middle_base
		// 22: HandIndex3
		model->blockid_to_jointid_map[9] = 22; // index_middle, index_top
		// 21: HandIndex2
		model->blockid_to_jointid_map[10] = 21; // index_bottom, index_middle
		// 20: HandIndex1
		model->blockid_to_jointid_map[11] = 20; // index_base, index_bottom
		// 6: HandThumb3
		model->blockid_to_jointid_map[12] = 6; // thumb_top, thumb_additional
		// 6: HandThumb3
		model->blockid_to_jointid_map[13] = 6; // thumb_top, thumb_middle
		// 5: HandThumb2
		model->blockid_to_jointid_map[14] = 5; // thumb_bottom, thumb_middle
		// 4: HandThumb1
		model->blockid_to_jointid_map[22] = 4; // thumb_base, thumb_bottom, thumb_fold		
		// 3: Hand
		model->blockid_to_jointid_map[15] = 3; // palm_right, palm_ring, palm_pinky
		model->blockid_to_jointid_map[16] = 3; // palm_back, palm_right, palm_ring
		model->blockid_to_jointid_map[17] = 3; // palm_back, palm_ring, palm_middle
		model->blockid_to_jointid_map[18] = 3; // palm_back, palm_middle, palm_thumb
		model->blockid_to_jointid_map[19] = 3; // palm_back, thumb_base, palm_thumb
		model->blockid_to_jointid_map[20] = 3; // palm_middle, palm_index, palm_thumb
		model->blockid_to_jointid_map[21] = 3; // thumb_base, thumb_fold, palm_thumb
		model->blockid_to_jointid_map[23] = 3; // palm_pinky, ring_membrane, pinky_membrane
		model->blockid_to_jointid_map[24] = 3; // palm_ring, palm_pinky, ring_membrane
		model->blockid_to_jointid_map[25] = 3; // palm_ring, middle_membrane, ring_membrane
		model->blockid_to_jointid_map[26] = 3; // palm_ring, palm_middle, middle_membrane
		model->blockid_to_jointid_map[27] = 3; // palm_middle, palm_index, middle_membrane
		model->blockid_to_jointid_map[28] = 3; // palm_index, middle_membrane, index_membrane
	}
	if (model->model_type == 2) {
		// 10: HandPinky3
		model->blockid_to_jointid_map[0] = 10; // pinky_middle, pinky_top
		// 9: HandPinky2
		model->blockid_to_jointid_map[1] = 9; // pinky_bottom, pinky_middle
		// 8: HandPinky1
		model->blockid_to_jointid_map[2] = 8; // pinky_base, pinky_bottom
		// 14: HandRing3
		model->blockid_to_jointid_map[3] = 14; // ring_top, ring_middle
		// 13: HandRing2
		model->blockid_to_jointid_map[4] = 13; // ring_bottom, ring_middle
		// 12 : HandRing1
		model->blockid_to_jointid_map[5] = 12; // ring_bottom, ring_base
		// 18: HandMiddle3
		model->blockid_to_jointid_map[6] = 18; // middle_top, middle_middle
		// 17: HandMiddle2
		model->blockid_to_jointid_map[7] = 17; // middle_bottom, middle_middle
		// 16: HandMiddle1
		model->blockid_to_jointid_map[8] = 16; // middle_bottom, middle_base
		// 22: HandIndex3
		model->blockid_to_jointid_map[9] = 22; // index_middle, index_top
		// 21: HandIndex2
		model->blockid_to_jointid_map[10] = 21; // index_bottom, index_middle
		// 20: HandIndex1
		model->blockid_to_jointid_map[11] = 20; // index_base, index_bottom
		// 6: HandThumb3
		model->blockid_to_jointid_map[12] = 6; // thumb_top, thumb_middle
		// 5: HandThumb2
		model->blockid_to_jointid_map[13] = 5; // thumb_bottom, thumb_middle
		// 4: HandThumb1
		model->blockid_to_jointid_map[14] = 4; // thumb_base, thumb_bottom, thumb_fold
		// 4: HandThumb1
		model->blockid_to_jointid_map[26] = 4; // thumb_base, thumb_fold, palm_thumb	
		// 6: HandThumb3
		model->blockid_to_jointid_map[27] = 6; // thumb_top, thumb_additional
		// 3: Hand
		model->blockid_to_jointid_map[15] = 3; // palm_right, palm_ring, palm_pinky
		model->blockid_to_jointid_map[16] = 3; // palm_back, palm_right, palm_ring
		model->blockid_to_jointid_map[17] = 3; // palm_back, palm_middle, palm_ring
		model->blockid_to_jointid_map[18] = 3; // palm_back, palm_left, palm_middle
		model->blockid_to_jointid_map[19] = 3; // palm_left, palm_index, palm_middle
		model->blockid_to_jointid_map[20] = 3; // palm_pinky, ring_membrane, pinky_membrane
		model->blockid_to_jointid_map[21] = 3; // palm_ring, palm_pinky, ring_membrane
		model->blockid_to_jointid_map[22] = 3; // palm_ring, middle_membrane, ring_membrane
		model->blockid_to_jointid_map[23] = 3; // palm_middle, palm_ring, middle_membrane
		model->blockid_to_jointid_map[24] = 3; // palm_index, palm_middle, middle_membrane
		model->blockid_to_jointid_map[25] = 3; // palm_index, middle_membrane, index_membrane
		// 16: Wrist
		model->blockid_to_jointid_map[28] = 24; // wrist_bottom_left, wrist_top_left, wrist_top_right
		model->blockid_to_jointid_map[29] = 24; // wrist_bottom_right, wrist_bottom_left, wrist_top_right		
	}
}

void ModelSemantics::setup_topology() {
	// Initialize name_to_id_map
	setup_centers_name_to_id_map();

	// Initialize phalanges attachments
	setup_phalanges_to_centers_map();

	// Initialize joint_id to center_id map
	setup_jointid_to_centerid_map();
	setup_blockid_to_jointid_map();

	model->reindex();
	model->compute_tangent_points();
}

void ModelSemantics::setup_dofs() {

	// 0 - Translation
	model->dofs[0].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[0].type = TRANSLATION_AXIS;
	model->dofs[0].joint_id = 0;
	model->dofs[0].phalange_id = 17;
	model->dofs[0].min = -numeric_limits<float>::max();
	model->dofs[0].max = numeric_limits<float>::max();
	// 1 - Translation
	model->dofs[1].axis = Eigen::Vector3f(0, 1, 0);
	model->dofs[1].type = TRANSLATION_AXIS;
	model->dofs[1].joint_id = 1;
	model->dofs[1].phalange_id = 17;
	model->dofs[1].min = -numeric_limits<float>::max();
	model->dofs[1].max = numeric_limits<float>::max();
	// 2 - Translation
	model->dofs[2].axis = Eigen::Vector3f(0, 0, 1);
	model->dofs[2].type = TRANSLATION_AXIS;
	model->dofs[2].joint_id = 2;
	model->dofs[2].phalange_id = 17;
	model->dofs[2].min = -numeric_limits<float>::max();
	model->dofs[2].max = numeric_limits<float>::max();
	// 3 - Rotation
	model->dofs[3].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[3].type = ROTATION_AXIS;
	model->dofs[3].joint_id = 3;
	model->dofs[3].phalange_id = 17;
	model->dofs[3].min = -numeric_limits<float>::max();
	model->dofs[3].max = numeric_limits<float>::max();
	// 4 - Rotation
	model->dofs[4].axis = Eigen::Vector3f(0, 1, 0);
	model->dofs[4].type = ROTATION_AXIS;
	model->dofs[4].joint_id = 4;
	model->dofs[4].phalange_id = 17;
	model->dofs[4].min = -numeric_limits<float>::max();
	model->dofs[4].max = numeric_limits<float>::max();
	// 5 - Rotation
	model->dofs[5].axis = Eigen::Vector3f(0, 0, 1);
	model->dofs[5].type = ROTATION_AXIS;
	model->dofs[5].joint_id = 5;
	model->dofs[5].phalange_id = 17;
	model->dofs[5].min = -numeric_limits<float>::max();
	model->dofs[5].max = numeric_limits<float>::max();

	// 6 - Nothing
	model->dofs[6].axis = Eigen::Vector3f(0, 0, 1);
	model->dofs[6].type = ROTATION_AXIS;
	model->dofs[6].joint_id = 6;
	model->dofs[6].phalange_id = -1;
	model->dofs[5].min = -numeric_limits<float>::max();
	model->dofs[5].max = numeric_limits<float>::max();

	// 7 - Wrist abduction
	model->dofs[7].axis = Eigen::Vector3f(0, 0, 1);
	model->dofs[7].type = ROTATION_AXIS;
	model->dofs[7].min = -1;
	model->dofs[7].max = 0.2;
	model->dofs[7].joint_id = 7;
	model->dofs[7].phalange_id = 16;
	// 8 - Wrist flexion
	model->dofs[8].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[8].type = ROTATION_AXIS;
	model->dofs[8].min = -1;
	model->dofs[8].max = 1.6;
	model->dofs[8].joint_id = 8;
	model->dofs[8].phalange_id = 16;

	// 9 - HandThumb1 abduction
	model->dofs[9].axis = Eigen::Vector3f(0, 0, 1);
	model->dofs[9].type = ROTATION_AXIS;
	model->dofs[9].joint_id = 9;
	model->dofs[9].phalange_id = 1;
	// 10 - HandThumb1 flexion
	model->dofs[10].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[10].type = ROTATION_AXIS;
	model->dofs[10].joint_id = 10;
	model->dofs[10].phalange_id = 1;
	// 11 - HandThumb2
	model->dofs[11].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[11].type = ROTATION_AXIS;
	model->dofs[11].joint_id = 11;
	model->dofs[11].phalange_id = 2;
	// 12 - HandThumb3
	model->dofs[12].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[12].type = ROTATION_AXIS;
	model->dofs[12].joint_id = 12;
	model->dofs[12].phalange_id = 3;

	// 13 - HandIndex1 abduction
	model->dofs[13].axis = Eigen::Vector3f(0, 0, 1);
	model->dofs[13].type = ROTATION_AXIS;
	model->dofs[13].joint_id = 13;
	model->dofs[13].phalange_id = 13;
	// 14 - HandIndex1 flexion
	model->dofs[14].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[14].type = ROTATION_AXIS;
	model->dofs[14].joint_id = 14;
	model->dofs[14].phalange_id = 13;
	// 15 - HandIndex2
	model->dofs[15].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[15].type = ROTATION_AXIS;
	model->dofs[15].joint_id = 15;
	model->dofs[15].phalange_id = 14;
	// 16 - HandIndex3
	model->dofs[16].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[16].type = ROTATION_AXIS;
	model->dofs[16].joint_id = 16;
	model->dofs[16].phalange_id = 15;

	// 17 - HandMiddle1 abduction
	model->dofs[17].axis = Eigen::Vector3f(0, 0, 1);
	model->dofs[17].type = ROTATION_AXIS;
	model->dofs[17].joint_id = 17;
	model->dofs[17].phalange_id = 10;
	// 18 - HandMiddle1 flexion
	model->dofs[18].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[18].type = ROTATION_AXIS;
	model->dofs[18].joint_id = 18;
	model->dofs[18].phalange_id = 10;
	// 19 - HandMiddle2 flexion
	model->dofs[19].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[19].type = ROTATION_AXIS;
	model->dofs[19].joint_id = 19;
	model->dofs[19].phalange_id = 11;
	// 20 - HandMiddle3 flexion
	model->dofs[20].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[20].type = ROTATION_AXIS;
	model->dofs[20].joint_id = 20;
	model->dofs[20].phalange_id = 12;

	// 21 - HandRing1 abduction
	model->dofs[21].axis = Eigen::Vector3f(0, 0, 1);
	model->dofs[21].type = ROTATION_AXIS;
	model->dofs[21].joint_id = 21;
	model->dofs[21].phalange_id = 7;
	// 22 - HandRing1 flexion
	model->dofs[22].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[22].type = ROTATION_AXIS;
	model->dofs[22].joint_id = 22;
	model->dofs[22].phalange_id = 7;
	// 23 - HandRing2 flexion
	model->dofs[23].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[23].type = ROTATION_AXIS;
	model->dofs[23].joint_id = 23;
	model->dofs[23].phalange_id = 8;
	// 24 - HandRing3 flexion
	model->dofs[24].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[24].type = ROTATION_AXIS;
	model->dofs[24].joint_id = 24;
	model->dofs[24].phalange_id = 9;

	// 25 - HandPinky1 abduction
	model->dofs[25].axis = Eigen::Vector3f(0, 0, 1);
	model->dofs[25].type = ROTATION_AXIS;
	model->dofs[25].joint_id = 25;
	model->dofs[25].phalange_id = 4;
	// 26 - HandPinky1 flexion
	model->dofs[26].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[26].type = ROTATION_AXIS;
	model->dofs[26].joint_id = 26;
	model->dofs[26].phalange_id = 4;
	// 27 - HandPinky2 flexion
	model->dofs[27].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[27].type = ROTATION_AXIS;
	model->dofs[27].joint_id = 27;
	model->dofs[27].phalange_id = 5;
	// 28 - HandPinky3 flexion
	model->dofs[28].axis = Eigen::Vector3f(1, 0, 0);
	model->dofs[28].type = ROTATION_AXIS;
	model->dofs[28].joint_id = 28;
	model->dofs[28].phalange_id = 6;

	// Joint limits

	if (model->model_type == 2 && model->user_name == 0) {
		// thumb abduction
		model->dofs[9].min = -0.1;	model->dofs[9].max = 1.0;
		// thumb flexion 1
		model->dofs[10].min = -0.5;	model->dofs[10].max = 1.0;
		// thumb flexion 2
		model->dofs[11].min = -0.9;	model->dofs[11].max = 0.9;
		// thumb flexion 3
		model->dofs[12].min = -0.9;	model->dofs[12].max = 0.7;

		// index abductions
		model->dofs[13].min = -0.30;	model->dofs[13].max = 0.35;
		// index flexion 1
		model->dofs[14].min = -1.30;	model->dofs[14].max = 1.60;
		// index flexion 2
		model->dofs[15].min = -0.10;	model->dofs[15].max = 2.00;
		// index flexion 3
		model->dofs[16].min = -0.10;	model->dofs[16].max = 2.00;

		// middle abduction
		model->dofs[17].min = -0.30;	model->dofs[17].max = 0.40;
		// middle flexion 1
		model->dofs[18].min = -1.30;	model->dofs[18].max = 1.60;
		// middle flexion 2
		model->dofs[19].min = -0.10;	model->dofs[19].max = 2.00;
		// middle flexion 3
		model->dofs[20].min = -0.10;	model->dofs[20].max = 2.00;

		// ring abduction
		model->dofs[21].min = -0.10;	model->dofs[21].max = 0.30;
		// ring flexion 1
		model->dofs[22].min = -1.30;	model->dofs[22].max = 1.60;
		// ring flexion 2
		model->dofs[23].min = -0.10;	model->dofs[23].max = 2.00;
		// ring flexion 3
		model->dofs[24].min = -0.10;	model->dofs[24].max = 2.00;

		// pinky abduction
		model->dofs[25].min = -0.20;   model->dofs[25].max = 0.65;
		// pinky flexion 1
		model->dofs[26].min = -1.30;	model->dofs[26].max = 1.60;
		// pinky flexion 2
		model->dofs[27].min = -0.10;	model->dofs[27].max = 2.00;
		// pinky flexion 3
		model->dofs[28].min = -0.10;	model->dofs[28].max = 2.00;
	}

	if (model->model_type == 2 && model->user_name != 0) {
		// Wrist abduction
		model->dofs[7].min = -1.5; model->dofs[7].max = 0.5;
		// Wrist flexion
		model->dofs[8].min = -1.3;   model->dofs[8].max = 2.0;
		// thumb abduction
		model->dofs[9].min = -1.1;	model->dofs[9].max = -0.2;
		// thumb flexion 1
		model->dofs[10].min = -0.3;	model->dofs[10].max = 1.8;
		// thumb flexion 2
		model->dofs[11].min = -0.35;	model->dofs[11].max = 1.70;
		// thumb flexion 3
		model->dofs[12].min = -0.35;	model->dofs[12].max = 1.50;

		// index abductions
		model->dofs[13].min = -0.50;	model->dofs[13].max = 0.35;
		// index flexion 1
		model->dofs[14].min = -2.00;	model->dofs[14].max = 1.00;
		// index flexion 2
		model->dofs[15].min = -2.00;	model->dofs[15].max = 0.30;
		// index flexion 3
		model->dofs[16].min = -2.00;	model->dofs[16].max = 0.30;

		// middle abduction
		model->dofs[17].min = -0.50;	model->dofs[17].max = 0.25;
		// middle flexion 1
		model->dofs[18].min = -2.00;	model->dofs[18].max = 1.00;
		// middle flexion 2
		model->dofs[19].min = -2.00;	model->dofs[19].max = 0.30;
		// middle flexion 3
		model->dofs[20].min = -2.00;	model->dofs[20].max = 0.30;

		// ring abduction
		model->dofs[21].min = -0.20;	model->dofs[21].max = 0.30;
		// ring flexion 1
		model->dofs[22].min = -2.00;	model->dofs[22].max = 1.00;
		// ring flexion 2
		model->dofs[23].min = -2.00;	model->dofs[23].max = 0.30;
		// ring flexion 3
		model->dofs[24].min = -2.00;	model->dofs[24].max = 0.30;

		// pinky abduction
		model->dofs[25].min = -0.30;   model->dofs[25].max = 0.60;
		// pinky flexion 1
		model->dofs[26].min = -2.00;	model->dofs[26].max = 1.00;
		// pinky flexion 2
		model->dofs[27].min = -2.00;	model->dofs[27].max = 0.30;
		// pinky flexion 3
		model->dofs[28].min = -2.00;	model->dofs[28].max = 0.30;
	}

	/*for (size_t i = 0; i < model->dofs.size(); i++) {
		cout << endl << i << endl;
		cout << "min: " << model->dofs[i].min << endl;
		cout << "max: " << model->dofs[i].max << endl;
		cout << "axis = Eigen::Vector3f(" << model->dofs[i].axis[0] << ", " << model->dofs[i].axis[1] << ", " << model->dofs[i].axis[02] << ");" << endl;
		cout << "type = " << model->dofs[i].type << ";" << endl;
		cout << "joint_id = " << model->dofs[i].joint_id << ";" << endl;
		cout << "phalange_id = " << model->dofs[i].phalange_id << ";" << endl;
		}
		*/
}

void ModelSemantics::setup_phalanges() {

	model->phalanges[0].name = "Hand";
	model->phalanges_name_to_id_map["Hand"] = 0;
	model->phalanges[0].parent_id = 17;
	model->phalanges[0].length = 0;
	model->phalanges[0].radius1 = 0;
	model->phalanges[0].radius2 = 0;
	model->phalanges[0].segment_id = 3;
	model->phalanges[0].kinematic_chain = { 0, 1, 2, 3, 4, 5 };
	model->phalanges[0].children_ids = { 1, 4, 7, 10, 13 };

	model->phalanges[1].name = "HandThumb1";
	model->phalanges_name_to_id_map["HandThumb1"] = 1;
	model->phalanges[1].parent_id = 0;
	model->phalanges[1].length = length(model->centers[model->centers_name_to_id_map["thumb_bottom"]] - 
		model->centers[model->centers_name_to_id_map["thumb_base"]]);
	model->phalanges[1].radius1 = model->radii[model->centers_name_to_id_map["thumb_base"]];
	model->phalanges[1].radius2 = model->radii[model->centers_name_to_id_map["thumb_bottom"]];
	model->phalanges[1].segment_id = 4;
	model->phalanges[1].kinematic_chain = { 9, 10, 0, 1, 2, 3, 4, 5 };
	model->phalanges[1].children_ids = { 2 };

	model->phalanges[2].name = "HandThumb2";
	model->phalanges_name_to_id_map["HandThumb2"] = 2;
	model->phalanges[2].parent_id = 1;
	model->phalanges[2].length = length(model->centers[model->centers_name_to_id_map["thumb_middle"]] - 
		model->centers[model->centers_name_to_id_map["thumb_bottom"]]);
	model->phalanges[2].radius1 = model->radii[model->centers_name_to_id_map["thumb_bottom"]];
	model->phalanges[2].radius2 = model->radii[model->centers_name_to_id_map["thumb_middle"]];
	model->phalanges[2].segment_id = 5;
	model->phalanges[2].kinematic_chain = { 11, 9, 10, 0, 1, 2, 3, 4, 5 };
	model->phalanges[2].children_ids = { 3 };

	model->phalanges[3].name = "HandThumb3";
	model->phalanges_name_to_id_map["HandThumb3"] = 3;
	model->phalanges[3].parent_id = 2;
	model->phalanges[3].length = length(model->centers[model->centers_name_to_id_map["thumb_additional"]] -
		model->centers[model->centers_name_to_id_map["thumb_middle"]]);
	model->phalanges[3].radius1 = model->radii[model->centers_name_to_id_map["thumb_middle"]];
	model->phalanges[3].radius2 = model->radii[model->centers_name_to_id_map["thumb_top"]];
	model->phalanges[3].segment_id = 6;
	model->phalanges[3].kinematic_chain = { 12, 11, 9, 10, 0, 1, 2, 3, 4, 5 };
	model->phalanges[3].children_ids = {};

	model->phalanges[4].name = "HandPinky1";
	model->phalanges_name_to_id_map["HandPinky1"] = 4;
	model->phalanges[4].parent_id = 0;
	model->phalanges[4].length = length(model->centers[model->centers_name_to_id_map["pinky_bottom"]] -
		model->centers[model->centers_name_to_id_map["pinky_base"]]);
	model->phalanges[4].radius1 = model->radii[model->centers_name_to_id_map["pinky_base"]];
	model->phalanges[4].radius2 = model->radii[model->centers_name_to_id_map["pinky_bottom"]];
	model->phalanges[4].segment_id = 8;
	model->phalanges[4].kinematic_chain = { 25, 26, 0, 1, 2, 3, 4, 5 };
	model->phalanges[4].children_ids = { 5 };

	model->phalanges[5].name = "HandPinky2";
	model->phalanges_name_to_id_map["HandPinky2"] = 5;
	model->phalanges[5].parent_id = 4;
	model->phalanges[5].length = length(model->centers[model->centers_name_to_id_map["pinky_middle"]] -
		model->centers[model->centers_name_to_id_map["pinky_bottom"]]);
	model->phalanges[5].radius1 = model->radii[model->centers_name_to_id_map["pinky_bottom"]];
	model->phalanges[5].radius2 = model->radii[model->centers_name_to_id_map["pinky_middle"]];
	model->phalanges[5].segment_id = 9;
	model->phalanges[5].kinematic_chain = { 27, 25, 26, 0, 1, 2, 3, 4, 5 };
	model->phalanges[5].children_ids = { 6 };

	model->phalanges[6].name = "HandPinky3";
	model->phalanges_name_to_id_map["HandPinky3"] = 6;
	model->phalanges[6].parent_id = 5;
	model->phalanges[6].length = length(model->centers[model->centers_name_to_id_map["pinky_top"]] -
		model->centers[model->centers_name_to_id_map["pinky_middle"]]);
	model->phalanges[6].radius1 = model->radii[model->centers_name_to_id_map["pinky_middle"]];
	model->phalanges[6].radius2 = model->radii[model->centers_name_to_id_map["pinky_top"]];
	model->phalanges[6].segment_id = 10;
	model->phalanges[6].kinematic_chain = { 28, 27, 25, 26, 0, 1, 2, 3, 4, 5 };
	model->phalanges[6].children_ids = {};

	model->phalanges[7].name = "HandRing1";
	model->phalanges_name_to_id_map["HandRing1"] = 7;
	model->phalanges[7].parent_id = 0;
	model->phalanges[7].length = length(model->centers[model->centers_name_to_id_map["ring_bottom"]] -
		model->centers[model->centers_name_to_id_map["ring_base"]]);
	model->phalanges[7].radius1 = model->radii[model->centers_name_to_id_map["ring_base"]];
	model->phalanges[7].radius2 = model->radii[model->centers_name_to_id_map["ring_bottom"]];
	model->phalanges[7].segment_id = 12;
	model->phalanges[7].kinematic_chain = { 21, 22, 0, 1, 2, 3, 4, 5 };
	model->phalanges[7].children_ids = { 8 };

	model->phalanges[8].name = "HandRing2";
	model->phalanges_name_to_id_map["HandRing2"] = 8;
	model->phalanges[8].parent_id = 7;
	model->phalanges[8].length = length(model->centers[model->centers_name_to_id_map["ring_middle"]] -
		model->centers[model->centers_name_to_id_map["ring_bottom"]]);
	model->phalanges[8].radius1 = model->radii[model->centers_name_to_id_map["ring_bottom"]];
	model->phalanges[8].radius2 = model->radii[model->centers_name_to_id_map["ring_middle"]];
	model->phalanges[8].segment_id = 13;
	model->phalanges[8].kinematic_chain = { 23, 21, 22, 0, 1, 2, 3, 4, 5 };
	model->phalanges[8].children_ids = { 9 };

	model->phalanges[9].name = "HandRing3";
	model->phalanges_name_to_id_map["HandRing3"] = 9;
	model->phalanges[9].parent_id = 8;
	model->phalanges[9].length = length(model->centers[model->centers_name_to_id_map["ring_top"]] -
		model->centers[model->centers_name_to_id_map["ring_middle"]]);
	model->phalanges[9].radius1 = model->radii[model->centers_name_to_id_map["ring_middle"]];
	model->phalanges[9].radius2 = model->radii[model->centers_name_to_id_map["ring_top"]];
	model->phalanges[9].segment_id = 14;
	model->phalanges[9].kinematic_chain = { 24, 23, 21, 22, 0, 1, 2, 3, 4, 5 };
	model->phalanges[9].children_ids = {};

	model->phalanges[10].name = "HandMiddle1";
	model->phalanges_name_to_id_map["HandMiddle1"] = 10;
	model->phalanges[10].parent_id = 0;
	model->phalanges[10].length = length(model->centers[model->centers_name_to_id_map["middle_bottom"]] -
		model->centers[model->centers_name_to_id_map["middle_base"]]);
	model->phalanges[10].radius1 = model->radii[model->centers_name_to_id_map["middle_base"]];
	model->phalanges[10].radius2 = model->radii[model->centers_name_to_id_map["middle_bottom"]];
	model->phalanges[10].segment_id = 16;
	model->phalanges[10].kinematic_chain = { 17, 18, 0, 1, 2, 3, 4, 5 };
	model->phalanges[10].children_ids = { 11 };

	model->phalanges[11].name = "HandMiddle2";
	model->phalanges_name_to_id_map["HandMiddle2"] = 11;
	model->phalanges[11].parent_id = 10;
	model->phalanges[11].length = length(model->centers[model->centers_name_to_id_map["middle_middle"]] -
		model->centers[model->centers_name_to_id_map["middle_bottom"]]);
	model->phalanges[11].radius1 = model->radii[model->centers_name_to_id_map["middle_bottom"]];
	model->phalanges[11].radius2 = model->radii[model->centers_name_to_id_map["middle_middle"]];
	model->phalanges[11].segment_id = 17;
	model->phalanges[11].kinematic_chain = { 19, 17, 18, 0, 1, 2, 3, 4, 5 };
	model->phalanges[11].children_ids = { 12 };

	model->phalanges[12].name = "HandMiddle3";
	model->phalanges_name_to_id_map["HandMiddle3"] = 12;
	model->phalanges[12].parent_id = 11;
	model->phalanges[12].length = length(model->centers[model->centers_name_to_id_map["middle_top"]] -
		model->centers[model->centers_name_to_id_map["middle_middle"]]);
	model->phalanges[12].radius1 = model->radii[model->centers_name_to_id_map["middle_middle"]];
	model->phalanges[12].radius2 = model->radii[model->centers_name_to_id_map["middle_top"]];
	model->phalanges[12].segment_id = 18;
	model->phalanges[12].kinematic_chain = { 20, 19, 17, 18, 0, 1, 2, 3, 4, 5 };
	model->phalanges[12].children_ids = {};

	model->phalanges[13].name = "HandIndex1";
	model->phalanges_name_to_id_map["HandIndex1"] = 13;
	model->phalanges[13].parent_id = 0;
	model->phalanges[13].length = length(model->centers[model->centers_name_to_id_map["middle_bottom"]] -
		model->centers[model->centers_name_to_id_map["middle_base"]]);
	model->phalanges[13].radius1 = model->radii[model->centers_name_to_id_map["index_base"]];
	model->phalanges[13].radius2 = model->radii[model->centers_name_to_id_map["index_bottom"]];
	model->phalanges[13].segment_id = 20;
	model->phalanges[13].kinematic_chain = { 13, 14, 0, 1, 2, 3, 4, 5 };
	model->phalanges[13].children_ids = { 14 };

	model->phalanges[14].name = "HandIndex2";
	model->phalanges_name_to_id_map["HandIndex2"] = 14;
	model->phalanges[14].parent_id = 13;
	model->phalanges[14].length = length(model->centers[model->centers_name_to_id_map["index_middle"]] -
		model->centers[model->centers_name_to_id_map["index_bottom"]]);
	model->phalanges[14].radius1 = model->radii[model->centers_name_to_id_map["index_bottom"]];
	model->phalanges[14].radius2 = model->radii[model->centers_name_to_id_map["index_middle"]];
	model->phalanges[14].segment_id = 21;
	model->phalanges[14].kinematic_chain = { 15, 13, 14, 0, 1, 2, 3, 4, 5 };
	model->phalanges[14].children_ids = { 15 };

	model->phalanges[15].name = "HandIndex3";
	model->phalanges_name_to_id_map["HandIndex3"] = 15;
	model->phalanges[15].parent_id = 14;
	model->phalanges[15].length = length(model->centers[model->centers_name_to_id_map["index_top"]] -
		model->centers[model->centers_name_to_id_map["index_middle"]]);
	model->phalanges[15].radius1 = model->radii[model->centers_name_to_id_map["index_middle"]];
	model->phalanges[15].radius2 = model->radii[model->centers_name_to_id_map["index_top"]];
	model->phalanges[15].segment_id = 22;
	model->phalanges[15].kinematic_chain = { 16, 15, 13, 14, 0, 1, 2, 3, 4, 5 };
	model->phalanges[15].children_ids = {};

	model->phalanges[16].name = "Wrist";
	model->phalanges_name_to_id_map["Wrist"] = 16;
	model->phalanges[16].parent_id = 0;
	model->phalanges[16].length = 0;
	model->phalanges[16].radius1 = 0;
	model->phalanges[16].radius2 = 0;
	model->phalanges[16].segment_id = -1;
	model->phalanges[16].kinematic_chain = { 7, 8, 0, 1, 2, 3, 4, 5 };
	model->phalanges[16].children_ids = {};

	model->phalanges[17].name = "Position";
	model->phalanges_name_to_id_map["Position"] = 17;
	model->phalanges[17].parent_id = -1;
	model->phalanges[17].length = 0;
	model->phalanges[17].radius1 = 0;
	model->phalanges[17].radius2 = 0;
	model->phalanges[17].segment_id = 0;
	model->phalanges[17].kinematic_chain = {};
	model->phalanges[17].children_ids = { 0 };

	/*model->phalanges[18].name = "Scale";
	model->phalanges_name_to_id_map["Scale"] = 18;
	model->phalanges[18].parent_id = 17;
	model->phalanges[18].length = 0;
	model->phalanges[18].radius1 = 0;
	model->phalanges[18].radius2 = 0;
	model->phalanges[18].segment_id = -1;
	model->phalanges[18].kinematic_chain = {};
	model->phalanges[18].children_ids = { 0 };*/

	/*for (size_t i = 0; i < num_phalanges + 1; i++) {
		Phalange phalange = model->phalanges[i];
		//cout << "id = " << i << endl;
		cout << "model->phalanges[" << i << "].name = " << "\"" << phalange.name << "\";" << endl;
		cout << "model->phalanges_name_to_id_map[" << model->phalanges[i].name << "] = " << i << ";" << endl;
		cout << "model->phalanges[" << i << "].parent_id = " << phalange.parent_id << ";" << endl;
		cout << "model->phalanges[" << i << "].length = " << phalange.length << ";" << endl;
		cout << "model->phalanges[" << i << "].radius1 = " << phalange.radius1 << ";" << endl;
		cout << "model->phalanges[" << i << "].radius2 = " << phalange.radius2 << ";" << endl;
		cout << "model->phalanges[" << i << "].segment_id = " << phalange.segment_id << ";" << endl;

		cout << "model->phalanges[" << i << "].kinematic_chain = {";
		for (size_t j = 0; j < phalange.kinematic_chain.size(); j++)
			cout << phalange.kinematic_chain[j] << ", ";
		cout << "};" << endl;
		cout << "model->phalanges[" << i << "].children = {";
		for (size_t j = 0; j < phalange.children_ids.size(); j++)
			cout << phalange.children_ids[j] << ", ";
		cout << "};" << endl << endl;
	}*/
}

