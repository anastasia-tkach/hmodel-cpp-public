#pragma once

/// @note this needs to be placed outside of Fitting.h as these settings
///       are parsed by the CUDA portions of the code

namespace energy {
	namespace fitting {
		struct Settings {

			///--- E_2D
			bool  fit2D_enable = true;
			float fit2D_weight = 1;

			///--- E_3D
			bool  fit3D_enable = true;
			float fit3D_weight = 1.0f;
			bool  fit3D_point2plane = true;
			bool  fit3D_reweight = true;
			bool fit3D_reweight_rigid = false;
		};
	}
}

