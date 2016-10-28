#pragma once
#include "cudax/kernel.h"

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

class ComputeJacobianRow {
protected:
    J_row* J_raw; ///< array of rows 
    float* e_raw; ///< right hand side (constraints)    
    int2* cnstr_indexes; ///< raw
    CustomJointInfo* jointinfos; ///< raw
    ChainElement* chains; ///< raw
public:
    ComputeJacobianRow(J_row* J_raw, float* e_raw){
        assert(J_raw!=NULL);
        assert(e_raw!=NULL);        
        this->J_raw = J_raw;
        this->e_raw = e_raw;
        this->cnstr_indexes = ::pixel_indexer->cnstr_indexes;
        this->jointinfos = ::kinematic->jointinfos;
        this->chains = ::kinematic->chains;
    }

protected:    
    __device__
    glm::mat3x2 projection_jacobian(const glm::vec3& pos){
        // printf("focal length: %f", focal_length);
        glm::mat3x2 M(0); ///< remember column major!
        M[0][0] = focal_length_x / pos[2];
        M[1][1] = focal_length_y / pos[2];
        M[2][0] = -pos[0] * focal_length_x / ( pos[2]*pos[2] );
        M[2][1] = -pos[1] * focal_length_y / ( pos[2]*pos[2] );
        return M;
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
