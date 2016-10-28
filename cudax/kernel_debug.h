#pragma once
#include "kernel.h"
#include <thrust/device_vector.h>
#include <thrust/copy.h>

using namespace cudax;

//-----------------------------------------------------------------//

struct ConstraintTypeFunctor{
    uchar* opencv_image_d_raw;
    int2* cnstr_indexes;
    float* e_raw;

    ConstraintTypeFunctor(uchar *opencv_image_d_raw, float* e_raw){
        this->opencv_image_d_raw = opencv_image_d_raw;
        this->cnstr_indexes = pixel_indexer->cnstr_indexes;
        this->e_raw = e_raw;
    }

};

struct SetZeroFunctor{
    J_row* J_raw;
    SetZeroFunctor(J_row* J_raw):J_raw(J_raw){}
    __device__
    void operator()(const int i){
        J_row &Ji = *(J_raw + i);
        for(int j=6; j<NUM_THETAS; ++j){
            /*((J_row)(*J)[i]).data[j] = 0;;*/
            Ji.data[j] = 0;
        }
    }
};

void kernel_simplify_jacobian(int n_total){
    thrust::counting_iterator<int> it(0);
    thrust::for_each(it, it+n_total, SetZeroFunctor(thrust::raw_pointer_cast(J->data())));
}

