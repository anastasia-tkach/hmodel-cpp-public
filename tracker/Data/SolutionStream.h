#pragma once
#include "tracker/Types.h"
#include <vector>
#include <QString>
#include <fstream>
#include <stdio.h>
#include "tracker/Data/DataStream.h"

///--- This is only valid when we record a stream
class SolutionStream{
public:
    std::vector< Thetas > frames;
	std::vector<Eigen::Matrix<Scalar, num_joints * 3, 1>> joint_locations;
    bool _valid = false;
    
public:
    bool isValid(int fid = 0){
        return _valid; // || ( (fid <= _valid_id) && (fid >= 0) );
    }
  
    void reserve(int size){
        frames.reserve(size);
    }

    void resize(int num_frames){
        frames.resize(num_frames);
    }    

    void set(int frame_id, const std::vector<Scalar>& theta ){
        Eigen::Map<const Thetas> _theta(theta.data());
        frames[frame_id] = _theta;
    }

};
