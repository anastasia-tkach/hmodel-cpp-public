/// @warning NEVER include this on the C++ side, only externals
#pragma once

#ifndef __CUDACC__
    #error you cannot compile c++ of this, only cudax/externs.h can be included
#endif

///--- system
#include <iostream>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include "cudax/cuda_glm.h"
#include "cudax/externs.h" ///< only thing that is exposed to C++
#include "tracker/Energy/Fitting/Settings.h"

namespace cudax {

//=============================================================================
/// Forward declarations
//=============================================================================
class Kinematic;
class KinectCamera;
struct CudaTimer;
class PixelIndexer;

//=============================================================================
/// Settings for fitting energy
//=============================================================================
energy::fitting::Settings* settings;

//=============================================================================
/// Kernel constants
//=============================================================================
__constant__ float focal_length_x;
__constant__ float focal_length_y;
__constant__ int width;
__constant__ int height;
int H_width;
int H_height;

//=============================================================================
/// Pixel type
//=============================================================================
namespace PixelType{ 
enum TYPE{INVALID=-1,          
          CONSTRAINT_EXTRA_PULL=2,
          CONSTRAINT_EXTRA_PUSH=3,
          RENDER_SILHOUETTE=4, 
          SENSOR_SILHOUETTE=5,
          OVERLAP_SILHOUETTE=6,
          UNION_SILHOUETTE=7,
          SIZE=8}; }

///--- Texture accessors
#define SENSOR_DEPTH(off) tex2D(depth_tex, off.x, (height-1-off.y)).x //< Depth is from kinect sensor, sunny side up!

///--- Texture types
typedef texture<uchar, 2, cudaReadModeElementType> GrayscaleTexture2D;
typedef texture<float4, 2, cudaReadModeElementType> Float4Texture2D;
typedef texture<ushort1, 2, cudaReadModeElementType> DepthTexture2D; /// GL_R16UI

//=============================================================================
/// Global memory
//=============================================================================
///--- These are the arrays mapped to the textures below
cudaArray* sensor_depth = NULL;

///--- Textures containing the OpenGL input data (must be declared in global scope)
DepthTexture2D     depth_tex; ///< texture with depth from sensor
CudaTimer* t = NULL;

///--- Transferred from tracking context
thrust::device_vector<uchar>* silhouette_sensor = NULL;
thrust::device_vector<int>* sensor_dtform_idxs = NULL;
    
Jacobian* J = NULL; ///< semi-preallocated memory to store jacobian
thrust::device_vector<float>* F = NULL; ///< effectors (same # columns as J)
thrust::device_vector<float>* JtJ = NULL; ///< preallocated memory NUM_THETAS^2
thrust::device_vector<float>* JtF = NULL; ///< preallocated memory NUM_THETAS

uchar* opencv_image = NULL;

thrust::device_vector<int2>* indexes_memory = NULL; ///< pixel to constraint type + index
PixelIndexer* pixel_indexer = NULL;


KinectCamera* camera_matrix = NULL;
Kinematic* kinematic = NULL;
    
#define SEGMENT_VALUES 44
#define SEGMENT_JOINTS 17
thrust::device_vector<float>* htrack_correspondences = NULL;
thrust::device_vector<float>* hmodel_correspondences = NULL;
bool store_corresps = false;

///--- Hmodel data
__device__ int D;
__device__ int NUM_CENTERS;
__device__ int NUM_BLOCKS;
__device__ int NUM_OUTLINES;
__device__ int NUM_TANGENT_FIELDS;
__device__ int NUM_OUTLINE_FIELDS;
__device__ int _model_type;
__device__ bool _htrack_device;

__device__ int _num_sensor_points;
thrust::device_vector<int>* _sensor_indicator = NULL;
__device__ int _num_rendered_points;
thrust::device_vector<int>* _rendered_pixels = NULL;
thrust::device_vector<float>* _rendered_points = NULL;
thrust::device_vector<int>* _rendered_block_ids = NULL;

thrust::device_vector<int>* push_indices = NULL;

thrust::device_vector<float>* device_pointer_centers = NULL;
thrust::device_vector<float>* device_pointer_radii = NULL;
thrust::device_vector<int>* device_pointer_blocks = NULL;
thrust::device_vector<float>* device_pointer_tangent_points = NULL;
thrust::device_vector<float>* device_pointer_outline = NULL;
thrust::device_vector<int>* device_pointer_blockid_to_jointid_map = NULL;
bool _htrack;
bool _test;


} 





