#ifndef CUDA_OPERATIONS_H
#define CUDA_OPERATIONS_H

// Define a structure for 3D points, which will be used in GPU-based operations
struct PointXYZ {
    float x, y, z;
};

// Ensure compatibility with C++ compilers
#ifdef __cplusplus
extern "C" {
#endif

// Declare a function(s) to process point clouds using a voxel grid downsampling approach
// This function(s) will be implemented in a corresponding .cu CUDA file
void processPointCloudVoxelGrid(PointXYZ *hostPoints, int numPoints, float voxelSize);
void processDepthImage(float* hostImage, int width, int height, float maxDepth);  // Added declaration

#ifdef __cplusplus
}
#endif

#endif // CUDA_OPERATIONS_H
