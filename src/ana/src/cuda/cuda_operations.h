#ifndef CUDA_OPERATIONS_H
#define CUDA_OPERATIONS_H

struct PointXYZ {
    float x, y, z;
};

#ifdef __cplusplus
extern "C" {
#endif

void processPointCloudVoxelGrid(PointXYZ *hostPoints, int numPoints, float voxelSize);

#ifdef __cplusplus
}
#endif

#endif // CUDA_OPERATIONS_H
