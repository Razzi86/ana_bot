#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>  // For error logging
#include <limits.h> // For INT_MAX

// TODO: Do research online and find an official standard for using CUDA

// Define constants for filter sizes and other parameters
#define FILTER_SIZE 3  // Example filter size for a smoothing operation
#define BLOCK_SIZE 16  // Define block size for kernel execution

// Struct for representing a 3D point
struct PointXYZ {
    float x, y, z;
};

// CUDA kernel to filter depth images based on a maximum depth threshold
__global__ void filterDepth(float* img, int width, int height, float maxDepth) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    int index = idy * width + idx;

    if (idx < width && idy < height) {
        float depth = img[index];
        if (depth > maxDepth) {
            img[index] = NAN;  // Set out of range values to NaN
        }
    }
}

// CUDA kernel to perform a simple box smoothing on the depth image
__global__ void smoothDepth(float* input, float* output, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    int index = idy * width + idx;

    if (idx >= width || idy >= height) return;

    float sum = 0.0f;
    int count = 0;

    // Apply a simple mean filter
    for (int ky = -FILTER_SIZE / 2; ky <= FILTER_SIZE / 2; ++ky) {
        for (int kx = -FILTER_SIZE / 2; kx <= FILTER_SIZE / 2; ++kx) {
            int n_x = idx + kx;
            int n_y = idy + ky;
            if (n_x >= 0 && n_x < width && n_y >= 0 && n_y < height) {
                sum += input[n_y * width + n_x];
                count++;
            }
        }
    }

    output[index] = sum / count;
}

// Host function to launch the filter and smoothing kernels
extern "C" void processDepthImage(float* hostImage, int width, int height, float maxDepth) {
    float* devImage;
    size_t imageSize = width * height * sizeof(float);
    float* smoothedDepth;

    // Allocate memory on the GPU
    cudaMalloc((void**)&devImage, imageSize);
    cudaMalloc((void**)&smoothedDepth, imageSize);

    // Copy data from host to device
    cudaMemcpy(devImage, hostImage, imageSize, cudaMemcpyHostToDevice);

    // Setup the execution configuration
    dim3 dimBlock(BLOCK_SIZE, BLOCK_SIZE);
    dim3 dimGrid((width + BLOCK_SIZE - 1) / BLOCK_SIZE, (height + BLOCK_SIZE - 1) / BLOCK_SIZE);

    // Run the depth filtering kernel
    filterDepth<<<dimGrid, dimBlock>>>(devImage, width, height, maxDepth);
    // Run the smoothing kernel
    smoothDepth<<<dimGrid, dimBlock>>>(devImage, smoothedDepth, width, height);

    // Copy the processed data back to host
    cudaMemcpy(hostImage, smoothedDepth, imageSize, cudaMemcpyDeviceToHost);

    // Free GPU memory
    cudaFree(devImage);
    cudaFree(smoothedDepth);
}

__global__ void voxelGridDownsample(PointXYZ *input, PointXYZ *output, int numPoints, float voxelSize, int *minIndexGrid, int gridDimensions) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx >= numPoints) return;

    int gridX = floor(input[idx].x / voxelSize);
    int gridY = floor(input[idx].y / voxelSize);
    int gridZ = floor(input[idx].z / voxelSize);
    int gridIndex = gridX + gridY * gridDimensions + gridZ * gridDimensions * gridDimensions;

    atomicMin(&minIndexGrid[gridIndex], idx);
    __syncthreads();

    if (idx == minIndexGrid[gridIndex]) {
        output[gridIndex] = input[idx];
    }
}

extern "C" void processPointCloudVoxelGrid(PointXYZ *hostPoints, int numPoints, float voxelSize) {
    PointXYZ *devPoints, *devReducedPoints;
    int *minIndexGrid;
    int gridDimensions = ceil(10.0 / voxelSize);  // Define the grid dimensions based on the voxel size
    int gridSize = gridDimensions * gridDimensions * gridDimensions;

    // Allocate memory on GPU
    cudaMalloc(&devPoints, numPoints * sizeof(PointXYZ));
    cudaMalloc(&devReducedPoints, gridSize * sizeof(PointXYZ));
    cudaMalloc(&minIndexGrid, gridSize * sizeof(int));
    cudaMemset(minIndexGrid, INT_MAX, gridSize * sizeof(int));

    // Copy data from host to device
    cudaMemcpy(devPoints, hostPoints, numPoints * sizeof(PointXYZ), cudaMemcpyHostToDevice);

    // Configure and launch the kernel
    dim3 block(256);
    dim3 grid((numPoints + block.x - 1) / block.x);
    voxelGridDownsample<<<grid, block>>>(devPoints, devReducedPoints, numPoints, voxelSize, minIndexGrid, gridDimensions);

    // Copy processed data back to host
    cudaMemcpy(hostPoints, devReducedPoints, gridSize * sizeof(PointXYZ), cudaMemcpyDeviceToHost);

    // Free GPU memory
    cudaFree(devPoints);
    cudaFree(devReducedPoints);
    cudaFree(minIndexGrid);
}
