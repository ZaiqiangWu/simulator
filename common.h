#pragma once

/*check error code of cudaMalloc and print out if needed*/
#define safe_cuda(CODE)\
 {\
  cudaError_t err = CODE;\
  if(err != cudaSuccess) {\
    std::cout<<"CUDA error:"<<cudaGetErrorString(err)<<std::endl;\
 }\
}