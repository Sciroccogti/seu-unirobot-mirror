#include <cuda_runtime.h>

__global__ void yuyv2yuv_kernal(unsigned char *in, unsigned char *out, unsigned int w)
{
    int y=blockIdx.x;
    int x=threadIdx.x;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;
    extern __shared__ unsigned char tp[];
    tp[src_offset+0] = in[tmp*2+src_offset+0];
    tp[src_offset+1] = in[tmp*2+src_offset+1];
    __syncthreads();
    out[tmp*3+dst_offset+0] = tp[src_offset+0];
    out[tmp*3+dst_offset+1] = tp[src_offset+(int)pow(-1, x&1)];
    out[tmp*3+dst_offset+2] = tp[src_offset+2+(int)pow(-1, x&1)];

    //out[tmp*3+dst_offset+0] = in[tmp*2+src_offset+0];
    //out[tmp*3+dst_offset+1] = in[tmp*2+src_offset+(int)pow(-1, x&1)];
    //out[tmp*3+dst_offset+2] = in[tmp*2+src_offset+2+(int)pow(-1, x&1)];
    //out[tmp*3+dst_offset+3] = in[tmp*2+src_offset+2];
    //out[tmp*3+dst_offset+4] = in[tmp*2+src_offset+1];
    //out[tmp*3+dst_offset+5] = in[tmp*2+src_offset+3];
}

void cudaYUYV2YUV(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h)
{
    yuyv2yuv_kernal<<<h, w, w*2>>>(in,out,w);
}