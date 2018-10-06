#include "image_process.hpp"

__global__ void yuyvPacked2yuvPlanar_kernal(unsigned char *in, float *out, unsigned int w, unsigned int h)
{
    int plansize=w*h;
    int y=blockIdx.x;
    int x=threadIdx.x;
    int tmp = y*w;
    int src_offset = x*4;
    
    out[tmp+x*2] = in[tmp*2+src_offset]/255.0f;
    out[plansize+tmp+x*2] = in[tmp*2+src_offset+1]/255.0f;
    out[2*plansize+tmp+x*2] = in[tmp*2+src_offset+3]/255.0f;
    out[tmp+x*2+1] = in[tmp*2+src_offset+2]/255.0f;
    out[plansize+tmp+x*2+1] = in[tmp*2+src_offset+1]/255.0f;
    out[2*plansize+tmp+x*2+1] = in[tmp*2+src_offset+3]/255.0f;
}

__global__ void rgbPacked2rgbPlanar_kernal(unsigned char *in, float *out, unsigned int w, unsigned int h)
{
    int plansize=w*h;
    int y=blockIdx.x;
    int x=threadIdx.x;
    int tmp = y*w;
    out[tmp+x] = in[tmp*3+x*3]/255.0f;
    out[plansize+tmp+x] = in[tmp*3+x*3+1]/255.0f;
    out[2*plansize+tmp+x] = in[tmp*3+x*3+2]/255.0f;
}

__global__ void bgrPacked2rgbPlanar_kernal(unsigned char *in, float *out, unsigned int w, unsigned int h)
{
    int plansize=w*h;
    int y=blockIdx.x;
    int x=threadIdx.x;
    int tmp = y*w;
    out[tmp+x] = in[tmp*3+x*3+2]/255.0f;
    out[plansize+tmp+x] = in[tmp*3+x*3+1]/255.0f;
    out[2*plansize+tmp+x] = in[tmp*3+x*3]/255.0f;
}

void cudaYUYVPacked2YUVPlanar(unsigned char *in, float *out, const unsigned int &w, const unsigned int &h)
{
    yuyvPacked2yuvPlanar_kernal<<<h, w/2>>>(in, out, w, h);
}

void cudaRGBPacked2RGBPlanar(unsigned char *in, float *out, const unsigned int &w, const unsigned int &h)
{
    if(w>1024) return;
    rgbPacked2rgbPlanar_kernal<<<h,w>>>(in, out, w, h);
}

void cudaBGRPacked2RGBPlanar(unsigned char *in, float *out, const unsigned int &w, const unsigned int &h)
{
    if(w>1024) return;
    bgrPacked2rgbPlanar_kernal<<<h,w>>>(in, out, w, h);
}
