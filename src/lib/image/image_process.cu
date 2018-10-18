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
}

template <typename T>
__device__ T boundrgb(T v)
{
    if(v>255) return 255;
    if(v<0) return 0;
    return v;
}

__global__ void yuyv2bgr_kernal(unsigned char *in, unsigned char *out, unsigned int w)
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

    unsigned char Y = tp[src_offset+0];
    unsigned char U = tp[src_offset+(int)pow(-1, x&1)];
    unsigned char V = tp[src_offset+2+(int)pow(-1, x&1)];
    int r,g,b;

    r = (1.164 * (Y - 16)) + (2.018 * (V - 128));
    g = (1.164 * (Y - 16)) - (0.813 * (U - 128)) - (0.391 * (V - 128));
    b = (1.164 * (Y - 16)) + (1.596 * (U - 128));
    r = boundrgb<int>(r);
    g = boundrgb<int>(g);
    b = boundrgb<int>(b);

    out[tmp*3+dst_offset+2] = (unsigned char)(r);
    out[tmp*3+dst_offset+1] = (unsigned char)(g);
    out[tmp*3+dst_offset+0] = (unsigned char)(b);
}

__global__ void yuyv2rgbpf_kernal(unsigned char *in, float *out, unsigned int w, unsigned int h)
{
    int y=blockIdx.x;
    int x=threadIdx.x;
    int planesize = w*h;
    int tmp = y*w;
    int src_offset = x*2;
    extern __shared__ unsigned char tp[];
    tp[src_offset+0] = in[tmp*2+src_offset+0];
    tp[src_offset+1] = in[tmp*2+src_offset+1];
    __syncthreads();

    unsigned char Y = tp[src_offset+0];
    unsigned char U = tp[src_offset+(int)pow(-1, x&1)];
    unsigned char V = tp[src_offset+2+(int)pow(-1, x&1)];
    float r,g,b;

    r = (1.164 * (Y - 16)) + (2.018 * (V - 128));
    g = (1.164 * (Y - 16)) - (0.813 * (U - 128)) - (0.391 * (V - 128));
    b = (1.164 * (Y - 16)) + (1.596 * (U - 128));
    r = boundrgb<float>(r)/255.0f;
    g = boundrgb<float>(g)/255.0f;
    b = boundrgb<float>(b)/255.0f;

    out[tmp+x] = r;
    out[planesize+tmp+x] = g;
    out[2*planesize+tmp+x] = b;
}

__global__ void yuyv2all_kernal(unsigned char *in, unsigned char *yuv, unsigned char *bgr,
        float *rgbpf, unsigned int w, unsigned int h)
{
    int y=blockIdx.x;
    int x=threadIdx.x;
    int planesize = w*h;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;
    extern __shared__ unsigned char tp[];
    tp[src_offset+0] = in[tmp*2+src_offset+0];
    tp[src_offset+1] = in[tmp*2+src_offset+1];
    __syncthreads();

    unsigned char Y = tp[src_offset+0];
    unsigned char U = tp[src_offset+(int)pow(-1, x&1)];
    unsigned char V = tp[src_offset+2+(int)pow(-1, x&1)];
    yuv[tmp*3+dst_offset+0] = Y;
    yuv[tmp*3+dst_offset+1] = U;
    yuv[tmp*3+dst_offset+2] = V;

    float r,g,b;
    r = (1.164 * (Y - 16)) + (2.018 * (V - 128));
    g = (1.164 * (Y - 16)) - (0.813 * (U - 128)) - (0.391 * (V - 128));
    b = (1.164 * (Y - 16)) + (1.596 * (U - 128));
    r = boundrgb<float>(r);
    g = boundrgb<float>(g);
    b = boundrgb<float>(b);
    bgr[tmp*3+dst_offset+0] = (unsigned char)b;
    bgr[tmp*3+dst_offset+1] = (unsigned char)g;
    bgr[tmp*3+dst_offset+2] = (unsigned char)r;

    rgbpf[tmp+x] = r/255.0f;
    rgbpf[planesize+tmp+x] = g/255.0f;
    rgbpf[2*planesize+tmp+x] = b/255.0f;
}

__global__ void yuyv2dst_kernal(unsigned char *in, unsigned char *bgr, float *rgb, unsigned int w, unsigned int h)
{
    int y=blockIdx.x;
    int x=threadIdx.x;
    int planesize = w*h;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;
    extern __shared__ unsigned char tp[];
    tp[src_offset+0] = in[tmp*2+src_offset+0];
    tp[src_offset+1] = in[tmp*2+src_offset+1];
    __syncthreads();

    unsigned char Y = tp[src_offset+0];
    unsigned char U = tp[src_offset+(int)pow(-1, x&1)];
    unsigned char V = tp[src_offset+2+(int)pow(-1, x&1)];


    float r,g,b;
    r = (1.164 * (Y - 16)) + (2.018 * (V - 128));
    g = (1.164 * (Y - 16)) - (0.813 * (U - 128)) - (0.391 * (V - 128));
    b = (1.164 * (Y - 16)) + (1.596 * (U - 128));
    r = boundrgb<float>(r);
    g = boundrgb<float>(g);
    b = boundrgb<float>(b);
    bgr[tmp*3+dst_offset+0] = (unsigned char)b;
    bgr[tmp*3+dst_offset+1] = (unsigned char)g;
    bgr[tmp*3+dst_offset+2] = (unsigned char)r;

    rgb[tmp+x] = r/255.0f;
    rgb[planesize+tmp+x] = g/255.0f;
    rgb[2*planesize+tmp+x] = b/255.0f;
}

__global__ void resize_kernal(float *src, unsigned int iw, unsigned int ih, float *dst, unsigned int ow, unsigned oh)
{
    int y = blockIdx.x;
    int x = threadIdx.x;
    int planesizeo = ow*oh;
    int planesizei = iw*ih;
    int tmpo = y*ow;

    double fRows = oh / (float)ih;
    double fCols = ow / (float)iw;
    int pX = 0;
    int pY = 0;
    pX = (int)(x / fCols);
    pY = (int)(y / fRows);
    int tmpi = pY*iw;

    if (pY < ih && pY >= 0 && pX < iw && pX >= 0)
    {
        dst[tmpo+x] = src[tmpi+pX];
        dst[planesizeo+tmpo+x] = src[planesizei+tmpi+pX];
        dst[2*planesizeo+tmpo+x] = src[2*planesizei+tmpi+pX];
    }
}

void cudaYUYV2YUV(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h)
{
    yuyv2yuv_kernal<<<h, w, w*2>>>(in,out,w);
}

void cudaYUYV2BGR(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h)
{
    yuyv2bgr_kernal<<<h, w, w*2>>>(in,out,w);
}

void cudaYUYV2RGBPF(unsigned char *in, float *out, const unsigned int &w, const unsigned int &h)
{
    yuyv2rgbpf_kernal<<<h, w, w*2>>>(in,out,w,h);
}

void cudaYUYV2DST(unsigned char *in, unsigned char *bgr, float *rgb, const unsigned int &w, const unsigned int &h)
{
    yuyv2dst_kernal<<<h,w,w*2>>>(in, bgr, rgb, w, h);
}

void cudaResize(float *in, unsigned int iw, unsigned int ih, float *out, unsigned int ow, unsigned oh)
{
    resize_kernal<<<oh, ow>>>(in, iw, ih, out, ow, oh);
}
