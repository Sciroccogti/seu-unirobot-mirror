#include <cuda_runtime.h>

__device__ unsigned char rgb_bound(int v)
{
    return v>255?255:(v<0?0:v);
}

template<typename T>
__device__ T max(T v1, T v2)
{
    return v1>v2?v1:v2;
}

template<typename T>
__device__ T min(T v1, T v2)
{
    return v1<v2?v1:v2;
}

__global__ void yuyv2yuv_kernal(unsigned char *in, unsigned char *out, int w, int h)
{
    int x=blockIdx.x;
    int y=threadIdx.x;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;

    out[tmp*3+dst_offset+0] = in[tmp*2+src_offset+0];
    out[tmp*3+dst_offset+1] = in[tmp*2+src_offset+(int)pow(-1, x&1)];
    out[tmp*3+dst_offset+2] = in[tmp*2+src_offset+2+(int)pow(-1, x&1)];
}

__global__ void yuyv2bgr_kernal(unsigned char *in, unsigned char *out, int w, int h)
{
    int x=blockIdx.x;
    int y=threadIdx.x;
    int tmp = y*w;
    int src_offset = x*2;
    int dst_offset = x*3;

    unsigned char Y = in[tmp*2+src_offset+0];
    unsigned char U = in[tmp*2+src_offset+(int)pow(-1, x&1)];
    unsigned char V = in[tmp*2+src_offset+2+(int)pow(-1, x&1)];
    float r,g,b;

    r = (1.164 * (Y - 16)) + (2.018 * (V - 128));
    g = (1.164 * (Y - 16)) - (0.813 * (U - 128)) - (0.391 * (V - 128));
    b = (1.164 * (Y - 16)) + (1.596 * (U - 128));

    out[tmp*3+dst_offset+2] = rgb_bound(r);
    out[tmp*3+dst_offset+1] = rgb_bound(g);
    out[tmp*3+dst_offset+0] = rgb_bound(b);
}

__global__ void bgr2rgbfp(unsigned char *in, float *rgbfp, int w, int h)
{
    int x=blockIdx.x;
    int y=threadIdx.x;
    int offset = y*w*3+x*3;
    float rf, gf, bf;
    rf = in[offset+2]/255.0f;
    gf = in[offset+1]/255.0f;
    bf = in[offset+0]/255.0f;
    int planesize = w*h;
    int tmp = y*w+x;
    rgbfp[tmp] = rf;
    rgbfp[planesize+tmp] = gf;
    rgbfp[planesize*2+tmp] = bf;
}

__global__ void baygr2bgr_kernal(unsigned char *bayergr, unsigned char *bgr, int w, int h,
    float s, float rgain, float ggain, float bgain)
{
    int x = blockIdx.x;
    int y = threadIdx.x;
    unsigned char r,g,b;
    int rn, gn, bn;

    b = bayergr[(y+((y+1)&1))*w+x-(x&1)];
    g = bayergr[y*w+x-(x&1)+(y&1)];
    r = bayergr[(y-(y&1))*w+x+((x+1)&1)];
    
    unsigned char rgbMax = max(max(r,g), b);
    unsigned char rgbMin = min(min(r,g), b);
    float delta = (rgbMax-rgbMin)/255.0f;
    if(delta==0.0)
    {
        rn=r;
        bn=b;
        gn=g;
    }
    else
    {
        float value = (rgbMax+rgbMin)/255.0f;
        float L = value/2.0f;
        float S = (L<0.5)?delta/value:delta/(2.0f-value);
        float alpha;
        if(s>=0)
        {
            alpha = ((s+S)>=1)?S:1-s;
            alpha = 1.0f/alpha-1.0f;
            rn = rgb_bound(r+(r-L*255)*alpha);
            gn = rgb_bound(g+(g-L*255)*alpha);
            bn = rgb_bound(b+(b-L*255)*alpha);
        }
        else
        {
            alpha = s;
            rn = rgb_bound(L*255+(r-L*255)*(1+alpha));
            gn = rgb_bound(L*255+(g-L*255)*(1+alpha));
            bn = rgb_bound(L*255+(b-L*255)*(1+alpha));
        }
    }
    
    bgr[y*w*3+x*3+0] = rgb_bound(bn*bgain);
    bgr[y*w*3+x*3+1] = rgb_bound(gn*ggain);
    bgr[y*w*3+x*3+2] = rgb_bound(rn*rgain);
}

template<typename T>
__global__ void resize_packed_kernal(T *in, int iw, int ih, T *out, int ow, int oh)
{
    int x = blockIdx.x;
    int y = threadIdx.x;
    int offset_out = y*ow*3+x*3;
    float h_scale_rate = (float)ih/oh;
    float w_scale_rate = (float)iw/ow;
    float y_scale = h_scale_rate * y;
    float x_scale = w_scale_rate * x;
    int j = y_scale, i = x_scale;
    float u = y_scale-j, v = x_scale-i;
    int offset_in1 = j*iw*3;
    int offset_in2 = (j+1)*iw*3;
    if(j+1>=ih || i+1>=iw)
    {
        out[offset_out+0] = in[offset_in1+i*3];
        out[offset_out+1] = in[offset_in1+i*3+1];
        out[offset_out+2] = in[offset_in1+i*3+2];
    }
    else
    {
        unsigned char x1,x2,x3,x4;
        x1 = in[offset_in1+i*3];
        x2 = in[offset_in1+(i+1)*3];
        x3 = in[offset_in2+i*3];
        x4 = in[offset_in2+(i+1)*3];
        out[offset_out+0] = ((1-u)*(1-v)*x1+(1-u)*v*x2+u*(1-v)*x3+u*v*x4);
        x1 = in[offset_in1+i*3+1];
        x2 = in[offset_in1+(i+1)*3+1];
        x3 = in[offset_in2+i*3+1];
        x4 = in[offset_in2+(i+1)*3+1];
        out[offset_out+1] = ((1-u)*(1-v)*x1+(1-u)*v*x2+u*(1-v)*x3+u*v*x4);
        x1 = in[offset_in1+i*3+2];
        x2 = in[offset_in1+(i+1)*3+2];
        x3 = in[offset_in2+i*3+2];
        x4 = in[offset_in2+(i+1)*3+2];
        out[offset_out+2] = ((1-u)*(1-v)*x1+(1-u)*v*x2+u*(1-v)*x3+u*v*x4);
    }
}

template<typename T>
__global__ void resize_planar_kernal(T *in, int iw, int ih, T *out, int ow, int oh)
{
    int x = blockIdx.x;
    int y = threadIdx.x;
    int offset_out = y*ow+x;
    int planesize_out = ow*oh;
    float h_scale_rate = (float)ih/oh;
    float w_scale_rate = (float)iw/ow;
    float y_scale = h_scale_rate * y;
    float x_scale = w_scale_rate * x;
    int j = y_scale, i = x_scale;
    float u = y_scale-j, v = x_scale-i;
    int planesize_in = iw*ih;
    int offset_in1 = j*iw;
    int offset_in2 = (j+1)*iw;
    if(j+1>=ih || i+1>=iw)
    {
        out[offset_out] = in[offset_in1+i];
        out[planesize_out+offset_out] = in[planesize_in+offset_in1+i];
        out[planesize_out*2+offset_out] = in[planesize_in*2+offset_in1+i];
    }
    else
    {
        unsigned char x1,x2,x3,x4;
        x1 = in[offset_in1+i];
        x2 = in[offset_in1+i+1];
        x3 = in[offset_in2+i];
        x4 = in[offset_in2+i+1];
        out[offset_out] = ((1-u)*(1-v)*x1+(1-u)*v*x2+u*(1-v)*x3+u*v*x4);
        x1 = in[planesize_in+offset_in1+i];
        x2 = in[planesize_in+offset_in1+i+1];
        x3 = in[planesize_in+offset_in2+i];
        x4 = in[planesize_in+offset_in2+i+1];
        out[planesize_out+offset_out] = ((1-u)*(1-v)*x1+(1-u)*v*x2+u*(1-v)*x3+u*v*x4);
        x1 = in[planesize_in*2+offset_in1+i];
        x2 = in[planesize_in*2+offset_in1+i+1];
        x3 = in[planesize_in*2+offset_in2+i];
        x4 = in[planesize_in*2+offset_in2+i+1];
        out[planesize_out*2+offset_out] = ((1-u)*(1-v)*x1+(1-u)*v*x2+u*(1-v)*x3+u*v*x4);
    }
}

void cudaYUYV2YUV(unsigned char *in, unsigned char *out, int w, int h)
{
    yuyv2yuv_kernal<<<w, h>>>(in,out,w,h);
}

void cudaYUYV2BGR(unsigned char *in, unsigned char *out, int w, int h)
{
    yuyv2bgr_kernal<<<w, h>>>(in,out,w,h);
}

void cudaBayer2BGR(unsigned char *bayer, unsigned char *bgr, int w, int h, 
    float sat, float rgain, float ggain, float bgain)
{
    baygr2bgr_kernal<<<w,h>>>(bayer, bgr, w, h, sat, rgain, ggain, bgain);
}

void cudaBGR2RGBfp(unsigned char *bgr, float *rgbfp, int w, int h)
{
    bgr2rgbfp<<<w,h>>>(bgr, rgbfp, w, h);
}

void cudaResizePacked(float *in, int iw, int ih, float *sized, int ow, int oh)
{
    resize_packed_kernal<<<ow, oh>>>(in, iw, ih, sized, ow, oh);
}

void cudaResizePacked(unsigned char *in, int iw, int ih, unsigned char *sized, int ow, int oh)
{
    resize_packed_kernal<<<ow, oh>>>(in, iw, ih, sized, ow, oh);
}

void cudaResizePlanar(float *in, int iw, int ih, float *sized, int ow, int oh)
{
    resize_planar_kernal<<<ow, oh>>>(in, iw, ih, sized, ow, oh);
}

void cudaResizePlanar(unsigned char *in, int iw, int ih, unsigned char *sized, int ow, int oh)
{
    resize_planar_kernal<<<ow, oh>>>(in, iw, ih, sized, ow, oh);
}

