#include "color_d.h"

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include "ext_math.h"

using namespace ext_math;
using namespace std;

namespace vision {

const int FieldColorDetector::pixelSpacing=16;	//nur jeden 16ten Pixel in x- und y-Richtung scannen
const int FieldColorDetector::minFieldArea=150; //min amount of pixels for initial green seed / carpet color detection
const int FieldColorDetector::colorBorder=8;	//exclude extrem color values (0+colorBorder) < color < (255-colorBorder)
const float FieldColorDetector::greenGain=2.0;	//gain for green threshold in maybeGreen function
const float FieldColorDetector::thetas[62]={-0.003928473492851304, 0.3267591297421786, 0.15024038619767252, -0.0026580701516830778, -0.06626938819648565,
											-0.0918234800891045, -0.007024391380169659, -0.19444191475988953, 0.10805971362794498, -0.013870596388515415,
											0.020367431009978596, 0.5006415354683456, 0.0523626396220556, 0.06796659398791673, 0.13091006086089688,
											-0.04015537555250251, 0.2534669682609816, 0.22259232118528924, 0.08825125863663277, -0.06163688650966539,
											0.18292785975365364, 0.18490873559957913, 0.1308039353774413, 0.15043700747876884, -0.030408070393040373,
											-0.18234162918227365, 0.30516577669883815, -0.060749446765493896, 0.473586856960429, 0.31872308251277015,
											-0.04073046667475529, -0.157079077089501, -0.4325757850385499, 0.04335112440158662, 0.05425490604442203,
											-0.21977115232887673, 0.06711078780969373, -0.08743680491075988, -0.07236939332440999, 0.03686565322899083,
											-0.0034490299130395608, -0.1224550945463936, 0.14004650713616937, 0.053306649062165465, 0.20536885019856824,
											0.010868449852810068, 0.14466029588975235, 0.08223414822611032, 0.09088654555978222, 0.12192354321809107,
											-0.15900537301205442, -0.07653902693746543, -0.13043295430645066, 0.10422567492674986, 0.12357034980655468,
											0.06425325379992515, 0.1547906519403598, -0.11945473558477798, -0.15195968390312445, -0.060779372474056875,
											-0.06570198431532723, 0.07563040487570107};

__global__ void makeHistCr(const uint8_t * const img, int w, int h, int width, int height, int *histCr, int seedSearchBorder, int pixelSpacing){
    int idx = blockIdx.x*blockDim.x*blockDim.y + (threadIdx.x + threadIdx.y * blockDim.x);
	
	if( idx >= w*h){
		return;
	}
	int x = seedSearchBorder + (idx - idx/w*w)  * pixelSpacing;
    int y = seedSearchBorder + (idx/w ) * pixelSpacing;
	int cr = img[((x + y * width) << 1) | 3];
	// atomic operation
	atomicAdd(&histCr[cr], 1);
}

__global__ void makeHistCb(const uint8_t * const img, int w, int h, int width, int height, int *histCb, int seedCr, int pixelSpacing){
    int idx = blockIdx.x*blockDim.x*blockDim.y + (threadIdx.x + threadIdx.y * blockDim.x);
	if( idx >= w*h){
		return;
	}
	int x = (idx - idx/w*w)  * pixelSpacing;
    int y = idx/w * pixelSpacing;
	int cr = img[((x + y * width) << 1) | 3];
    if(abs(cr-seedCr)<4){
        int cb=img[(((x + y * width) >> 1) << 2) + 1];
		atomicAdd(&histCb[cb], 1);
    }
}

__global__ void makeHistY(const uint8_t * const img, int w, int h, int width, int height, int *histY, int seedCr, int seedCb, int pixelSpacing){
    int idx = blockIdx.x*blockDim.x*blockDim.y + (threadIdx.x + threadIdx.y * blockDim.x);
	if( idx >= w*h){
		return;
	}
	int x = (idx - idx/w*w)  * pixelSpacing;
    int y = idx/w * pixelSpacing;
	int cr = img[((x + y * width) << 1) | 3];
    if(abs(cr-seedCr)<4){
		int cb=img[(((x + y * width) >> 1) << 2) + 1];
		if(abs(cb-seedCb)<8){
			int cy=img[(x + y * width) << 1];
			atomicAdd(&histY[cy], 1);
		}
    }
}

__global__ void sumGreen(const uint8_t * const img, int w, int h, int width, int height, int pixelSpacing, int seedCr, int seedCb, int seedY, float *data){
    int idx = blockIdx.x*blockDim.x*blockDim.y + (threadIdx.x + threadIdx.y * blockDim.x);
	if( idx >= w*h){
		return;
	}
	int x = (idx - idx/w*w)  * pixelSpacing;
	int y = idx/w * pixelSpacing;
	
	int cy=img[(x + y * width) << 1];
	int cb=img[(((x + y * width) >> 1) << 2) + 1];
	int cr=img[((x + y * width) << 1) | 3];
	atomicAdd(&data[0], cy);
	atomicAdd(&data[2], (cb-128)*(cb-128));
	atomicAdd(&data[3], (cr-128)*(cr-128));
	if(abs(cr-seedCr)<=2&&abs(cb-seedCb)<=2&&abs(cy-seedY)<=2){
		atomicAdd(&data[5], 1);
		if(abs(cr-seedCr)<=1&&abs(cb-seedCb)<=1&&abs(cy-seedY)<=1){
			atomicAdd(&data[4], 1);
		}
	}
}

__global__ void getVarY(const uint8_t * const img, int w, int h, int width, int height, int pixelSpacing, float *data){
    int idx = blockIdx.x*blockDim.x*blockDim.y + (threadIdx.x + threadIdx.y * blockDim.x);
	if( idx >= w*h){
		return;
	}
	int x = (idx - idx/w*w)  * pixelSpacing;
	int y = idx/w * pixelSpacing;
	
	int cy=img[(x + y * width) << 1];
	atomicAdd(&data[1], (cy-data[0])*(cy-data[0]));
}


FieldColorDetector::FieldColorDetector(int _width, int _height, int8_t *_lutCb, int8_t *_lutCr)
    : BaseDetector(_width, _height, _lutCb, _lutCr)
{
	greenCy=0;
	greenCr=0;
	greenCb=0;
	resetArrays();

}

FieldColorDetector::~FieldColorDetector(){
}

/**
 * detects the yCbCr color of the playing field in the image.
 * saves two histograms with rating values for different color combinations
 */
void FieldColorDetector::proceed(const uint8_t * const img) {
	resetArrays();
	searchInitialSeed(img);
	extractFeatures(img,features);
	setYCbCrCube(features);
}
/**
 * dynamic YCbCr-cube size estimation used for green classification
 * (offline training by CMA-ES optimization using 200 labeled color settings)
 */
void FieldColorDetector::setYCbCrCube(float* features){
	int idx=0;
	float minCy=25+50*thetas[idx++];
	float minCb=8+15*thetas[idx++];
	float minCr=8+15*thetas[idx++];
	for(int j=1;j<=NUM_FEATURES;j++){
        float feature=pow(features[j-1],1.3f+thetas[idx++]);
		minCy+=100*thetas[idx++]*feature;
		minCb+=30*thetas[idx++]*feature;
		minCr+=30*thetas[idx++]*feature;
	}
	float gY=3;
	float gC=2;
	if(minCy<1)minCy=1;
	if(minCy>80)minCy=80;
	if(minCb<1)minCb=1;
	if(minCb>30)minCb=30;
	if(minCr<1)minCr=1;
	if(minCr>30)minCr=30;
    float gy=1.5;
    float gc=1.5;
    this->minCy=(int)(greenCy-minCy*gy);
    this->minCb=(int)(greenCb-minCb*gc);
    this->minCr=(int)(greenCr-minCr*gc);
	this->minCy2=(int)(greenCy-minCy*greenGain);
	this->minCb2=(int)(greenCb-minCb*greenGain);
	this->minCr2=(int)(greenCr-minCr*greenGain);

	float maxCy=25+50*thetas[idx++];
	float maxCb=8+15*thetas[idx++];
	float maxCr=8+15*thetas[idx++];
	for(int j=1;j<=NUM_FEATURES;j++){
        float feature=pow(features[j-1],1.3f+thetas[idx++]);
		maxCy+=100*thetas[idx++]*feature;
		maxCb+=30*thetas[idx++]*feature;
		maxCr+=30*thetas[idx++]*feature;
	}
	if(maxCy<1)maxCy=1;
	if(maxCy>80)maxCy=80;
	if(maxCb<1)maxCb=1;
	if(maxCb>30)maxCb=30;
	if(maxCr<1)maxCr=1;
	if(maxCr>30)maxCr=30;
    this->maxCy=(int)(greenCy+maxCy*gy);
    this->maxCb=(int)(greenCb+maxCb*gc);
    this->maxCr=(int)(greenCr+maxCr*gc);
	this->maxCy2=(int)(greenCy+maxCy*greenGain);
	this->maxCb2=(int)(greenCb+maxCb*greenGain);
	this->maxCr2=(int)(greenCr+maxCr*greenGain);
}
/**
 * extraction of image features
 */
void FieldColorDetector::extractFeatures(const uint8_t * const img_d, float* features){

    //uint8_t *img_d;
    //cudaMalloc( (void**)&img_d, sizeof(uint8_t) * width * height * 2 );
    //cudaMemcpy( img_d, img, sizeof(uint8_t) * width * height * 2, cudaMemcpyHostToDevice ) ;
	int cnt=0;

	float meanY=0;
	float varY=0;
	float varCb=0;
	float varCr=0;
	float sumGreen1=0;
	float sumGreen2=0;

    int xi = (width-pixelSpacing / 2) / pixelSpacing;
    int yi = (height-pixelSpacing / 2) / pixelSpacing;
	
	int max = xi*yi;
	int block_num = max / 256 + 1;
	dim3 dim_(16, 16);
	//printf("%d %d max %d, num: %d\n", xi, yi, max, block_num);

	float *data_d, *data = new float[6]; 
	cudaMalloc( (void**)&data_d, sizeof(float) * 6 );

	sumGreen<<<block_num, dim_>>>(img_d, xi, yi, width, height, pixelSpacing, seedCr, seedCb, seedY, data_d);
	cudaMemcpy( data, data_d, sizeof(float) * 6, cudaMemcpyDeviceToHost ) ;
	
	meanY = data[0];
	varCb = data[2];
	varCr = data[3];
	sumGreen1 = data[4];
	sumGreen2 = data[5];
	cnt = max;

	//printf("total %d\n", cnt);
	sumGreen1/=cnt;
	sumGreen2/=cnt;
	varCb=sqrtf(varCb/cnt);
	varCr=sqrtf(varCr/cnt);
	meanY/=cnt;
	data[0] = meanY;

	cudaMemcpy( data_d, data, sizeof(float) * 6, cudaMemcpyHostToDevice ) ;
	getVarY<<<block_num, dim_>>>(img_d, xi, yi, width, height, pixelSpacing, data_d);
	cudaMemcpy( data, data_d, sizeof(float) * 6, cudaMemcpyDeviceToHost ) ;
	//printf("total %d, var Y %f %f \n", total, varY, data[1]);
	varY = data[1];
	varY=sqrtf(varY/cnt);
	features[0]=greenCy/256;
	features[1]=varY/32;
	features[2]=varCb/16;
	features[3]=varCr/16;
	features[4]=sumGreen1*50;
	features[5]=sumGreen2*25;
	features[6]=(sumGreen2-sumGreen1)*50;
	//cudaFree(img_d);
	cudaFree(data_d);
}


/**
 * dominant color search for green detection
 */
void FieldColorDetector::searchInitialSeed(const uint8_t * const img_d){
    //uint8_t *img_d;
    //cudaMalloc( (void**)&img_d, sizeof(uint8_t) * width * height * 2 );
    //cudaMemcpy( img_d, img, sizeof(uint8_t) * width * height * 2, cudaMemcpyHostToDevice ) ;

	//building histogram of all cr-channel
    int seedSearchBorder=width/16;
    int xi = (width-seedSearchBorder * 2) / pixelSpacing;
    int yi = (height -seedSearchBorder * 2) / pixelSpacing;
	
	int max = xi*yi;
	int block_num = max / 256 + 1;
	//printf("%d %d max %d, num: %d\n", xi, yi, max, block_num);

	dim3 dim_(16, 16);

	int *histCr_d;
	cudaMalloc( (void**)&histCr_d, 256*sizeof(int) );
    makeHistCr<<<block_num,dim_>>>(img_d, xi, yi, width, height, histCr_d, seedSearchBorder, pixelSpacing);
	cudaError_t  error_check ;
	error_check = cudaGetLastError();
	if( error_check != cudaSuccess ){
	    printf("%s\n" , cudaGetErrorString( error_check ) );
	}
	cudaMemcpy( histCr, histCr_d, 256*sizeof(int),cudaMemcpyDeviceToHost ) ;
	cudaFree(histCr_d);
	
    //finding initial cr-value (later used as a seed color)
	seedCr=clamp(colorBorder,getStableMin(histCr,minFieldArea),255-colorBorder);

	//build histogram of cb-channel for promising pixels
    xi = width / pixelSpacing;
	yi = (height) / pixelSpacing;
	
	max = xi*yi;
	block_num = max / 256 + 1;

    int *histCb_d;
    cudaMalloc( (void**)&histCb_d, 256*sizeof(int) );
    makeHistCb<<<block_num,dim_>>>(img_d, xi, yi, width, height, histCb_d, seedCr, pixelSpacing);
    cudaMemcpy( histCb, histCb_d, 256*sizeof(int),cudaMemcpyDeviceToHost ) ;
	cudaFree(histCb_d);

	//finding initial cb-value (later used as a seed color)
	seedCb=clamp(colorBorder,getPeak(histCb),255-colorBorder);

	//build histogram of y-channel for promising pixels
    xi = width / pixelSpacing;
	yi = height / pixelSpacing;
	
	max = xi*yi;
	block_num = max / 256 + 1;

    int *histY_d;
    cudaMalloc( (void**)&histY_d, 256*sizeof(int) );
    makeHistY<<<block_num,dim_>>>(img_d, xi, yi, width, height, histY_d, seedCr, seedCb, pixelSpacing);
    cudaMemcpy( histY, histY_d, 256*sizeof(int),cudaMemcpyDeviceToHost ) ;
	cudaFree(histY_d);

	//finding initial y-value (later used as a seed color)
	seedY=clamp(colorBorder,getPeak(histY),255-colorBorder);
	greenCy=seedY;
	greenCb=seedCb;
	greenCr=seedCr;
	//cudaFree(img_d);
}



void FieldColorDetector::resetArrays() {
    memset(histY,0,sizeof(histY));
    memset(histCb,0,sizeof(histCb));
    memset(histCr,0,sizeof(histCr));
}

/**
 * only in a histogram with 256 bins: get the index of the bin with the lowest value but with a stabilization criteria ('thres').
 */

int FieldColorDetector::getStableMin(const int* const hist, int thres) {
	int sum=0;
	for(int i=0;i<256;i++){
		sum+=hist[i];
		if(sum>thres){
			return i;
		}
	}
	return 0;
}

/**
 * only in a histogram with 256 bins: get the index of the bin with the highest value
 */
int FieldColorDetector::getPeak(const int* const hist) {
	int max=0;
	int maxIdx=0;
	for(int i=0;i<256;i++){
		if(hist[i]>max){
			max=hist[i];
			maxIdx=i;
		}
	}
	return maxIdx;
}

}  // namespace htwk
