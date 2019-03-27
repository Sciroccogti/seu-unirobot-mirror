#include "field_d.h"

#include <cmath>
#include <cstring>
#include <algorithm>

using namespace std;

namespace vision {

template <class ForwardIterator>
ForwardIterator get_max_sec_element ( ForwardIterator first, ForwardIterator last, ForwardIterator& largest, ForwardIterator& second )
{
  if (first==last) return last;
  largest = first;
  second = first;

  while (++first!=last)
    if (*largest<*first){
        second = largest;
        largest=first;
    }    // or: if (comp(*largest,*first)) for version (2)
    
  return second;
}


FieldDetector::FieldDetector(int width, int height, int8_t *lutCb, int8_t *lutCr)
    : BaseDetector(width, height, lutCb, lutCr)
{
    fieldBorderFull = new int[width];

    for (int x = 0; x < width; x++) {
        fieldBorderFull[x] = height - 2;
    }

}

FieldDetector::~FieldDetector() {
    delete [] fieldBorderFull;
}
/*
__global__ void checkPoints(const uint8_t * const img, int w, int h, int width, int offset, int lineSpacing, thrust::host_vector<point_2d> borderPoints, Scanline* lines){
    int idx = blockIdx.x*blockDim.x*blockDim.y + (threadIdx.x + threadIdx.y * blockDim.x);
	
	if( idx >= w*h) return;
    
    int x = offset + (idx/h) * lineSpacing;
    
    int i = (idx - idx/h*h);
    
    Scanline sl = lines[x / lineSpacing];
    if(i >= sl.edgeCnt-1) return;
    //printf("i: %d ",i);

    int px1 = sl.edgesX[i];
    int py1 = sl.edgesY[i];
    int px2 = sl.edgesX[i + 1];
    int py2 = sl.edgesY[i + 1];
    if (i > 0 && !sl.regionsIsGreen[i] && !sl.regionsIsWhite[i] && sl.regionsIsGreen[i - 1]) {
        //borderPoints.push_back(point_2d(px1, py1));
    }
    if (i == sl.edgeCnt - 2 && sl.regionsIsGreen[i]) {
        //borderPoints.push_back(point_2d(px2, py2));
    }
	//int cr = img[((x + y * width) << 1) | 3];
	// atomic operation
	//atomicAdd(&histCr[cr], 1);
}
*/
__device__ inline float fast_fabsf_d(float f){
    floatint fi;
    fi.f=f;
    fi.i &= 0x7fffffff;
    return fi.f; // convert bits back to float
}

__device__ inline float getDistance(int x0, int y0, int x1, int y1, int x2, int y2){
    return (fast_fabsf_d((y2-y1)*x0 + (x1-x2)*y0 + ((x2*y1) - (x1*y2)))) / (sqrt(pow(y2-y1, 2) + pow(x1-x2, 2)));
}

__global__ void Corner(const int *border, float *distance, int max){
    int idx = blockIdx.x*blockDim.x*blockDim.y + (threadIdx.x + threadIdx.y * blockDim.x);
    if(idx >= max || idx == 0 || idx == max-1){
        return; 
    }

    distance[idx] = getDistance(idx, border[idx], 0, border[0], max-1, border[max-1]);
    //printf("%d ", distance[idx]);
}

/**
 * decides which pixels belonging to the playing-field by modelling the
 * fieldborder with up to two lines
 */
void FieldDetector::proceed(
        const uint8_t *const img, const FieldColorDetector *const field, int spacing, Scanline* lines) {
    borderPoints.clear();
    vector<point_2d> pointsLeft;
    vector<point_2d> pointsDone;


    // search possible points on the field-border
    //(points on edges, with a green color on the bottom and not a green or white
    //color on the top)
    int lineSpacing = spacing;
    int offset = lineSpacing / 2;
    // maxEdgesPerScanline 16
/*
    int xi = (width-offset) / lineSpacing;
    int yi = maxEdgesPerScanline;

    uint8_t *img_d;
    Scanline* lines_d;
    thrust::device_vector<point_2d> points_d(borderPoints.size());
    cudaMalloc( (void**)&img_d, sizeof(uint8_t) * width * height * 2 );
    cudaMemcpy( img_d, img, sizeof(uint8_t) * width * height * 2, cudaMemcpyHostToDevice ) ;
    cudaMalloc( (void**)&lines_d, sizeof(Scanline ) * (xi+1));
    cudaMemcpy( lines_d, lines, sizeof(Scanline ) * (xi+1), cudaMemcpyHostToDevice ) ;


	int max = xi*yi;
	int block_num = max / 256 + 1;
    dim3 dim_(16, 16);
    printf("%d %d max %d, num: %d\n", xi, yi, max, block_num);
    
    checkPoints<<<block_num, dim_>>>(img_d, xi, yi, width, offset, lineSpacing, points_d, lines_d);
    //borderPoints = points_d
*/
    for (int x = offset; x < width; x += lineSpacing) {
        Scanline sl = lines[x / lineSpacing];
        for (int i = 0; i < sl.edgeCnt - 1 && i < maxEdgesPerScanline; i++) {
            int px1 = sl.edgesX[i];
            int py1 = sl.edgesY[i];
            int px2 = sl.edgesX[i + 1];
            int py2 = sl.edgesY[i + 1];
            if (i > 0 && !sl.regionsIsGreen[i] && !sl.regionsIsWhite[i] && sl.regionsIsGreen[i - 1]) {
                borderPoints.emplace_back(px1, py1);
            }
            if (i == sl.edgeCnt - 2 && sl.regionsIsGreen[i]) {
                borderPoints.emplace_back(px2, py2);
            }
        }
    }
    // search for the best line matching for the given border points with RANSAC
    // algorithm
    if (borderPoints.size() >= 6) {
        float bestNx = 0;
        float bestNy = 0;
        float bestD = 0;
        int maxSum = 0;
        int pointsCnt = 0;
        for (int i = 0; i < 200; i++) {
            point_2d p1 = borderPoints[(int)(dist(rng) * borderPoints.size())];
            point_2d p2 = borderPoints[(int)(dist(rng) * borderPoints.size())];
            if (p1 == p2)
                continue;
            if (p1.x > p2.x) {
                point_2d tmp = p1;
                p1 = p2;
                p2 = tmp;
            }
            float dx = p1.x - p2.x;
            float dy = p1.y - p2.y;
            float len = std::sqrt(dx * dx + dy * dy);
            if (len < 16) continue;
            float nx = -dy / len;
            float ny = dx / len;
            float d = nx * p1.x + ny * p1.y;
            int sum = 0;
            int cnt = 0;
            for (const point_2d& p : borderPoints) {
                float dist = nx * p.x + ny * p.y - d;
                if (std::fabs(dist) < 1.5f) {
                    cnt++;
                    sum += 1;
                } else if (dist > 4 && dist < 32) {
                    sum -= 1;
                }
            }
            if (sum > maxSum) {
                maxSum = sum;
                pointsCnt = cnt;
                bestNx = nx;
                bestNy = ny;
                bestD = d;
            }
        }
        // if enough points left, search for the best second line matching for the
        // rest of the border points with RANSAC algorithm
        if (pointsCnt >= 5) {
            pointsLeft.clear();
            pointsDone.clear();
            for (const point_2d &p : borderPoints) {
                float dist = fabsf(bestNx * p.x + bestNy * p.y - bestD);
                if (dist >= 1) {
                    pointsLeft.push_back(p);
                } else {
                    pointsDone.push_back(p);
                }
            }
            float bestNx2 = 0;
            float bestNy2 = 0;
            float bestD2 = 0;
            int pointsCnt2 = 0;
            if (pointsLeft.size() >= 4) {
                int maxSum2 = 0;
                for (int i = 0; i < 200; i++) {
                    point_2d p1 = pointsLeft[(int)(dist(rng) * pointsLeft.size())];
                    point_2d p2 = pointsLeft[(int)(dist(rng) * pointsLeft.size())];
                    if (p1 == p2)
                        continue;
                    if (p1.x > p2.x) {
                        point_2d tmp = p1;
                        p1 = p2;
                        p2 = tmp;
                    }
                    float dx = p1.x - p2.x;
                    float dy = p1.y - p2.y;
                    float len = std::sqrt(dx * dx + dy * dy);
                    if (len < 16) continue;
                    float nx = -dy / len;
                    float ny = dx / len;
                    float d = nx * p1.x + ny * p1.y;
                    int sum = 0;
                    int cnt = 0;
                    for (const point_2d &p : pointsLeft) {
                        float dist = nx * p.x + ny * p.y - d;
                        if (fabsf(dist) < 2) {
                            sum += 2;
                            cnt++;
                        } else if (dist > 3 && dist < 20) {
                            sum -= 1;
                        }
                    }
                    for (const point_2d &p : pointsDone) {
                        float dist = nx * p.x + ny * p.y - d;
                        if (dist > 2) {
                            sum -= 2;
                        }
                    }
                    if (sum > maxSum2) {
                        maxSum2 = sum;
                        pointsCnt2 = cnt;
                        bestNx2 = nx;
                        bestNy2 = ny;
                        bestD2 = d;
                    }
                }
            }

            //interpolate the field-border from one or two lines
            for (int x = 0; x < width; x++) {
                if (bestNy == 0) continue;
                int y1 = (int)((bestD - bestNx * x) / bestNy);
                if (pointsCnt2 >= 4 && bestNy2 != 0) {
                    int y2 = (int)((bestD2 - bestNx2 * x) / bestNy2);
                    if (y2 > y1) y1 = y2;
                }
                if (y1 < 0) y1 = 0;
                if (y1 >= height - 2) y1 = height - 2;
                fieldBorderFull[x] = y1;
            }
        } else {
            // interpolate the field-border with default values (if no field border is
            // visible)
            for (int x = 0; x < width; x++) {
                fieldBorderFull[x] = 0;  // 0 means a field border on the top of the
                // image, so that all pixels below are valid
                // field pixels
            }
        }
    }

    int* border_d;
    cudaMalloc( (void**)&border_d, sizeof(int) * width);
    cudaMemcpy( border_d, fieldBorderFull, sizeof(int) * width, cudaMemcpyHostToDevice ) ;
    float distance_data[width] = {0}, *distance_d;
    cudaMalloc( (void**)&distance_d, sizeof(int) * width);

    int block_num = width / 256 + 1;
    dim3 dim_(16, 16);

    Corner<<<block_num, dim_>>>(border_d, distance_d, width);
    cudaMemcpy( distance_data, distance_d, sizeof(int) * width, cudaMemcpyDeviceToHost ) ;

    float *max_val, *sec_val;
    corner.clear();
    get_max_sec_element(distance_data, distance_data+width, max_val, sec_val);
    //float = sec_element(distance_data, distance_data+width);
    if(*max_val > MIN_DIS){
        int max_i = distance(distance_data, max_val);
        corner.push_back(point_2d(max_i, fieldBorderFull[max_i]));
        if(*sec_val > MIN_DIS){
            int sec_i = distance(distance_data, sec_val);
            //printf("sec_i %d sec_val %f\n", sec_i, *sec_val);
            if(abs(sec_i - max_i) > width/6)
                corner.push_back(point_2d(sec_i, fieldBorderFull[sec_i]));
        }
        //printf("max_i %d max_val %f\n", max_i, *max_val);
    }
    cudaFree(border_d);
    cudaFree(distance_d);
    // use fieldBorderFull from last frame.
    //cudaFree(img_d);
}

}

