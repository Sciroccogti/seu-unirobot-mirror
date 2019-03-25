#include "detector.h"

using namespace std;
using namespace cv;

namespace vision
{
Detector::Detector(int width, int height):_width(width), _height(height)
{
    lutCb = (int8_t*)malloc(sizeof(*lutCb)*width);
    lutCr = (int8_t*)malloc(sizeof(*lutCr)*width);
    buf = (uint8_t*) malloc(sizeof(uint8_t) * _width * _height * 2);
    for(int i=0;i<width;i++){
        if((i&1)==0){
            lutCb[i]=1;
            lutCr[i]=3;
        }else{
            lutCb[i]=-1;
            lutCr[i]=1;
        }
    }
    fcd = new FieldColorDetector(width, height, lutCb, lutCr);
    rc = new RegionClassifier(width, height, true, lutCb, lutCr);
    fd = new FieldDetector(width, height, lutCb, lutCr);
}

void Detector::process(uint8_t *_img_d)
{
    img_d = _img_d;
    fcd->proceed(img_d);
    cudaMemcpy( buf, img_d, sizeof(uint8_t) * _width * _height * 2, cudaMemcpyDeviceToHost ) ;
    rc->proceed(buf, fcd);
    fd->proceed(buf, fcd, rc->getLineSpacing(), rc->getScanVertical());
    
}

bool Detector::isGreen(int x, int y)
{
    int cy=fcd->getY(buf,x,y);
    int cb=fcd->getCb(buf,x,y);
    int cr=fcd->getCr(buf,x,y);
    if(fcd->isGreen(cy,cb,cr)){
        return true;
    }
    else{
        return false;
    }
}
}