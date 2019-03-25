#include "imageproc.hpp"
#include <cmath>

using namespace cv;
using namespace std;

namespace vision
{
    bool obj_prob_greater(const object_prob &a, const object_prob &b)
    {
        return a.prob > b.prob;
    }

    bool is_same(object_prob o1, object_prob o2, float l=40)
    {
        return sqrt(pow(o1.x-o2.x, 2)+pow(o1.y-o2.y, 2))<l;
    }

    vector<object_prob> ball_and_post_detection(network &net, float *rgbfp, bool from_gpu, object_prob ball,
        object_prob post, int w, int h, float thresh, float hier)
    {
        vector<object_prob> ball_dets, post_dets, results;
        layer l = net.layers[net.n - 1];
        network_predict(net, rgbfp, from_gpu?1:0);
        int nboxes = 0;
        float nms = .45;
        detection *dets = get_network_boxes(&net, w, h, thresh, hier, 0, 1, &nboxes, 0);
        if (nms)
        {
            do_nms_sort(dets, nboxes, l.classes, nms);
        }
        for (int i = 0; i < nboxes; i++)
        {
            if(dets[i].prob[ball.id] > dets[i].prob[post.id])
            {
                if(dets[i].prob[ball.id] > ball.prob)
                    ball_dets.push_back(object_prob(ball.id, dets[i].prob[ball.id], 
                        dets[i].bbox.x*w, (dets[i].bbox.y+dets[i].bbox.h/2.0)*h));
            }
            else
            {
                if(dets[i].prob[post.id] > post.prob)
                    post_dets.push_back(object_prob(post.id, dets[i].prob[post.id],
                        dets[i].bbox.x*w, (dets[i].bbox.y+dets[i].bbox.h/2.0)*h));
            }
        }
        free_detections(dets, nboxes);
        sort(ball_dets.begin(), ball_dets.end(), obj_prob_greater);
        sort(post_dets.begin(), post_dets.end(), obj_prob_greater);
        if(!ball_dets.empty()) results.push_back(ball_dets[0]);
        for(int i=0;i<post_dets.size();i++)
        {
            if(i==0) results.push_back(ball_dets[0]);
            else
            {
                if(!is_same(ball_dets[0], ball_dets[i]))
                {
                    results.push_back(ball_dets[i]);
                    break;
                }
            }
        }
        return results;
    }

    float* rgb2rgbpf(const Mat &rgb)
    {
        int w = rgb.size().width;
        int h = rgb.size().height;
        float  *rgbpf = new float[h*w*3];
        int psize=w*h;
        for(int i=0;i<h;i++)
        {
            for(int j=0;j<w;j++)
            {
                unsigned int r = rgb.at<Vec3b>(i, j)[0];
                unsigned int g = rgb.at<Vec3b>(i, j)[1];
                unsigned int b = rgb.at<Vec3b>(i, j)[2];
                rgbpf[i*w+j] = static_cast<float>(r/255.0);
                rgbpf[psize+i*w+j] = static_cast<float>(g/255.0);
                rgbpf[2*psize+i*w+j] = static_cast<float>(b/255.0);
            }
        }
        return rgbpf;
    }
}

