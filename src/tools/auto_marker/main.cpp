#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include "darknet/network.h"
#include "darknet/parser.h"
#include "cuda/cudaproc.h"

using namespace cv;
using namespace std;
namespace bfs = boost::filesystem;

vector<string> get_filenames(const string& dir)
{
    vector<string> filenames;
    bfs::path path(dir);
    if (!bfs::exists(path))
        return filenames;

    bfs::directory_iterator end_iter;
    for (bfs::directory_iterator iter(path); iter!=end_iter; ++iter)
    {
        if (bfs::is_regular_file(iter->status()))
        {
            string picname=iter->path().string();
            bfs::path file(picname);

            if(file.extension().string()==".png"||file.extension().string()==".jpg")
                filenames.push_back(picname);
        }

        if (bfs::is_directory(iter->status()))
        {
            vector<string> temp = get_filenames(iter->path().string());
            filenames.insert(filenames.end(), temp.begin(), temp.end());
        }
    }
    return filenames;
}


int main(int argc, char **argv) 
{
    string dir;
    if(argc==2)
        dir.assign(argv[1]);
    else
    {
        cout<<"please run with images path"<<endl;
        exit(0);
    }
    string label_path = dir+"_labels";
    bfs::path lpath(label_path);
    if(!bfs::exists(lpath))
    {
        bfs::create_directories(lpath);
    }
    vector<string> picnames = get_filenames(dir);
    network net;
    unsigned char *dev_src_;
    unsigned char *dev_sized_;
    float *dev_rgbfp_;
    cudaError_t err;
    net.gpu_index = 0;
    string cfg_file = "data/algorithm/robocup.cfg";
    string w_file = "data/algorithm/robocup.weights";
    net = parse_network_cfg_custom((char*)cfg_file.c_str(), 1);
    load_weights(&net, (char*)w_file.c_str());
    set_batch_network(&net, 1);
    fuse_conv_batchnorm(net);
    calculate_binary_weights(net);
    srand(2222222);
    int nw = net.w, nh = net.h;
    int resize_size = nh*nw*3*sizeof(unsigned char);
    int rgbfp_size = nh*nw*3*sizeof(float);
    err = cudaMalloc((void **)&dev_sized_, resize_size);
    check_error(err);
    err = cudaMalloc((void **)&dev_rgbfp_, rgbfp_size);
    check_error(err);    
    for(int i=0;i<picnames.size();i++)
    {
        cout<<"label: "<<picnames[i]<<endl;
        Mat src = imread(picnames[i]);
        int w = src.size().width;
        int h = src.size().height;
        int src_size = h*w*3*sizeof(unsigned char);
        err = cudaMalloc((void **)&dev_src_, src_size);
        check_error(err);
        err = cudaMemcpy(dev_src_, src.data, src_size, cudaMemcpyHostToDevice);
        check_error(err);
        cudaResizePacked(dev_src_, w, h, dev_sized_, nw, nh);
        cudaBGR2RGBfp(dev_sized_, dev_rgbfp_, nw, nh);
        layer l = net.layers[net.n - 1];
        network_predict(net, dev_rgbfp_, 0);
        int nboxes = 0;
        float nms = .45;
        detection *dets = get_network_boxes(&net, w, h, 0.5, 0.5, 0, 1, &nboxes, 0);
        bfs::path file(picnames[i]);
        string fn = file.filename().string();
        ofstream label(label_path+'/'+fn.substr(0, fn.size()-3)+"txt");
        if (nms)
            do_nms_sort(dets, nboxes, l.classes, nms);
        for (int i = 0; i < nboxes; i++)
        {
            if(dets[i].prob[0] > dets[i].prob[1])
            {
                if(dets[i].prob[0] >= 0.5)
                {
                    int bx = (dets[i].bbox.x - dets[i].bbox.w / 2.0)*w;
                    int by = (dets[i].bbox.y - dets[i].bbox.h / 2.0)*h;
                    int bw = dets[i].bbox.w*w, bh = dets[i].bbox.h*h;
                    float w_h = (float)bw/(float)bh;
                    if(bw>=20 && bh>=20 && fabs(w_h-1.0)<0.3)
                    {
                        label<<0<<' '<<dets[i].bbox.x<<' '<<dets[i].bbox.y<<' '<<dets[i].bbox.w<<' '<<dets[i].bbox.h<<endl;
                    }
                }
            }
            else
            {
                if(dets[i].prob[1] >= 0.4)
                {
                    int px = (dets[i].bbox.x - dets[i].bbox.w / 2.0)*w;
                    int py = (dets[i].bbox.y - dets[i].bbox.h / 2.0)*h;
                    int pw = dets[i].bbox.w*w, ph = dets[i].bbox.h*h;
                    if(pw>=15 && ph>=20)
                    {
                        label<<1<<' '<<dets[i].bbox.x<<' '<<dets[i].bbox.y<<' '<<dets[i].bbox.w<<' '<<dets[i].bbox.h<<endl;
                    }
                }
            }
        }
        label.close();
        free_detections(dets, nboxes);
        cudaFree(dev_src_);
    }
    free_network(net);
    return 0;
}