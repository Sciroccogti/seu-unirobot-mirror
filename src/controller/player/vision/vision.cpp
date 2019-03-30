#include "vision.hpp"
#include "parser/camera_parser.hpp"
#include "darknet/parser.h"
#include <cuda_runtime.h>
#include "cuda/cudaproc.h"
#include "server/server.hpp"
#include <algorithm>
#include "core/worldmodel.hpp"
#include "imageproc/imageproc.hpp"
#include <fstream>

using namespace std;
using namespace cv;
using namespace robot_math;
using namespace Eigen;
using namespace vision;
using namespace robot;

Vision::Vision(): timer(CONF->get_config_value<int>("vision_period"))
{
    p_count_ = 0;
    cant_see_ball_count_ = 0;
    is_busy_ = false;
    w_ = CONF->get_config_value<int>("image.width");
    h_ = CONF->get_config_value<int>("image.height");
    img_sd_type_ = IMAGE_SEND_RESULT;
    camera_src_ = nullptr;
    detector_ = make_shared<Detector>(w_, h_);
    parser::camera_info_parser::parse(CONF->get_config_value<string>(CONF->player() + ".camera_info_file"), camera_infos_);
    parser::camera_param_parser::parse(CONF->get_config_value<string>(CONF->player() + ".camera_params_file"), params_);
    LOG(LOG_INFO) << setw(12) << "algorithm:" << setw(18) << "[vision]" << " started!" << endll;
    ball_id_=0;
    post_id_=1;
    ball_prob_ = CONF->get_config_value<float>("detection_prob.ball");
    post_prob_ = CONF->get_config_value<float>("detection_prob.post");
}

Vision::~Vision()
{
    LOG(LOG_INFO) << setw(12) << "algorithm:" << setw(18) << "[vision]" << " ended!" << endll;
}

void Vision::set_camera_info(const camera_info &para)
{
    for (auto &item : camera_infos_)
    {
        if (item.second.id == para.id)
        {
            item.second.value = para.value;
            break;
        }
    }
}

Vector2d Vision::odometry(const Vector2i &pos, const robot_math::transform_matrix &camera_matrix)
{
    float Xw, Yw;
    float OC = camera_matrix.p().z();
    float roll = -static_cast<float>(camera_matrix.x_rotate());
    float theta = static_cast<float>(camera_matrix.y_rotate());
    //theta = theta-0.05*(M_PI_2-theta);
    Vector2i Pos(pos.x()-params_.cx, params_.cy-pos.y());
    Vector2i calCenterPos(Pos.x()/cos(roll), Pos.y()+Pos.x()*tan(roll));
    Vector2i calPos(calCenterPos.x()+params_.cx, params_.cy-calCenterPos.y());
    double gama = atan((params_.cy-calPos.y())/params_.fy);
    double O_C_P = M_PI_2-theta+gama;
    Yw = OC*tan(O_C_P);
    Xw = (calPos.x()-params_.cx)*OC*cos(gama)/(cos(O_C_P)*params_.fx);
    return Vector2d(Xw, Yw);
}

Vector2d Vision::camera2self(const Vector2d &pos, double head_yaw)
{
    return rotation_mat_2d(head_yaw+90.0)*pos;
}

Vector2i Vision::undistored(const Eigen::Vector2i &pix)
{
    int x = pix.x(), y=pix.y();
    float u_distorted = 0, v_distorted = 0;
    float x1,y1,x2,y2;
    x1 = (x-w_/2)/params_.fx;
    y1 = (y-h_/2)/params_.fy;
    float r2;
    r2 = pow(x1,2)+pow(y1,2);
    x2  = x1/(1+params_.k1*r2+params_.k2*pow(r2,2));
    y2 = y1/(1+params_.k1*r2+params_.k2*pow(r2,2));
    int ox = params_.fx*x2+params_.cx;
    int oy = params_.fy*y2+params_.cy;
    bound(0, w_, ox);
    bound(0, h_, oy);
    return Vector2i(ox, oy);   
}

void Vision::get_point_dis(int x, int y)
{
    transform_matrix camera_matrix;
    frame_mtx_.lock();
    camera_matrix = camera_matrix_;
    frame_mtx_.unlock();
    Vector2d dis(0.0, 0.0);
    dis=odometry(Vector2i(x,y), camera_matrix);
    float xx=static_cast<float>(dis[0]), yy=static_cast<float>(dis[1]);
    tcp_command cmd;
    cmd.type = REMOTE_DATA;
    cmd.size = 2*enum_size+2*float_size;
    remote_data_type t1=IMAGE_SEND_TYPE;
    image_send_type t2=IMAGE_SEND_DIS;
    cmd.data.clear();
    cmd.data.append((char *) &t1, enum_size);
    cmd.data.append((char *) &t2, enum_size);
    cmd.data.append((char *) &xx, float_size);
    cmd.data.append((char *) &yy, float_size);
    SERVER->write(cmd);
}

void Vision::run()
{
    if (is_alive_)
    {
        if (camera_src_ == nullptr)
            return;
        if (is_busy_)
            return;
        is_busy_ = true;
        p_count_ ++;
        robot_math::transform_matrix camera_matrix;
        float head_yaw;
        
        cudaError_t err;
        frame_mtx_.lock();
        err = cudaMemcpy(dev_src_, camera_src_, src_size_, cudaMemcpyHostToDevice);
        check_error(err);
        if(OPTS->use_robot())
        {
            camera_matrix = camera_matrix_;
            head_yaw = head_yaw_;
        }
        frame_mtx_.unlock();

        if (use_mv_)
        {
            cudaBayer2BGR(dev_src_, dev_bgr_,  camera_w_,  camera_h_, camera_infos_["saturation"].value,
                        camera_infos_["red_gain"].value, camera_infos_["green_gain"].value, camera_infos_["blue_gain"].value);
        }
        else
        {
            cudaYUYV2BGR(dev_src_,  dev_bgr_,  camera_w_,  camera_h_);
        }
        cudaResizePacked(dev_bgr_, camera_w_,  camera_h_,  dev_ori_, w_,  h_);
        cudaBGR2YUV422(dev_ori_, dev_yuyv_, w_, h_);
        cudaResizePacked(dev_ori_, w_, h_, dev_sized_, net_.w, net_.h);
        cudaBGR2RGBfp(dev_sized_, dev_rgbfp_, net_.w, net_.h);
        
        detector_->process(dev_yuyv_);
        
        layer l = net_.layers[net_.n - 1];
        network_predict(net_, dev_rgbfp_, 0);
        int nboxes = 0;
        float nms = .45;
        detection *dets = get_network_boxes(&net_, w_, h_, 0.5, 0.5, 0, 1, &nboxes, 0);

        if (nms)
            do_nms_sort(dets, nboxes, l.classes, nms);
        ball_dets_.clear();
        post_dets_.clear();
        for (int i = 0; i < nboxes; i++)
        {
            if(dets[i].prob[ball_id_] > dets[i].prob[post_id_])
            {
                if(dets[i].prob[ball_id_] >= ball_prob_)
                    ball_dets_.push_back(object_det(ball_id_, dets[i].prob[ball_id_],
                        (dets[i].bbox.x - dets[i].bbox.w / 2.0)*w_, (dets[i].bbox.y - dets[i].bbox.h / 2.0)*h_,
                        dets[i].bbox.w*w_, dets[i].bbox.h*h_));
            }
            else
            {
                if(dets[i].prob[post_id_] >= post_prob_)
                    post_dets_.push_back(object_det(post_id_, dets[i].prob[post_id_],
                        (dets[i].bbox.x - dets[i].bbox.w / 2.0)*w_, (dets[i].bbox.y - dets[i].bbox.h / 2.0)*h_,
                        dets[i].bbox.w*w_, dets[i].bbox.h*h_));
            }
        }
        sort(ball_dets_.rbegin(), ball_dets_.rend());
        sort(post_dets_.begin(), post_dets_.end());
        free_detections(dets, nboxes);

        if(OPTS->use_robot())
        {
            self_block p = WM->self();
            if(!ball_dets_.empty())
            {
                //LOG(LOG_INFO)<<"*********************\n";
                Vector2i ball_pix(ball_dets_[0].x+ball_dets_[0].w/2, ball_dets_[0].y+ball_dets_[0].h*0.8);
                Vector2d odo_res = odometry(ball_pix, camera_matrix);
                //LOG(LOG_INFO) <<odo_res.transpose()<<endll;
                LOG(LOG_INFO)<<"ball: " <<odo_res.norm()<<endll;
                Vector2d ball_pos = camera2self(odo_res, head_yaw);
                //LOG(LOG_INFO) <<ball_pos.transpose()<<endll;
                cant_see_ball_count_=0;
                //LOG(LOG_INFO)<<p.global.x()<<'\t'<< p.global.y()<<endll;
                Vector2d temp_ball = Vector2d(p.global.x(), p.global.y())+rotation_mat_2d(-p.dir)*ball_pos;
                //LOG(LOG_INFO) <<temp_ball.transpose()<<endll;
                //LOG(LOG_INFO)<<"*********************"<<endll;
                int tempx=ball_dets_[0].x+ball_dets_[0].w/2-w_/2, tempy=ball_dets_[0].y+ball_dets_[0].h-h_/2;
                WM->set_ball_pos(temp_ball, ball_pos, Vector2d(tempx/(float)w_*params_.h_v, tempy/(float)h_*params_.v_v));
            }
            else
            {
                cant_see_ball_count_++;
                if(cant_see_ball_count_*period_ms_>10000)
                    WM->set_ball_pos(Vector2d(0,0), Vector2d(0,0), Vector2d(0,0), false);
            }
            int post_num=0;
            list< GoalPost > posts_;
            for(auto &post: post_dets_)
            {
                Vector2i post_pix(post.x+post.w/2, post.y+post.h);
                Vector2d odo_res = odometry(undistored(post_pix), camera_matrix);
                //LOG(LOG_INFO)<<"post: "<<odo_res.norm()<<endll;
                Vector2d ball_pos = camera2self(odo_res, head_yaw);
                if(++post_num>=2) break;
            }
            SL->update(player_info(p.global.x(), p.global.y(), p.dir), posts_);
        }

        if (OPTS->use_debug())
        {
            Mat bgr(h_, w_, CV_8UC3);
            err = cudaMemcpy(bgr.data, dev_ori_, ori_size_, cudaMemcpyDeviceToHost);
            check_error(err);    
                  
            if(OPTS->image_record())
            {
                send_image(bgr);
                is_busy_ = false;
                return;
            }

            if (img_sd_type_ == IMAGE_SEND_ORIGIN)
            {
                send_image(bgr);
            }
            else if (img_sd_type_ == IMAGE_SEND_RESULT)
            {   
                /*
                for(int j=0;j<480;j++)
                    for(int i=0;i<640;i++)
                        if(detector_->isGreen(i, j))
                            bgr.at<Vec3b>(j, i) = Vec3b(0, 255, 0);
                */
                if(!ball_dets_.empty())
                {
                    rectangle(bgr, Point(ball_dets_[0].x, ball_dets_[0].y), Point(ball_dets_[0].x + ball_dets_[0].w,
                        ball_dets_[0].y + ball_dets_[0].h), Scalar(255, 0, 0), 2);
                    putText(bgr, to_string(ball_dets_[0].prob).substr(0,4), Point(ball_dets_[0].x, ball_dets_[0].y),
                        FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, 8);
                }
                int i=0;
                for(auto &dd: post_dets_)
                {
                    if(i>=2) break;
                    rectangle(bgr, Point(post_dets_[i].x, post_dets_[i].y), Point(post_dets_[i].x + post_dets_[i].w,
                        post_dets_[i].y + post_dets_[i].h), Scalar(0, 0, 255), 2);
                    putText(bgr, to_string(post_dets_[i].prob).substr(0,4), Point(post_dets_[i].x, post_dets_[i].y),
                        FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
                    i++;
                }
                send_image(bgr);
            }
        }
        is_busy_ = false;
    }
}

void Vision::send_image(const cv::Mat &src)
{
    cv::Mat bgr;
    src.copyTo(bgr);
    std::vector<unsigned char> jpgbuf;
    cv::imencode(".jpg", bgr, jpgbuf);
    bgr.release();
    tcp_command cmd;
    cmd.type = IMG_DATA;
    cmd.size = jpgbuf.size();
    cmd.data.assign((char *) & (jpgbuf[0]), jpgbuf.size());
    SERVER->write(cmd);
}

void Vision::updata(const pub_ptr &pub, const int &type)
{

    if (!is_alive_)
    {
        return;
    }
    if (type == sensor::SENSOR_CAMERA)
    {
        shared_ptr<camera> sptr = dynamic_pointer_cast<camera>(pub);
        if (camera_src_ ==  nullptr)
        {
            camera_w_ = sptr->camera_w();
            camera_h_ = sptr->camera_h();
            camera_size_ = sptr->camera_size();
            use_mv_ = sptr->use_mv();
            src_size_ = camera_size_;
            bgr_size_ = camera_w_ * camera_h_ * 3;
            camera_src_ = (unsigned char *)malloc(camera_size_);
            cudaError_t err;
            err = cudaMalloc((void **) &dev_src_,  src_size_);
            check_error(err);
            err = cudaMalloc((void **) &dev_bgr_, bgr_size_);
            check_error(err);
        }
        frame_mtx_.lock();
        memcpy(camera_src_, sptr->buffer(), src_size_);
        if(OPTS->use_robot())
        {
            AngleAxisd roll, pitch;
            pitch = AngleAxisd(deg2rad(imu_data_.pitch), Vector3d::UnitY());
            roll = AngleAxisd(deg2rad(imu_data_.roll), Vector3d::UnitX());
            Quaternion<double> quat = roll * pitch;
            int spf = WM->support_foot();
            std::vector<double> foot_degs = ROBOT->get_foot_degs(spf);
            std::vector<double> head_degs = ROBOT->get_head_degs();
            head_yaw_  = -head_degs[0];
            transform_matrix body = ROBOT->leg_forward_kinematics(foot_degs, spf);
            body.set_R(quat.matrix());
            camera_matrix_ = body*transform_matrix(0,0,ROBOT->trunk_length())*transform_matrix(head_degs[0],'z')
                            *transform_matrix(0, 0, ROBOT->neck_length())*transform_matrix(head_degs[1], 'y')
                            *transform_matrix(-0.02, 0, ROBOT->head_length());
        }
        frame_mtx_.unlock();
        return;
    }
    if(type == sensor::SENSOR_IMU)
    {
        shared_ptr<imu> sptr = dynamic_pointer_cast<imu>(pub);
        imu_mtx_.lock();
        imu_data_ = sptr->data();
        imu_mtx_.unlock();
        return;
    }
}

bool Vision::start()
{
    names_.clear();
    ifstream ifs(CONF->get_config_value<string>("net_names_file"));

    while (!ifs.eof())
    {
        string s;
        ifs >> s;
        names_.push_back(s);
    }

    ifs.close();
    net_.gpu_index = 0;
    net_ = parse_network_cfg_custom((char *)CONF->get_config_value<string>("net_cfg_file").c_str(), 1);
    load_weights(&net_, (char *)CONF->get_config_value<string>("net_weights_file").c_str());
    set_batch_network(&net_, 1);
    fuse_conv_batchnorm(net_);
    calculate_binary_weights(net_);
    srand(2222222);

    ori_size_ = w_ * h_ * 3;
    yuyv_size_ = w_*h_*2;
    sized_size_ = net_.w * net_.h * 3;
    rgbf_size_ = w_ * h_ * 3 * sizeof(float);

    cudaError_t err;
    err = cudaMalloc((void **)&dev_ori_, ori_size_);
    check_error(err);
    err = cudaMalloc((void**)&dev_yuyv_, yuyv_size_);
    check_error(err);
    err = cudaMalloc((void **)&dev_sized_, sized_size_);
    check_error(err);
    err = cudaMalloc((void **)&dev_rgbfp_, rgbf_size_);
    check_error(err);
    is_alive_ = true;
    start_timer();
    return true;
}

void Vision::stop()
{
    if (is_alive_)
    {
        delete_timer();
        free_network(net_);
        free(camera_src_);
        cudaFree(dev_ori_);
        cudaFree(dev_yuyv_);
        cudaFree(dev_src_);
        cudaFree(dev_bgr_);
        cudaFree(dev_rgbfp_);
        cudaFree(dev_sized_);
    }
    is_alive_ = false;
}
