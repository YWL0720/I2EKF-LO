// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//#include "ros/package.h"
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include "IMU_Processing.hpp"
#include "ros/package.h"
#include <unistd.h>
#include <Python.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <math.h>
#ifndef DEPLOY
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

#define PUBFRAME_PERIOD     (20)

float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, \
 effect_feat_num = 0, scan_count = 0, publish_count = 0;

int feats_points_size = 0;

int feats_points_full_size = 0;

int NUM_MAX_UNDISTORT = 0;

double res_mean_last = 0.05;
double gyr_cov = 0.1, acc_cov = 0.1;
double last_timestamp_lidar = 0, last_timestamp_imu = 0.0;
double filter_size_surf_min = 0, filter_size_map_min = 0;
double cube_len = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;

// Time Log Variables
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0;

int distort_time = 0;

int lidar_type, pcd_save_interval = -1, pcd_index = 0;
bool lidar_pushed, flg_reset, flg_exit = false, flg_EKF_inited = true;
bool imu_en = false;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool runtime_pos_log = false, pcd_save_en = false,  path_en = true;

bool cut_frame = true;
int cut_frame_num = 1, frame_num = 0;
double time_lag_IMU_wtr_lidar = 0.0;
ofstream fout_result;
double cov_lidar = 0.001;

bool adaptive_cov = false;
double K_cov = 100;
double a_cov = 5;
double b_cov = 10;

shared_ptr<ImuProcess> p_imu(new ImuProcess());
StatesGroup state_propagat;

vector<BoxPointType> cub_needrm;

deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<std::vector<Point3D>> lidar_points_buffer;
deque<double> time_buffer;
deque<sensor_msgs::Imu::Ptr> imu_buffer;

vector<vector<int>> pointSearchInd_surf;
vector<PointVector> Nearest_Points;
bool point_selected_surf[100000] = {0};
float res_last[100000] = {0.0};
double total_residual;

//surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

std::vector<Point3D> feats_points;
std::vector<Point3D> feats_points_full;

double current_dt = 0.1;
double current_end_time = 0.1;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE ikdtree;

M3D last_rot(M3D::Zero());
V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D last_odom(Zero3d);


//estimator inputs and output;
MeasureGroup Measures;
StatesGroup state;
StatesGroup last_state;

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
pcl::PCDWriter pcd_writer;
string all_points_dir;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());


float calc_dist(PointType p1, PointType p2) {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

void calcBodyVar(Eigen::Vector3d &pb, const float range_inc,
                 const float degree_inc, Eigen::Matrix3d &var) {
    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
    float range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
            pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0,
            -direction(0), -direction(1), direction(0), 0;
    Eigen::Vector3d base_vector1(1, 1,
                                 -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
            base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    var = direction * range_var * direction.transpose() +
          A * direction_var * A.transpose();
}

void SigHandle(int sig) {
    if (pcd_save_en && pcd_save_interval < 0){
        all_points_dir = string(root_dir + "/PCD/PCD_all" + string(".pcd"));
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp) {
    V3D rot_ang(Log(state.rot_end));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state.pos_end(0), state.pos_end(1), state.pos_end(2)); // Pos
    fprintf(fp, "%lf %lf %lf ", state.vel_end(0), state.vel_end(1), state.vel_end(2)); // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
    fprintf(fp, "%lf %lf %lf ", state.bias_g(0), state.bias_g(1), state.bias_g(2));    // Bias_g
    fprintf(fp, "%lf %lf %lf ", state.bias_a(0), state.bias_a(1), state.bias_a(2));    // Bias_a
    fprintf(fp, "%lf %lf %lf ", state.gravity(0), state.gravity(1), state.gravity(2)); // Bias_a  
    fprintf(fp, "\r\n");
    fflush(fp);
}


void pointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot_end * (state.offset_R_L_I * p_body + state.offset_T_L_I) + state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->normal_x = pi->normal_x;
    po->normal_y = pi->normal_y;
    po->normal_z = pi->normal_z;
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state.rot_end * (state.offset_R_L_I * p_body + state.offset_T_L_I) + state.pos_end);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointTypeRGB *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot_end * (state.offset_R_L_I * p_body + state.offset_T_L_I) + state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->r = pi->normal_x;
    po->g = pi->normal_y;
    po->b = pi->normal_z;

    float intensity = pi->intensity;
    intensity = intensity - floor(intensity);

    int reflection_map = intensity * 10000;
}

int points_cache_size = 0;

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
    for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}


BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
int kdtree_delete_counter = 0;
void lasermap_fov_segment()
{
    cub_needrm.clear();

    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = state.pos_end;

    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;
    points_cache_collect();

    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);

}

double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_WARN("lidar loop back, clear buffer");
        lidar_buffer.clear();
        time_buffer.clear();
        lidar_points_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
        printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    }

    if (cut_frame)
    {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        deque<std::vector<Point3D>> points_lidar;

        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, points_lidar, cut_frame_num, scan_count);

        while (!ptr.empty() && !timestamp_lidar.empty() && !points_lidar.empty())
        {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
            timestamp_lidar.pop_front();
            lidar_points_buffer.push_back(points_lidar.front());
            points_lidar.pop_front();
        }
    }
    else
    {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        std::vector<Point3D> points;
        p_pre->process(msg, ptr, points);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);
        lidar_points_buffer.push_back(points);
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear Lidar buffer.");
        lidar_buffer.clear();
        time_buffer.clear();
    }

    last_timestamp_lidar = msg->header.stamp.toSec();
    if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
        printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    }

    if ((lidar_type == VELO || lidar_type == OUSTER || lidar_type == PANDAR || lidar_type == ROBOSENSE) && cut_frame)
    {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        deque<std::vector<Point3D>> pts_lidar;
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, pts_lidar, cut_frame_num, scan_count);
        while (!ptr.empty() && !timestamp_lidar.empty() && !pts_lidar.empty())
        {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//unit:s
            timestamp_lidar.pop_front();
            lidar_points_buffer.push_back(pts_lidar.front());
            pts_lidar.pop_front();
        }
    }
    else
    {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        std::vector<Point3D> points;

        p_pre->process(msg, ptr, points);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(msg->header.stamp.toSec());
        lidar_points_buffer.push_back(points);

    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || lidar_points_buffer.empty())
    {
        return false;
    }


    /** push a lidar scan **/
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.points = lidar_points_buffer.front();

        if (meas.lidar->points.size() <= 1)
        {
            ROS_WARN("Too few input point cloud!\n");
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            lidar_points_buffer.pop_front();
            return false;
        }

        meas.lidar_beg_time = time_buffer.front(); //unit:s

        if (lidar_type == L515)
            lidar_end_time = meas.lidar_beg_time;
        else
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); //unit:s

        lidar_pushed = true;
    }

    lidar_buffer.pop_front();
    lidar_points_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

bool sync_packages_only_lidar(MeasureGroup &meas) {
    if (lidar_buffer.empty())
        return false;

    /** push a lidar scan **/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();

        if (meas.lidar->points.size() <= 1) {
            ROS_WARN("Too few input point cloud!\n");
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }

        meas.lidar_beg_time = time_buffer.front(); //unit:s

        if (lidar_type == L515)
            lidar_end_time = meas.lidar_beg_time;
        else
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); //unit:s

        lidar_pushed = true;
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

PointCloudXYZI point3DtoPCL(std::vector<Point3D> &v_point_temp, int type)
{
    PointCloudXYZI::Ptr p_cloud_temp(new PointCloudXYZI);
    switch (type)
    {
        case RAW:
            for(int i = 0; i < v_point_temp.size(); i++)
            {
                PointType cloud_temp;
                cloud_temp.x = v_point_temp[i].raw_point.x();
                cloud_temp.y = v_point_temp[i].raw_point.y();
                cloud_temp.z = v_point_temp[i].raw_point.z();
                cloud_temp.normal_x = 0;
                cloud_temp.normal_y = 0;
                cloud_temp.normal_z = 0;
                cloud_temp.intensity = v_point_temp[i].intensity;
                cloud_temp.curvature = v_point_temp[i].relative_time;
                p_cloud_temp->points.push_back(cloud_temp);
            }
            break;
        case UNDISTORT:
            for(int i = 0; i < v_point_temp.size(); i++)
            {
                PointType cloud_temp;
                cloud_temp.x = v_point_temp[i].undistort_lidar_point.x();
                cloud_temp.y = v_point_temp[i].undistort_lidar_point.y();
                cloud_temp.z = v_point_temp[i].undistort_lidar_point.z();
                cloud_temp.normal_x = 0;
                cloud_temp.normal_y = 0;
                cloud_temp.normal_z = 0;
                cloud_temp.intensity = v_point_temp[i].intensity;
                cloud_temp.curvature = v_point_temp[i].relative_time;
                p_cloud_temp->points.push_back(cloud_temp);
            }
            break;
        case WORLD:
            for(int i = 0; i < v_point_temp.size(); i++)
            {
                PointType cloud_temp;
                cloud_temp.x = v_point_temp[i].world_point.x();
                cloud_temp.y = v_point_temp[i].world_point.y();
                cloud_temp.z = v_point_temp[i].world_point.z();
                cloud_temp.normal_x = 0;
                cloud_temp.normal_y = 0;
                cloud_temp.normal_z = 0;
                cloud_temp.intensity = v_point_temp[i].intensity;
                cloud_temp.curvature = v_point_temp[i].relative_time;

                p_cloud_temp->points.push_back(cloud_temp);
            }
            break;
        default:
            for(int i = 0; i < v_point_temp.size(); i++)
            {
                PointType cloud_temp;
                cloud_temp.x = v_point_temp[i].raw_point.x();
                cloud_temp.y = v_point_temp[i].raw_point.y();
                cloud_temp.z = v_point_temp[i].raw_point.z();
                cloud_temp.normal_x = 0;
                cloud_temp.normal_y = 0;
                cloud_temp.normal_z = 0;
                cloud_temp.intensity = v_point_temp[i].intensity;
                cloud_temp.curvature = v_point_temp[i].relative_time;

                p_cloud_temp->points.push_back(cloud_temp);
            }
            break;
    }
    return *p_cloud_temp;
}

PointType point3DtoPCLPoint(Point3D& point_3d, int type)
{
    PointType point_temp;
    switch (type)
    {
        case RAW:
            point_temp.x = point_3d.raw_point.x();
            point_temp.y = point_3d.raw_point.y();
            point_temp.z = point_3d.raw_point.z();
            break;
        case UNDISTORT:
            point_temp.x = point_3d.undistort_lidar_point.x();
            point_temp.y = point_3d.undistort_lidar_point.y();
            point_temp.z = point_3d.undistort_lidar_point.z();
            break;
        case WORLD:
            point_temp.x = point_3d.world_point.x();
            point_temp.y = point_3d.world_point.y();
            point_temp.z = point_3d.world_point.z();
            break;
        default:
            point_temp.x = point_3d.raw_point.x();
            point_temp.y = point_3d.raw_point.y();
            point_temp.z = point_3d.raw_point.z();
            break;
    }
    point_temp.intensity = point_3d.intensity;
    return point_temp;
}

bool log_time = false;


void iterCurrentPoint3D(Point3D& current_point)
{
    Sophus::SE3d T_begin(last_state.rot_end, last_state.pos_end);
    Sophus::SE3d T_end(state.rot_end, state.pos_end);
    double offset_time = current_dt - (current_end_time - current_point.relative_time) / 1000.0;
    double scale = offset_time / current_dt;
    scale = scale >= 1 ? 1 : scale <= 0 ? 0 : scale;
//    if (log_time)
//        f << "offset_time = " << offset_time << " current_dt = " << current_dt << " " << "current_end_time = "  << current_end_time << " relative_time = " <<  current_point.relative_time << " scale = " << scale << endl;

//     T_wj = inter(T_wb, T_we)   T_ej = T_ew * T_wj   P_e = T_ej * P_j  P_w = T_wj * P_j
//    Sophus::SE3d T_j = Sophus::interpolate(T_begin, T_end, scale);
//    Sophus::SE3d T_ej = T_end.inverse() * T_wj;
//    current_point.undistort_lidar_point = T_ej * current_point.raw_point;
//    current_point.world_point = T_j * current_point.raw_point;
}


void iterCurrentPoint3D(Point3D& current_point, int iter_num)
{
    Sophus::SE3d T_begin(last_state.rot_end, last_state.pos_end);
    Sophus::SE3d T_end(state.rot_end, state.pos_end);

    if (iter_num == 0  || iter_num == 18  )
    {
        double offset_time = current_dt - (current_end_time - current_point.relative_time) / 1000.0;
        double scale = offset_time / current_dt;
        scale = scale >= 1 ? 1 : scale <= 0 ? 0 : scale;
        // T_wj = inter(T_wb, T_we)   T_ej = T_ew * T_wj   P_e = T_ej * P_j  P_w = T_wj * P_j
        Sophus::SE3d T_j = Sophus::interpolate(T_begin, T_end, scale);
        Sophus::SE3d T_ej = T_end.inverse() * T_j;
        current_point.undistort_lidar_point = T_ej * current_point.raw_point;
        current_point.world_point = T_j * current_point.raw_point;
    }
    else
    {
        current_point.world_point = T_end * current_point.undistort_lidar_point;
    }
}



void iterCurrentPoint3D(Point3D& current_point, bool converge)
{
    Sophus::SE3d T_begin(last_state.rot_end, last_state.pos_end);
    Sophus::SE3d T_end(state.rot_end, state.pos_end);

    if (converge)
    {
        double offset_time = current_dt - (current_end_time - current_point.relative_time) / 1000.0;
        double scale = offset_time / current_dt;
        scale = scale >= 1 ? 1 : scale <= 0 ? 0 : scale;
        // T_wj = inter(T_wb, T_we)   T_ej = T_ew * T_wj   P_e = T_ej * P_j  P_w = T_wj * P_j
        Sophus::SE3d T_j = Sophus::interpolate(T_begin, T_end, scale);
        Sophus::SE3d T_ej = T_end.inverse() * T_j;
        current_point.undistort_lidar_point = T_ej * current_point.raw_point;
        current_point.world_point = T_j * current_point.raw_point;
    }
    else
    {
        current_point.world_point = T_end * current_point.undistort_lidar_point;
    }
}

int process_increments = 0;


void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_points_size);
    PointNoNeedDownsample.reserve(feats_points_size);
    for (int i = 0; i < feats_points_size; i++)
    {
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(point3DtoPCLPoint(feats_points[i], WORLD).x / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.y = floor(point3DtoPCLPoint(feats_points[i], WORLD).y / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.z = floor(point3DtoPCLPoint(feats_points[i], WORLD).z / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            float dist = calc_dist(point3DtoPCLPoint(feats_points[i], WORLD), mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(point3DtoPCLPoint(feats_points[i], WORLD));
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(point3DtoPCLPoint(feats_points[i], WORLD));
        }
        else
        {
            PointToAdd.push_back(point3DtoPCLPoint(feats_points[i], WORLD));
        }
    }

    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes) {
    if (scan_pub_en)
    {

        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_down_body : feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB(size, 1));
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            if (lidar_type == L515)
                RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorldRGB->points[i]);
            else
                pointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        if (lidar_type == L515)
            pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
        else
            pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }


    /**************** save map ****************/
    /* 1. make sure you have enough memories
       2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        boost::filesystem::create_directories(root_dir + "/PCD");
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++) {
            pointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
        }

        *pcl_wait_save += *laserCloudWorld;
        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            all_points_dir = string(root_dir + "/PCD/PCD") + to_string(pcd_index) + string(".pcd");
            cout << "current scan saved to " << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher &pubLaserCloudFullRes_body) {
    PointCloudXYZI::Ptr laserCloudFullRes(feats_undistort);
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_undistort, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes_body.publish(laserCloudmsg);
}

void publish_effect_world(const ros::Publisher &pubLaserCloudEffect) {
    PointCloudXYZI::Ptr laserCloudWorld(\
                    new PointCloudXYZI(effect_feat_num, 1));
    for (int i = 0; i < effect_feat_num; i++) {
        pointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher &pubLaserCloudMap) {
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T &out)
{
    if (!imu_en) {
        out.position.x = state.pos_end(0);
        out.position.y = state.pos_end(1);
        out.position.z = state.pos_end(2);
    } else {
        //Pubulish LiDAR's pose and position
        V3D pos_cur_lidar = state.rot_end * state.offset_T_L_I + state.pos_end;
        out.position.x = pos_cur_lidar(0);
        out.position.y = pos_cur_lidar(1);
        out.position.z = pos_cur_lidar(2);
    }
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped) {
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose.pose);

    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped"));
}

void publish_mavros(const ros::Publisher &mavros_pose_publisher) {
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);

    msg_body_pose.header.frame_id = "camera_odom_frame";
    set_posestamp(msg_body_pose.pose);
    mavros_pose_publisher.publish(msg_body_pose);
}

void publish_path(const ros::Publisher pubPath) {
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}

void fileout_calib_result() {
    fout_result.setf(ios::fixed);
    fout_result << setprecision(6)
                << "Rotation LiDAR to IMU (degree)     = " << RotMtoEuler(state.offset_R_L_I).transpose() * 57.3
                << endl;
    fout_result << "Translation LiDAR to IMU (meter)   = " << state.offset_T_L_I.transpose() << endl;
    fout_result << "Time Lag IMU to LiDAR (second)     = " << time_lag_IMU_wtr_lidar + timediff_imu_wrt_lidar << endl;
    fout_result << "Bias of Gyroscope  (rad/s)         = " << state.bias_g.transpose() << endl;
    fout_result << "Bias of Accelerometer (meters/s^2) = " << state.bias_a.transpose() << endl;
    fout_result << "Gravity in World Frame(meters/s^2) = " << state.gravity.transpose() << endl << endl;

    MD(4, 4) Transform;
    Transform.setIdentity();
    Transform.block<3, 3>(0, 0) = state.offset_R_L_I;
    Transform.block<3, 1>(0, 3) = state.offset_T_L_I;
    fout_result << "Homogeneous Transformation Matrix from LiDAR to IMU: " << endl;
    fout_result << Transform << endl << endl << endl;
}

void print_refine_result() {
    cout.setf(ios::fixed);
    cout << endl;
    printf(BOLDGREEN "[Final Result] " RESET);
    cout << setprecision(6)
         << "Rotation LiDAR to IMU    = " << RotMtoEuler(state.offset_R_L_I).transpose() * 57.3 << " deg" << endl;
    printf(BOLDGREEN "[Final Result] " RESET);
    cout << "Translation LiDAR to IMU = " << state.offset_T_L_I.transpose() << " m" << endl;
    printf(BOLDGREEN "[Final Result] " RESET);
    printf("Time Lag IMU to LiDAR    = %.8lf s \n", time_lag_IMU_wtr_lidar + timediff_imu_wrt_lidar);
    printf(BOLDGREEN "[Final Result] " RESET);
    cout << "Bias of Gyroscope        = " << state.bias_g.transpose() << " rad/s" << endl;
    printf(BOLDGREEN "[Final Result] " RESET);
    cout << "Bias of Accelerometer    = " << state.bias_a.transpose() << " m/s^2" << endl;
    printf(BOLDGREEN "[Final Result] " RESET);
    cout << "Gravity in World Frame   = " << state.gravity.transpose() << " m/s^2" << endl;
}

void printProgress(double percentage) {
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\033[1A\r");
    printf(BOLDMAGENTA "[Refinement] ");
    if (percentage < 1) {
        printf(BOLDYELLOW "Online Refinement: ");
        printf(YELLOW "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
    } else {
        printf(BOLDGREEN " Online Refinement ");
        printf(GREEN "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
    }
    fflush(stdout);
}


void subSampleFrame(std::vector<Point3D>& frame, double size_voxel)
{
    std::tr1::unordered_map<voxel, std::vector<Point3D>, std::hash<voxel>> grid;
    for (int i = 0; i < (int) frame.size(); i++)
    {
        auto kx = static_cast<short>(frame[i].raw_point[0] / size_voxel);
        auto ky = static_cast<short>(frame[i].raw_point[1] / size_voxel);
        auto kz = static_cast<short>(frame[i].raw_point[2] / size_voxel);
        grid[voxel(kx, ky, kz)].push_back(frame[i]);
    }
    frame.resize(0);
    int step = 0;
    for(const auto &n: grid)
    {
        if(n.second.size() > 0)
        {
            frame.push_back(n.second[0]);
            step++;
        }
    }
}

void adjustCVCov()
{
    auto delta_state = state - state_propagat;

    double delta_v = delta_state.block<3, 1>(12, 0).norm() ;
    double delta_omega = delta_state.block<3, 1>(15, 0).norm() ;

    double cov_v = K_cov / (1 + exp(-1 * a_cov * delta_v + b_cov)) + 0.01;
    double cov_omega = K_cov / (1 + exp(-1 * a_cov * delta_omega + b_cov)) +  0.01;
    V3D new_cov_v(cov_v, cov_v, cov_v);
    V3D new_cov_omega(cov_omega, cov_omega, cov_omega);

    p_imu->Reforward_propagation_without_imu(last_state, state, new_cov_v, new_cov_omega);

//    cout << " current delta_t = " << delta_v << " scale = " << delta_v << " acc_cov = " << cov_v << endl;
//    cout << " current delta_R = " << delta_omega << " scale = " << delta_omega << " gyr_cov = " << cov_omega << endl;
//    cout << endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 10);
    nh.param<int>("mapping/max_undistort", NUM_MAX_UNDISTORT, 3);
    nh.param<bool>("adaptive_cov/use", adaptive_cov, false);
    nh.param<double>("adaptive_cov/K", K_cov, 1);
    nh.param<double>("adaptive_cov/a", a_cov, 1.5);
    nh.param<double>("adaptive_cov/b", b_cov, 5.0);
    nh.param<int>("initialization/cut_frame_init_num", p_pre->cut_frame_init_num, 5);
    nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 2);
    nh.param<string>("map_file_path", map_file_path, "");
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<double>("mapping/filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("mapping/filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("mapping/cube_side_length", cube_len, 200);
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("preprocess/blind", p_pre->blind, 1.0);
    nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<bool>("preprocess/feature_extract_en", p_pre->feature_enabled, 0);
    nh.param<bool>("initialization/cut_frame", cut_frame, true);
    nh.param<int>("initialization/cut_frame_num", cut_frame_num, 1);
    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, 1);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, 1);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, 1);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<double>("mapping/cov_lidar", cov_lidar, 0.001);

    cout << "lidar_type: " << lidar_type << endl;

    path.header.stamp = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id = "camera_init";

    /*** variables definition ***/
    VD(DIM_STATE) solution;
    MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
    V3D rot_add, T_add, vel_add, gyr_add;

    PointType pointOri, pointSel, coeff;

    double deltaT, deltaR;
    bool flg_EKF_converged = 0, EKF_stop_flg = 0;

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));

    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    last_state = state;

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
         nh.subscribe(lid_topic, 200000, standard_pcl_cbk);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
            ("/aft_mapped_to_init", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>
            ("/path", 100000);

//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    int frame_id = 0;
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        if (sync_packages(Measures))
        {
            frame_id++;
            if (flg_reset)
            {
                ROS_WARN("reset when rosbag play back.");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }


            feats_points = Measures.points;
            feats_points_full = Measures.points;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                ROS_WARN("First frame, no points stored.");
            }

            p_imu->Process(Measures, state, feats_undistort);

            current_dt = p_imu->frame_dt;
            current_end_time = p_imu->frame_end_time;

            state_propagat = state;


            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();


            boost::mt19937_64 seed;
            std::shuffle(feats_points.begin(), feats_points.end(), seed);
            subSampleFrame(feats_points, filter_size_surf_min);
            std::shuffle(feats_points.begin(), feats_points.end(), seed);


            feats_points_size = feats_points.size();

            feats_points_full_size = feats_points_full.size();

            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr )
            {
                if (feats_points_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_points_full_size);
                    for (int i = 0; i < feats_points_full_size; i++)
                    {
                        feats_down_world->points[i] = point3DtoPCLPoint(feats_points_full[i], RAW);
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }

            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_points_size);
            feats_down_world->resize(feats_points_size);
            euler_cur = RotMtoEuler(state.rot_end);

            pointSearchInd_surf.resize(feats_points_size);
            Nearest_Points.resize(feats_points_size);
            int rematch_num = 0;
            bool nearest_search_en = true;

            /*** iterated state estimation ***/
            std::vector<M3D> body_var;
            std::vector<M3D> crossmat_list;
            body_var.reserve(feats_points_size);
            crossmat_list.reserve(feats_points_size);


            double knn_time = 0;
            double solve_time = 0;
            int knn_count = 0;
            int iter_count = 0;
            StatesGroup last_converge_state = state;

            bool current_converge = true;
            int current_iter_count = 0;

            distort_time = 0;

            for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++)
            {
                laserCloudOri->clear();
                corr_normvect->clear();
                total_residual = 0.0;

                iter_count++;

                if (nearest_search_en)
                    knn_count++;

                if (current_converge)
                    distort_time++;

                /** closest surface search and residual computation **/
                #ifdef MP_EN
                    omp_set_num_threads(MP_PROC_NUM);
                    #pragma omp parallel for
                #endif
                for (int i = 0; i < feats_points_size; i++)
                {
                    iterCurrentPoint3D(feats_points[i], current_converge);
                    PointType point_body = point3DtoPCLPoint(feats_points[i], UNDISTORT);
                    PointType point_world = point3DtoPCLPoint(feats_points[i], WORLD);

                    V3D p_body(point_body.x, point_body.y, point_body.z);

                    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                    auto &points_near = Nearest_Points[i];
                    uint8_t search_flag = 0;

                    if (nearest_search_en)
                    {
                        /** Find the closest surfaces in the map **/
                        ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, 5);
                        if (points_near.size() < NUM_MATCH_POINTS)
                            point_selected_surf[i] = false;
                        else
                            point_selected_surf[i] = !(pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5);
                    }

                    res_last[i] = -1000.0f;

                    if (!point_selected_surf[i] || points_near.size() < NUM_MATCH_POINTS)
                    {
                        point_selected_surf[i] = false;
                        continue;
                    }

                    point_selected_surf[i] = false;
                    VD(4) pabcd;
                    pabcd.setZero();
                    if (esti_plane(pabcd, points_near, 0.1)) //(planeValid)
                    {
                        float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z +
                                    pabcd(3);
                        float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                        if (s > 0.9) {
                            point_selected_surf[i] = true;
                            normvec->points[i].x = pabcd(0);
                            normvec->points[i].y = pabcd(1);
                            normvec->points[i].z = pabcd(2);
                            normvec->points[i].intensity = pd2;
                            res_last[i] = abs(pd2);
                        }
                    }
                }

                effect_feat_num = 0;
                for (int i = 0; i < feats_points_size; i++)
                {
                    if (point_selected_surf[i])
                    {
                        laserCloudOri->points[effect_feat_num] = point3DtoPCLPoint(feats_points[i], UNDISTORT);
                        corr_normvect->points[effect_feat_num] = normvec->points[i];
                        effect_feat_num++;
                    }
                }

                res_mean_last = total_residual / effect_feat_num;

                /*** Computation of Measurement Jacobian matrix H and measurents vector ***/

                MatrixXd Hsub(effect_feat_num, 12);
                MatrixXd Hsub_T_R_inv(12, effect_feat_num);
                VectorXd R_inv(effect_feat_num);
                VectorXd meas_vec(effect_feat_num);

                Hsub.setZero();
                Hsub_T_R_inv.setZero();
                meas_vec.setZero();


                for (int i = 0; i < effect_feat_num; i++)
                {
                    const PointType &laser_p = laserCloudOri->points[i];
                    V3D point_this_L(laser_p.x, laser_p.y, laser_p.z);

                    V3D point_this = state.offset_R_L_I * point_this_L + state.offset_T_L_I;
                    M3D var;
                    calcBodyVar(point_this, 0.02, 0.05, var);
                    var = state.rot_end * var * state.rot_end.transpose();
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);

                    /*** get the normal vector of closest surface/corner ***/
                    const PointType &norm_p = corr_normvect->points[i];
                    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

                    R_inv(i) = 1000;
                    laserCloudOri->points[i].intensity = sqrt(R_inv(i));

                    /*** calculate the Measurement Jacobian matrix H ***/
                    V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
                    Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z, 0, 0, 0, 0, 0, 0;

                    Hsub_T_R_inv.col(i) = Hsub.row(i).transpose() / cov_lidar;
                    /*** Measurement: distance to the closest surface/corner ***/
                    meas_vec(i) = -norm_p.intensity;
                }

                MatrixXd K(DIM_STATE, effect_feat_num);

                EKF_stop_flg = false;
                flg_EKF_converged = false;
                current_converge = false;
                /*** Iterative Kalman Filter Update ***/

                H_T_H.block<12, 12>(0, 0) = Hsub_T_R_inv * Hsub;
                MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + state.cov.inverse()).inverse();
                K = K_1.block<DIM_STATE, 12>(0, 0) * Hsub_T_R_inv;
                auto vec = state_propagat - state;
                solution = K * meas_vec + vec - K * Hsub * vec.block<12, 1>(0, 0);

                //state update
                state += solution;

                rot_add = solution.block<3, 1>(0, 0);
                T_add = solution.block<3, 1>(3, 0);

                if ((rot_add.norm() * 57.3 < 0.01) && (T_add.norm() * 100 < 0.015))
                    flg_EKF_converged = true;

                deltaR = rot_add.norm() * 57.3;
                deltaT = T_add.norm() * 100;

                euler_cur = RotMtoEuler(state.rot_end);

                current_iter_count++;
                /*** Rematch Judgement ***/
                nearest_search_en = false;
                if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2))))
                {
                    nearest_search_en = true;
                    rematch_num++;

                    auto delta_state = state - last_converge_state;

                    if (((delta_state.block<3, 1>(0, 0).norm() * 57.3 < 1) && (delta_state.block<3, 1>(3, 0).norm() * 100 < 1.0) && iterCount >= 4)
                            || current_iter_count >= 15)
                    {
                        current_converge = true;
                        current_iter_count = 0;
                        if (distort_time >= NUM_MAX_UNDISTORT)
                        {
                            EKF_stop_flg = true;
                        }
                    }

                    last_converge_state = state;
                }

                /*** Adjust const velocity model cov ***/
                if (iterCount == 1 && adaptive_cov) adjustCVCov();


                /*** Convergence Judgements and Covariance Update ***/
                if (EKF_stop_flg || ((iterCount == NUM_MAX_ITERATIONS - 1)) )
                {
                    if (flg_EKF_inited)
                    {
                        /*** Covariance Update ***/
                        G.setZero();
                        G.block<DIM_STATE, 12>(0, 0) = K * Hsub;
                        state.cov = (I_STATE - G) * state.cov;
                        total_distance += (state.pos_end - position_last).norm();
                        position_last = state.pos_end;
                        M3D rot_cur_lidar = state.rot_end * state.offset_R_L_I;
                        V3D euler_cur_lidar = RotMtoEuler(rot_cur_lidar);
                        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                (euler_cur_lidar(0), euler_cur_lidar(1), euler_cur_lidar(2));
                        VD(DIM_STATE) K_sum = K.rowwise().sum();
                        VD(DIM_STATE) P_diag = state.cov.diagonal();
                    }
                    EKF_stop_flg = true;
                }

                if (EKF_stop_flg) break;
            }

            *feats_down_body = point3DtoPCL(feats_points, UNDISTORT);

            last_state = state;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            map_incremental();
            kdtree_size_end = ikdtree.size();

            /******* Publish points *******/
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFullRes);
            last_odom = state.pos_end;
            last_rot = state.rot_end;
            publish_effect_world(pubLaserCloudEffect);
            if (path_en) publish_path(pubPath);


            frame_num++;


        }
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
