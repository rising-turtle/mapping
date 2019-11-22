/*
 * Nov. 19, 2017, He Zhang, hzhang8@vcu.edu 
 *
 * Generate 3D map: 
 *     input:  a trajectory.log and a bag file where the images are stored 
 *     output: a .ply file, shown the 3d point cloud 
 *
 * */

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
// #include "std_msgs/Float32MultiArray.h"
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std; 
using namespace cv; 

#define D2R(d) (((d)*M_PI)/180)

string fname("/home/david/work/data/sr4k/imu_bdat/etas_f5/imu_v100.log"); 

struct trajItem
{
  long long timestamp; // sequence id 
  // double timestamp; 
  // string timestamp; 
  double px, py, pz; // euler angle roll, pitch, yaw; 
  double qx, qy, qz, qw; // quaternion
};

// TODO: make it as a parameter
Eigen::Matrix<float, 4, 4> Tu2c; //  = Eigen::Matrix<float, 4, 4>::Identity(); // specify it 
double CX, CY, FX, FY; 

// void setTu2c(Pose3& Tu2c); 
void readParameters(); 
void headerPLY(std::ofstream& ouf, int vertex_number);
bool mapPLY(std::string f, std::string img_bag, std::string outPLY, int skip, int* rect = 0);
void generatePointCloud(cv::Mat& rgb, cv::Mat& depth, int skip, vector<float>& pts, vector<unsigned char>& color, int * rect);
bool readTraj(std::string f, vector<struct trajItem>& );
// void LoadImages(string fAsso, vector<string> & vRGB, vector<string>& vDPT, vector<double>& vTime); 
bool findIt(double t, vector<double>& vTime); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mapping_bag_ply"); 
  ros::NodeHandle n; 
  // euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); \

  cout<<"usage: ./mapping_bag_ply [traj] [bag] [out_ply]"<<endl; 

  // parameters 
  ros::NodeHandle np("~"); 
  int skip = 4; 
  int t_skip = 1; 
  string trajFile("/home/hzhang8/work/result/room8.csv");  // the trajectory log file 
  string bagFile("/home/hzhang8/work/data/sturct_core_v2/room8.bag");    // images dir 
  string outPLY("/home/hzhang8/work/result/room8.ply");    // the output PLY file 
  

  if(argc >= 2){
    trajFile = string(argv[1]); 
    if(argc >= 3){
      bagFile = string(argv[2]); 
      if(argc >= 4)
        outPLY = string(argv[3]);
    }
  }

  int su, sv, eu, ev ; 

  np.param("trajectory_file", trajFile, trajFile); 
  np.param("bag_file", bagFile, bagFile); 
  np.param("output_PLY_file", outPLY, outPLY); 
  np.param("downsample_skip", skip, skip); 
  // np.param("trajectory_skip", t_skip, t_skip); 
  np.param("top_left_u", su, 0); 
  np.param("top_left_v", sv, 0); 
  np.param("bot_right_u", eu, 640); 
  np.param("bot_right_v", ev, 480); 

  readParameters();
  
  int rect[4] = {su, sv, eu, ev}; 

  cout <<"rect "<<rect[0]<<" "<<rect[1]<<" "<<rect[2]<<" "<<rect[3]<<endl;

  mapPLY(trajFile, bagFile, outPLY, skip, &rect[0]);

  ROS_INFO("Finish mapping into %s!", outPLY.c_str());

  return 0; 
}

void handle_rgbd(cv::Mat& rgb, cv::Mat& dpt, trajItem& pj, vector<float>& pts_loc, vector<unsigned char>& pts_col, int skip, int* rect ){

    static int nxt = 0; 
    if(nxt++ & 0x01)
      return; 

    vector<float> p_loc; // location 
    vector<unsigned char> p_col; // color 

    generatePointCloud(rgb, dpt, skip, p_loc, p_col, rect); 

    Eigen::Quaternionf q(pj.qw, pj.qx, pj.qy, pj.qz); 

    Eigen::Matrix4f Tg2u = Eigen::Matrix4f::Identity(); 
    Tg2u.block<3,3>(0,0) = q.toRotationMatrix(); 
    Tg2u(0,3) = pj.px; Tg2u(1,3) = pj.py; Tg2u(2,3) = pj.pz; 

    // transform into global 
    Eigen::Vector4f fo, u_fo, to; 
    for(int i=0; i<p_loc.size(); i+=3)
    {
      fo << p_loc[i], p_loc[i+1], p_loc[i+2], 1.0; 
      // to << p_loc[i], p_loc[i+1], p_loc[i+2]; 
      // to_imu = Tu2c.transform_from(fo);
      // to = p.transform_from(fo); 
      u_fo = Tu2c * fo;

      // from imu to global 
      to = Tg2u * u_fo; 

      p_loc[i] = to(0); p_loc[i+1] = to(1); p_loc[i+2] = to(2); 
    }
    
    pts_loc.insert(pts_loc.end(), p_loc.begin(), p_loc.end()); 
    pts_col.insert(pts_col.end(), p_col.begin(), p_col.end());

    // ouf<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<g_color[c][0]<<" "<<g_color[c][1]<<" "<<g_color[c][2]<<endl;
    // ROS_INFO("handle frame at timestamp %lf, add %d pts", vTimes[ti.sid-1], p_loc.size());
}

void readParameters(){

  // TODO: use config file set parameters 
  Tu2c << 0.00193013, -0.999997, 0.00115338, -0.00817048,
      -0.999996, -0.0019327, -0.00223606, 0.015075,
      0.00223829, -0.00114906, -0.999997, -0.0110795,
      0, 0, 0, 1;

  CX = 332.695; 
  CY = 258.998;
  FX = 459.357;
  FY = 459.764;

}

void processBagFile(string bagfile, vector<trajItem>& v_traj, vector<float>& pts_loc, vector<unsigned char>& pts_col, int skip, int* rect)
{
  std::vector<std::string> topics;
  string rgb_tpc = "/cam0/color"; // "/cam0/image_raw";   
  string dpt_tpc = "/cam0/depth"; 
  string imu_tpc = "/imu0";
  topics.push_back(rgb_tpc); 
  topics.push_back(dpt_tpc); 
  topics.push_back(imu_tpc); 

  rosbag::Bag bag; 
  bag.open(bagfile, rosbag::bagmode::Read); 
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // for extract cv::mat 
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;

  int n_cnt = 0; 

  cv::Mat rgb_mat;
  cv::Mat dpt_mat; 

  bool rgb_ready = false; 
  bool dpt_ready = false; 

  BOOST_FOREACH(rosbag::MessageInstance const m, view){

    if(m.getTopic() == rgb_tpc){
      sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();

      long long timestamp = simage->header.stamp.toSec()*1e9; 

      auto it = find_if(v_traj.begin(), v_traj.end(), [timestamp](const trajItem& it)
        {
          return it.timestamp == timestamp;
        });
      if(it!= v_traj.end()){
        // find it 

        if(simage->encoding=="8UC3"){

          sensor_msgs::Image img;
          img.header = simage->header;
          img.height = simage->height;
          img.width = simage->width;
          img.is_bigendian = simage->is_bigendian;
          img.step = simage->step;
          img.data = simage->data;
          img.encoding = "bgr8";
          cv_ptrRGB = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
          rgb_mat = cv_ptrRGB->image.clone();   
        }else{
          cv_ptrRGB = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::BGR8); 
          rgb_mat = cv_ptrRGB->image;
        }
        rgb_ready = true; 
        if(dpt_ready){
          // handle it 
          handle_rgbd(rgb_mat, dpt_mat, (*it), pts_loc, pts_col,skip, rect); 

          dpt_ready = false; 
          rgb_ready = false; 
        }
        // if(++n_cnt < 50)
        //  printf("mapping_bag_ply.cpp: receive rgb image at %ld \n", timestamp); 
      }

  
    }else if(m.getTopic() == dpt_tpc){
      sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
      
      long long timestamp = simage->header.stamp.toSec()*1e9; 

      auto it = find_if(v_traj.begin(), v_traj.end(), [timestamp](const trajItem& it)
        {
          return it.timestamp == timestamp;
        });
      if(it!= v_traj.end()){
        // find it 
        cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1); 
        dpt_mat = cv_ptrD->image; 
        dpt_ready = true; 
        if(rgb_ready){
          // handle it 
          handle_rgbd(rgb_mat, dpt_mat, (*it), pts_loc, pts_col,skip, rect); 

          dpt_ready = false; 
          rgb_ready = false; 
        }

      if(n_cnt < 50)
        printf("mapping_bag_ply.cpp: receive dpt image at %ld \n", timestamp); 
      }

    
    }

    if(n_cnt> 50)
      break;
  }

}


bool mapPLY(std::string f, std::string img_bag, std::string outPLY, int skip, int* rect)
{
  // output file 
  ofstream ouf(outPLY.c_str()); 
  if(!ouf.is_open())
  {
    cout <<" mapping_bag_ply.cpp: failed to open output PLY file : "<<outPLY<<endl; 
    return false; 
  }

  // generate a global point cloud 
  vector<trajItem> v_traj; 
  if(!readTraj(f, v_traj))
  {
    return false; 
  }
  
  ROS_INFO("succeed to load %d pose ", v_traj.size()); 


  vector<float> pts_loc; 
  vector<unsigned char> pts_col;

  // handle bag file
  processBagFile(img_bag, v_traj, pts_loc, pts_col, skip, rect); 

  // output into file, first, add header 
  int vertex_number = pts_loc.size()/3; 
  headerPLY(ouf, vertex_number);

  for(int i=0; i<pts_loc.size(); i+=3)
  {
    ouf<< pts_loc[i]<<" "<<pts_loc[i+1]<<" "<<pts_loc[i+2]<<" "<<(int)pts_col[i]<<" "<<(int)pts_col[i+1]<<" "<<(int)pts_col[i+2]<<endl;
  }
  ouf.flush(); 
  ouf.close();

  /*
  vector<string> vRGBs; vector<string> vDPTs; vector<double> vTimes; 
  LoadImages(record_file, vRGBs, vDPTs, vTimes); 

  // transform point from camera to imu 
  // Pose3 Tu2c ; 
  // setTu2c(Tu2c); 

  for(int i=0; i<t.size(); i+= t_skip)
  {
    trajItem& ti = t[i]; 

    // construct pose3
    // Rot3 R = Rot3::RzRyRx(ti.roll, ti.pitch, ti.yaw); 
    
    // This is a bug in GTSAM, where the correct sequence is R(w, x, y, z) not R(x, y, z, w)
    Rot3 R(ti.qw, ti.qx, ti.qy, ti.qz); 
    // Rot3 R(ti.qx, ti.qy, ti.qz, ti.qw); 
    Point3 t; t << ti.px, ti.py, ti.pz; 
    Pose3 p = Pose3(R, t); 

    // get image 
    stringstream ss_rgb, ss_dpt; 
    // ss<<img_dir<<"/d1_"<<setfill('0')<<setw(7)<<ti.sid<<".bdat";  // setw(4)
    // r4k.readOneFrameCV(ss.str(), i_img, d_img);

    // read image 
    // ss_rgb << img_dir <<"/color/"<<ti.timestamp<<".png"; 
    // ss_dpt << img_dir <<"/depth/"<<ti.timestamp<<".png"; 
    int index = ti.sid - 1; 
    ss_rgb << img_dir <<"/"<<vRGBs[index]; 
    ss_dpt << img_dir <<"/"<<vDPTs[index]; 

    i_img = cv::imread(ss_rgb.str().c_str(), -1); 
    d_img = cv::imread(ss_dpt.str().c_str(), -1); 
    if(i_img.data == NULL || d_img.data == NULL)
    {
      ROS_ERROR("failed to load camera data at dir %s with timestamp = %lf", img_dir.c_str(), vTimes[ti.sid-1]); 
      ROS_WARN("ss_rgb = %s ss_dpt = %s", ss_rgb.str().c_str(), ss_dpt.str().c_str());
      break; 
    }

    cv::medianBlur(d_img, d_filter, 5);  // median filter for depth data 
    
    // compute local point cloud 
    vector<float> p_loc; 
    vector<unsigned char> p_col; 
    pSF->generatePointCloud(i_img, d_filter, skip, depth_scale, p_loc, p_col, rect); 

    // transform into global 
    Point3 fo, to, to_imu; 
    for(int i=0; i<p_loc.size(); i+=3)
    {
      fo << p_loc[i], p_loc[i+1], p_loc[i+2]; 
      // to << p_loc[i], p_loc[i+1], p_loc[i+2]; 
      // to_imu = Tu2c.transform_from(fo);
      to = p.transform_from(fo); 
      p_loc[i] = to(0); p_loc[i+1] = to(1); p_loc[i+2] = to(2); 
    }
    
    pts_loc.insert(pts_loc.end(), p_loc.begin(), p_loc.end()); 
    pts_col.insert(pts_col.end(), p_col.begin(), p_col.end());

    // ouf<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<g_color[c][0]<<" "<<g_color[c][1]<<" "<<g_color[c][2]<<endl;
    ROS_INFO("handle frame at timestamp %lf, add %d pts", vTimes[ti.sid-1], p_loc.size());
  }
  
  // output into file 

  // first, add header 
  
  int vertex_number = pts_loc.size()/3; 
  headerPLY(ouf, vertex_number);

  for(int i=0; i<pts_loc.size(); i+=3)
  {
    ouf<< pts_loc[i]<<" "<<pts_loc[i+1]<<" "<<pts_loc[i+2]<<" "<<(int)pts_col[i]<<" "<<(int)pts_col[i+1]<<" "<<(int)pts_col[i+2]<<endl;
  }
  ouf.flush(); 
  ouf.close(); */
return true;  
}
  
void headerPLY(std::ofstream& ouf, int vertex_number)
{
  ouf << "ply"<<endl
    <<"format ascii 1.0"<<endl
    <<"element vertex "<<vertex_number<<endl
    <<"property float x"<<endl
    <<"property float y"<<endl
    <<"property float z"<<endl
    <<"property uchar red"<<endl
    <<"property uchar green"<<endl
    <<"property uchar blue"<<endl
    <<"end_header"<<endl;
}


bool readTraj(std::string f, vector<struct trajItem>& t)
{

  ifstream inf(f.c_str()); 
  if(!inf.is_open())
  {
    cout<<"failed to open file : "<<f<<endl;
    return false;
  }

  char buf[4096]; 
  // char b[21]={0}; 
  while(inf.getline(buf, 4096))
  {
    trajItem ti; 
    sscanf(buf, "%ld, %lf, %lf, %lf, %lf, %lf, %lf, %lf", &ti.timestamp, &ti.px, &ti.py, &ti.pz, &ti.qw, &ti.qx, &ti.qy, &ti.qz); 
    // ti.timestamp = string(b); 
    t.push_back(ti); 
    if(t.size() < 10)
    {
      printf("read %ld %f %f %f %f %f %f %f\n", ti.timestamp, ti.px, ti.py, ti.pz, ti.qw, ti.qx, ti.qy, ti.qz);
    }
  }

  cout << " read "<<t.size()<<" trajectory items"<<endl;
  return true;

}
/*
int findIndex(double t, vector<double>& vTime)
{
  for(int i=0; i<vTime.size(); i++)
  {
    if(vTime[i] == t)
      return i; 
    if(vTime[i] > t)
      return -1; 
  }
  return -1; 
}


void LoadImages(string fAsso, vector<string> & vRGB, vector<string>& vDPT, vector<double>& vTime)
{
  ifstream inf(fAsso.c_str()); 
  double t; 
  string sRGB, sDPT; 
  while(!inf.eof())
  {
    string s; 
    getline(inf, s); 
    if(!s.empty())
    {
      stringstream ss; 
      ss << s; 
      ss >> t;
      vTime.push_back(t); 
      ss >> sRGB; 
      vRGB.push_back(sRGB); 
      ss >> t; 
      ss >> sDPT; 
      vDPT.push_back(sDPT); 
    }
  }
  cout <<"mapping_PLY_rs.cpp: load "<<vTime.size()<<" records"<<endl;
}


void setTu2c(Pose3& Tu2c)
{
  //// Body/IMU Coordinate System 
  //          
  //             X
  //            /
  //           /
  //          /_____ Y
  //          |
  //          |
  //        Z | 
  //
  //        
  //   CAMERA Coordinate System    
  //           Z 
  //          /
  //         / 
  //        /  
  //        ------- X
  //        |
  //        |
  //        |Y
  //

  float p = 0; 
  Rot3 R_g2b = Rot3::RzRyRx( M_PI/2., 0. , M_PI/2.); // roll, pitch, yaw, in BODY coordinate system 
  Rot3 R_b2o = Rot3::RzRyRx(p ,0 , 0); 
  Rot3 R_g2o = R_g2b * R_b2o; 
  // Rot3 R_g2o = R_b2o * R_g2b; 

  Point3 t = Point3::Zero(); 
  // (*mp_u2c) = Pose3::Create(R_g2o, t);
  Tu2c = Pose3::Create(R_g2o, t); 
   t<< 1, 3, 2; 
   cout << t<< " after rotation "<<endl <<Tu2c*t<<endl;
   cout << "rotation pitch: "<< R_b2o * t<<endl; 
   cout << "rotation all: "<< R_g2b * R_b2o * t<<endl; 

  return ; 
}
*/

void generatePointCloud(cv::Mat& rgb, cv::Mat& depth, int skip, vector<float>& pts, vector<unsigned char>& color, int * rect)
{
  double z; 
  double px, py, pz; 
  int N = (rgb.rows/skip)*(rgb.cols/skip); 
  pts.reserve(N*3); 
  color.reserve(N*3); 

  unsigned char r, g, b; 
  int pixel_data_size = 3; 
  if(rgb.type() == CV_8UC1)
  {
    pixel_data_size = 1; 
  }
  
  int color_idx; 
  char red_idx = 0, green_idx =1, blue_idx = 2;

  int sv, su, ev, eu; 
  if(rect == 0)
  {
    sv = su = 0; 
    ev = rgb.rows; 
    eu = rgb.cols; 
  }else{
    su = rect[0]; sv = rect[1]; eu = rect[2]; ev = rect[3]; 
    su = su < 0 ? 0 : su;   sv = sv < 0 ? 0 : sv; 
    eu = eu <= rgb.cols ? eu : rgb.cols; ev = ev <= rgb.rows ? ev : rgb.rows;
  }

  for(int v = sv; v<ev; v+=skip)
  for(int u = su; u<eu; u+=skip)
  {
    z = depth.at<unsigned short>((v), (u))*0.001;
    if(std::isnan(z) || z <= 0.0 || z >= 4.0) continue; 
    // m_cam_model.convertUVZ2XYZ(u, v, z, px, py, pz); 
    px = (u - CX) / FX * z; 
    py = (v - CY) / FY * z; 
    pz = z;

    pts.push_back(px);  pts.push_back(py);  pts.push_back(pz); 
    color_idx = (v*rgb.cols + u)*pixel_data_size;
    if(pixel_data_size == 3)
    {
      r = rgb.at<uint8_t>(color_idx + red_idx);
      g = rgb.at<uint8_t>(color_idx + green_idx); 
      b = rgb.at<uint8_t>(color_idx + blue_idx);
    }else{
      r = g = b = rgb.at<uint8_t>(color_idx); 
    }
    color.push_back(r); color.push_back(g); color.push_back(b); 
  }
  return ;
}