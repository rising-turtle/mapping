/*
 * April 1, 2020, He Zhang, hzhang8@vcu.edu 
 *
 * Generate ply traj: 
 *     input:  a trajectory.log 
 *     output: a .ply file
 *
 * */

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <vector>
#include <stdio.h>

using namespace std; 

#define D2R(d) (((d)*M_PI)/180)

struct trajItem
{
  long long timestamp; // sequence id 
  // double timestamp; 
  // string timestamp; 
  double px, py, pz; // euler angle roll, pitch, yaw; 
  double qx, qy, qz, qw; // quaternion
};

void headerPLY(std::ofstream& ouf, int vertex_number);
bool readTraj(std::string f, vector<struct trajItem>& );
bool mapPLY(std::string f, std::string outPLY);

int main(int argc, char* argv[])
{
  cout<<"usage: ./mapping_bag_ply [traj] [out_ply]"<<endl; 

  // parameters 
  int skip = 4; 
  int t_skip = 1; 
  string trajFile("/home/hzhang8/work/result/room8.csv");  // the trajectory log file 
  string outPLY("/home/hzhang8/work/result/room8.ply");    // the output PLY file  

  if(argc >= 2){
    trajFile = string(argv[1]); 
    if(argc >= 3)
      outPLY = string(argv[2]); 
  }

  mapPLY(trajFile, outPLY);

  printf("Finish convert trajectory into %s!\n", outPLY.c_str());

  return 0; 
}

bool mapPLY(std::string f, std::string outPLY)
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
  
  printf("succeed to load %d pose \n", v_traj.size()); 

  // vector<float> pts_loc(v_traj.size()*3);
  headerPLY(ouf, v_traj.size()); 

  // TODO: make the color as an input parameter 
  for(int i=0; i<v_traj.size(); i++){
    // pts_loc[i*3] = v_traj[i].px; 
    // pts_loc[i*3+1] = v_traj[i].py; 
    // pts_loc[i*3+2] = v_traj[i].pz; 
    ouf<< v_traj[i].px <<" "<<v_traj[i].py<<" "<<v_traj[i].pz<<" "<<0<<" "<<0<<" "<<0<<endl; 
  }
  ouf.flush(); 
  ouf.close(); 
  return ; 
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


