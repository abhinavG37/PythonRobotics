//
// Created by abhinav137 on 29/05/21.
//

#include <iostream>
#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
using namespace std;


std::tuple<float, int> nearest_neighbor_association( Eigen::MatrixXd previous_points, Eigen::MatrixXd current_points){

  Eigen::MatrixXd delta = previous_points - current_points;
  Eigen::MatrixXd d = delta.rowwise().norm();
  float error = d.sum(); //sum of squared errors in between two point sets
  int new_rows = previous_points.rows() * previous_points.rows();
  Eigen::MatrixXd tiled = current_points.colwise().replicate(previous_points.rows());
  Eigen::MatrixXd repeated = Eigen::MatrixXd::Zero(new_rows, 2);

  for (int row_val = 0; row_val < new_rows; row_val++) {
    repeated(row_val, 0) = previous_points(static_cast<int>(row_val / previous_points.rows()), 0);
    repeated(row_val, 1) = previous_points(static_cast<int>(row_val / previous_points.rows()), 1);
  }
  cout << tiled.rows()<<tiled.cols()<<"\n\n\n";
  cout<<tiled<<"\n\n\n";
  cout<<repeated.rows()<<repeated.cols()<<"\n\n\n";
  cout<<repeated<<"\n\n\n";

  d = (repeated-tiled).rowwise().norm();
  cout<<"SHAPE OF d:\t"<<d.rows()<<"\t"<<d.cols()<<endl;
  cout<<"d:\t\n"<<d<<endl;
  printf("Reshaping\n");

  Eigen::MatrixXd d1 = d.reshaped(previous_points.rows(), current_points.rows());
  cout<<"D1:\n"<<d1<<endl;
  cout<<"DONE"<<endl;

//  Eigen::VectorXi min_index = Eigen::VectorXi::Zero(d1.rows());
//  for (int i = 0; i <d1.rows() ; i++) {
//    min_index[i] = 0;
//    for (int j = 0; j < d1.cols(); j++) {
//      if (d1(i, j) <= d1(i, min_index[i])) {
//        min_index[i] = j;
//      }
//    }
//  }
  std::vector<float> min_index(d1.rows(),0);
  for (int i = 0; i <d1.rows() ; i++) {
    for (int j = 0; j < d1.cols(); j++) {
      if (d1(i, j) <= d1(i, static_cast<int>(static_cast<int>(min_index[i])))) {
        min_index[i] = j;
      }
    }
  }



  return std::make_tuple(1,2);
}

//
//void svd_motion_estimation(int num_points, const Eigen::MatrixXd& previous_points, const Eigen::MatrixXd&  current_points) {
//  Eigen::Vector<float,2> prev_mean = previous_points.colwise().mean();
//  Eigen::Vector<float,2> curr_mean = current_points.colwise().mean();
//  Eigen::MatrixXf temp             = prev_mean.colwise().replicate(previous_points.rows());
//  Eigen::MatrixXf prev_shift       = previous_points - temp;
//
//
//}



int main(){
  int npoint = 5;
  float fieldLength = 50.0;
  Eigen::Vector3f motion(0.5,2.0,M_PI/8.0);

//  Eigen::MatrixXd previous_points = Eigen::MatrixXd::Random(npoint,2);
//  previous_points+=   Eigen::MatrixXd::Constant(npoint,2,1);
//  previous_points/=2;
//  previous_points=previous_points-Eigen::MatrixXd::Constant(npoint,2,0.5);
//  previous_points*=fieldLength;
//
//  cout<< previous_points<<endl;
//
//
//  Eigen::MatrixXd current_points = previous_points;
//  double prev_x, prev_y;
//  for(int row = 0;row<npoint; row++){
//    prev_x= previous_points(row,0), prev_y = previous_points(row,1);
//    current_points(row, 0) = prev_x*cos(motion(2)) -
//                             sin(motion(2))*prev_y+ motion(0);
//    current_points(row, 1) = prev_x*cos(motion(2)) -
//                             sin(motion(2))*prev_y+ motion(1);
//  }
//
//
//
//  nearest_neighbor_association(previous_points, current_points);
  Eigen::MatrixXd H = Eigen::MatrixXd::Random(4,4);
  Eigen::MatrixXd R = H(Eigen::seq(0,H.rows()-2),Eigen::seq(0,H.cols()-2));
  Eigen::MatrixXd T = H(Eigen::seq(0,H.rows()-2),H.cols()-1);
  cout<<H<<endl;
  cout<<R<<endl;
  cout<<T<<endl;
  return 0;
}