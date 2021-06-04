//
// Created by abhinav137 on 27/05/21.
//
#include <iostream>
#include <utility>
#include <string>
#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/src/Core/ArithmeticSequence.h>
#include <cmath>
#include <cstdlib>
#include <tuple>
using namespace std;

#define NPOINT 1000
#define FIELDLENGTH 50.0

#define EPS 0.0001
#define MAX_ITER 100

std::tuple<std::vector<int>, float> nearest_neighbor_association( Eigen::MatrixXd previous_points, Eigen::MatrixXd current_points){

  Eigen::MatrixXd delta = previous_points - current_points;
  Eigen::MatrixXd d     = delta.rowwise().norm();
  cout<<"D"<<endl;
  cout<<d<<endl;
  float error = d.sum(); //sum of squared errors in between two point sets

  int new_rows             = previous_points.rows() * previous_points.rows();
  Eigen::MatrixXd tiled    = current_points.colwise().replicate(previous_points.rows());
  Eigen::MatrixXd repeated = Eigen::MatrixXd::Zero(new_rows, 2);
  for (int row_val = 0; row_val < new_rows; row_val++) {
    repeated(row_val, 0) = previous_points(static_cast<int>(row_val / previous_points.rows()), 0);
    repeated(row_val, 1) = previous_points(static_cast<int>(row_val / previous_points.rows()), 1);
  }
  delta= repeated - tiled;
  d = delta.rowwise().norm();
  cout<<"BREAKPOINT"<<endl;
  Eigen::MatrixXd d1 = d.reshaped(previous_points.rows(), current_points.rows());
  std::vector<int> min_index(d1.rows(),0);
  for (int i = 0; i <d1.rows() ; i++) {
    for (int j = 0; j < d1.cols(); j++) {
      if (d1(i, j) <= d1(i, static_cast<int>(static_cast<int>(min_index[i])))) {
        min_index[i] = j;
      }
    }
  }
  cout<<"SHAPE OF d:\t"<<d.rows()<<"\t"<<d.cols()<<endl;
  cout<<"d:\t\n"<<d<<endl;
  printf("Reshaping\n");
  cout<<"D1:\n"<<d1<<endl;
  cout<<"DONE"<<endl;

   return std::make_tuple(min_index,error);
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> svd_motion_estimation(Eigen::MatrixXd previous_points, Eigen::MatrixXd  current_points) {
  Eigen::Vector<double,2> prev_mean = previous_points.colwise().mean();
  Eigen::Vector<double,2> curr_mean = current_points.colwise().mean();
  Eigen::MatrixXd temp             = prev_mean.colwise().replicate(previous_points.rows());
  Eigen::MatrixXd prev_shift       = previous_points - temp;
  temp                             = curr_mean.colwise().replicate(previous_points.rows());
  Eigen::MatrixXd curr_shift       = current_points - temp;
  Eigen::MatrixXd W = curr_shift*prev_shift.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd1(W);
  Eigen::MatrixXd R = (svd1.matrixU()*svd1.matrixV()).transpose();
  Eigen::MatrixXd t   = prev_mean - (R*curr_mean);

  return std::make_tuple(R,t);
}


Eigen::MatrixXd update_homogenous_matrix( Eigen::MatrixXd H_in, const Eigen::MatrixXd& R, Eigen::MatrixXd T){
  int row_size = R.rows();
  Eigen::MatrixXd H = Eigen::MatrixXd::Constant( row_size+1, row_size+1, 0);
  H(Eigen::seq(0,row_size-1),Eigen::seq(0,row_size-1)) = R;
  H(Eigen::seq(0,row_size-1),row_size) = T;
  H(row_size, row_size) = 1.0;

  if(H_in.rows() == 0)
    return H_in;
  else
    return H_in*H;
}


std::tuple<Eigen::MatrixXd,Eigen::VectorXd> icpMatching( Eigen::MatrixXd previous_points,Eigen::MatrixXd current_points){
  /*
   Iterative Closest Point matching
    - input
    previous_points: 2D or 3D points in the previous frame
    current_points: 2D or 3D points in the current frame
    -other
    H: Homogenous Transformation Matrix
    - output
    R: Rotation matrix
    T: Translation vector
    */
  Eigen::MatrixXd R,T,H;
  auto dError   = std::numeric_limits<float>::max();
  auto preError =  std::numeric_limits<float>::max();
  float error;
  std::vector<int> index;
  int count = 0;
  while (dError>=EPS){
    Eigen::MatrixXd Rt;
    Eigen::MatrixXd Tt;
    count +=1;
    std::tie(index, error) = nearest_neighbor_association(previous_points, current_points);
    std::tie(Rt, Tt) = svd_motion_estimation(previous_points, current_points);
    current_points = Rt*current_points+Tt.colwise().replicate(Tt.rows());
    dError-=preError-error;
    printf("RESIDUAL:\t%f\n\n", error);

    if (dError < 0) {
      printf("NO CONVERGENCE\n");
      break;
    }
    preError = error;
    H = update_homogenous_matrix(H, Rt, Tt);
    if (dError<= EPS){
      printf("Converging Count:%d\t Error:%f\t dError:%f\t", count, error, dError);
      break;
    }
    else if(MAX_ITER<=count){
      printf("Not Converging Count:%d\t Error:%f\t dError:%f\t", count, error, dError);
      break;
    }
    R = H(Eigen::seq(0,H.rows()-2),Eigen::seq(0,H.cols()-2));
    T = H(Eigen::seq(0,H.rows()-2),H.cols()-1);
  }
  return make_tuple(R,T);
}

int main(){

  Eigen::Vector3f motion(0.5,2.0,M_PI/8.0);
  int nsim = 3;

  for(int i = 0; i< nsim; i++) {
    Eigen::MatrixXd previous_points = Eigen::MatrixXd::Random(NPOINT, 2);
    Eigen::MatrixXd current_points  = previous_points;

    previous_points += Eigen::MatrixXd::Constant(NPOINT, 2, 1);
    previous_points /= 2;
    previous_points =  previous_points - Eigen::MatrixXd::Constant(NPOINT, 2, 0.5);
    previous_points *= FIELDLENGTH;

    double prev_x, prev_y;
    for (int row = 0; row < NPOINT; row++) {
      prev_x = previous_points(row, 0), prev_y = previous_points(row, 1);
      current_points(row, 0) =
          prev_x * cos(motion(2)) - sin(motion(2)) * prev_y + motion(0);
      current_points(row, 1) =
          prev_x * cos(motion(2)) - sin(motion(2)) * prev_y + motion(1);
    }
    cout <<"PREVIOUS POINTS:\n"<< previous_points << "\n\n\n";
    cout << "CURRENT_POINTS:\n"<< current_points  << "\n\n\n";



    Eigen::MatrixXd R;
    Eigen::VectorXd T;
    std::tie(R, T) = icpMatching(previous_points, current_points);
    // unpack with std::tie
    cout << R << endl;
    cout << T << endl;
  }
}
