//
// Created by abhinav137 on 19/05/21.
//
#include <iostream>
#include<opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/src/Core/ArithmeticSequence.h>
#include "SimParams.h"
#include <cmath>
using namespace std;
//////////////////////////////////////////////IMPORT SIMULATION PARAMS//////////////////////////

class Params{
public:
  Params(){
    Eigen::DiagonalMatrix<double, 1> Q(static_cast<long>(0.04));
    Eigen::DiagonalMatrix<double, 2> P(2.0, ((M_PI / 180.0) * 40));
    Eigen::DiagonalMatrix<double, 1> Q_Sim(static_cast<long>(0.04));
    Eigen::DiagonalMatrix<double, 2> P_Sim(1.0, ((M_PI / 180.0) * 30));
    Eigen::Vector4f x_est(0,0,0,0); //State Vector [x y yaw v]'
    Eigen::Vector4f x_true(0,0,0,0); //# True State Vector [x y yaw v]'
    Eigen::MatrixXf px = Eigen::MatrixXf::Constant(4,NP, 0.0);
    Eigen::MatrixXf pw = Eigen::MatrixXf::Constant(1,NP, 1.0/NP) ;
    Eigen::MatrixXf x_dr = Eigen::MatrixXf::Constant(4,NP, 0) ;
    Eigen::Vector4f h_x_est = x_est;
    Eigen::Vector4f h_x_true = x_true;
    Eigen::Vector4f h_x_dr = x_true;
  }
};

void observation()


Eigen::Vector2d calc_input(){ //control input
  float v = 1.0;
  float yaw_rate = 1.0;
  Eigen::Vector2d u(v, yaw_rate);
  return u;
}

Eigen::Vector4f motion_model(Eigen::Vector4f x, Eigen::Vector2f u){
  Eigen::Matrix4f F_;
  F_<<1.0,   0,   0,   0,
      0, 1.0,   0,   0,
      0,   0, 1.0,   0,
      0,   0,   0, 1.0;

  Eigen::Matrix<float, 4, 2> B_;
  B_<< DT * cos(x(2,0)),  0,
      DT * sin(x(2,0)),  0,
      0.0,  DT,
      1.0,  0.0;

  return F_ * x + B_ * u;
} // x_{t+1} = F@x_{t}+B@u_t  ** '@' means matrix product in python **

float gauss_likelihood(float x, float sigma){
  float p = 1.0/sqrt(2.0*M_PI* pow(sigma, 2))*
            pow(pow(x,2)/(2.0* pow(sigma,2)),2);
  return p;
}

Eigen::Matrix3f calc_covariance(Eigen::Vector4f x_est, Eigen::MatrixXf px, Eigen::MatrixXf pw){
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  Eigen::Matrix<float,1,1> pweight = pw*pw.transpose();
  float sumsq = 0;
  for (int i = 0; i<NP;i++){
    Eigen::Vector4f dx = (px.col(i) - x_est)(0,Eigen::seq(0,3));
    //Select only (x,y) and control input u
    cov += pw(i) * dx*dx.transpose();
    sumsq += pw(i)*pw(i);
  }
  cov*= (1.0/(1.0-sumsq));
  return cov;
}

int main(){
  printf("__________________START__________________\n");
  Params params_obj;
  float time = 0.0;
  while (SIM_TIME>= time){
    time += DT;
    Eigen::Vector2d u = calc_input();
    x_true
  }

}

