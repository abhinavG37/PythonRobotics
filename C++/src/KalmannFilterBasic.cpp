#include <iostream>
#include <vector>
#include<functional>
#include <bits/stdc++.h>
#include <algorithm>

#include <eigen3/Eigen/Dense>


using namespace Eigen;

using namespace std;



double expectationCalc(vector<double> x, vector<double> p){
    vector<double> prod(3,0);
    transform(x.begin(),x.end(),p.begin(), prod.begin(),multiplies<double>());    
   double E_x = accumulate(prod.begin(),prod.end(),0.0);
    cout<<"Expectation:" << E_x << endl;
    return E_x;
}

void eigenTester(){
      MatrixXd m;
    for (int i=0; i<10;i++){
   m= MatrixXd::Random(3,3);
    cout<<m+MatrixXd::Constant(3,3,1.2)*50<<endl;
    }
    cout<<m<<endl;
    VectorXd v(1,2,3);
    
    cout<<"VECTOR:"<<v<<endl;

}


int main(){
vector<double> x{3,1,2};
vector<double> p{0.1, 0.3, 0.4};

double E_x = expectationCalc(x,p);
eigenTester();
}