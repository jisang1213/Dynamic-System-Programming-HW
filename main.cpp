#include <math.h>
#include <stdio.h>
#include "matplotlibcpp.h"

#define g 9.80665
#define PI 3.14159265

namespace plt = matplotlibcpp; 

double theta[800] = {PI/4};
double vel[800] = {0.};
double Ek[800] = {0.};
double Ep[800];
double Et[800];

int main(){
    for(int i=1; i<800; i++){
        vel[i] = vel[i-1] - g * sin(theta[i-1]) * 0.01;
        theta[i] = theta[i-1] + vel[i];
        Ek[i] = 0.5*vel[i]*vel[i];
        Ep[i] = g * (1-cos(theta[i]));
        Et[i] = Ek[i] + Ep[i];
        printf("%f\n", theta[i]);
    }

    plt::plot({1,2,3,4}, "*");
    plt::show;
}