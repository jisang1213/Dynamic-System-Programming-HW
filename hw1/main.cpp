#include <cmath>
#include <stdio.h>
#include <vector>
#include <string>
#include "matplotlibcpp.h"

#define g 9.80665
#define PI 3.14159265
#define steps 800
#define dt 0.01

#define explicit
//#define semi_implicit
//#define leapfrog

namespace plt = matplotlibcpp; 

std::vector<double> theta(steps), vel(steps), Ek(steps), Ep(steps), Et(steps), timestep(steps);
std::string mode;

int main(){
    theta[0] = PI/4;
    Ep[0] = g * (1-cos(PI/4));
    Et[0] = Ek[0] + Ep[0];

    #ifdef explicit
    mode = "Explicit Euler";
    for(int i=1; i<steps; i++){
        timestep[i] = (double) (dt*i);
        vel[i] = vel[i-1] - g * sin(theta[i-1]) * dt;
        theta[i] = theta[i-1] + vel[i-1] * dt;
        Ek[i] = 0.5*vel[i]*vel[i];
        Ep[i] = g * (1-cos(theta[i]));
        Et[i] = Ek[i] + Ep[i];
    }
    #endif

    #ifdef semi_implicit
    mode = "Semi-implicit";
    for(int i=1; i<steps; i++){
        timestep[i] = (double) (dt*i);
        vel[i] = vel[i-1] - g * sin(theta[i-1]) * dt;
        theta[i] = theta[i-1] + vel[i] * dt;
        Ek[i] = 0.5*vel[i]*vel[i];
        Ep[i] = g * (1-cos(theta[i]));
        Et[i] = Ek[i] + Ep[i];
    }
    #endif

    #ifdef leapfrog
    mode = "Leapfrog";
    for(int i=1; i<steps; i++){
        timestep[i] = (double) (dt*i);
        theta[i] = theta[i-1] + vel[i-1] * dt - 0.5*g*sin(theta[i-1])*dt*dt;
        vel[i] = vel[i-1] - 0.5*g*(sin(theta[i-1])+sin(theta[i])) * dt;
        Ek[i] = 0.5*vel[i]*vel[i];
        Ep[i] = g * (1-cos(theta[i]));
        Et[i] = Ek[i] + Ep[i];
    }
    #endif

    plt::suptitle(mode);
    plt::subplot(1,2,1);
    plt::plot(timestep, theta);
    plt::xlabel("time(s)");
    plt::ylabel("Angle(rad)");
    plt::xlim(0., 8.0); 
    
    plt::subplot(1,2,2);
    plt::plot(timestep, Et, "r");
    plt::xlabel("time(s)");
    plt::ylabel("Energy(J)");
    plt::xlim(0., 8.0); 
    plt::ylim(0., 6.0); 

    plt::show();
}