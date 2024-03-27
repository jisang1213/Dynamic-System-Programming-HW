#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <mymat.hpp>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp; 

int main()
{
    // create and open file to record simulation results
    std::ofstream file_results("results.csv");

    double h = 0.0001; // simulation timestep, 0.0001 seconds (0.1 ms)
    int n_sim = 6000000; // number of simulation step
    int rec_steps = 1000; // record the result every n steps (every 0.1 s)
    int num_rec = n_sim/rec_steps;
    
    Mat33 I_lcs(1.8, 2.0, 2.2); // inertia of the body (local coordinate system)
    std::cout << "I_lcs" << std::endl;
    I_lcs.display();

    Mat33 I_lcs_inverse = I_lcs.inverse();

    Mat33 R_l2g(1.0, 1.0, 1.0); // passive rotation matrix from local to global frame (orientation of local frame)
    std::cout << "R_l2g" << std::endl;
    R_l2g.display();

    Vec3 w_lcs(0.0, 1.0, 0.01); // angular velocity of the local coordinate system expressed in the local frame

    std::cout << "w_lcs" << std::endl;
    w_lcs.display();

    Vec3 wd_lcs; // dw/dt in the local frame
    Mat33 eye(1.0, 1.0, 1.0);

    int count =0;

    std::vector<double> cosx(num_rec), cosy(num_rec), cosz(num_rec), time(num_rec);

    std::cout << "Simulation start" << std::endl;
    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        // TODO: implement semi-implicit Euler method to simulate the tennis racket theorem

        //find rate change of angular velocity in local frame
        wd_lcs = I_lcs_inverse*((w_lcs.skew()*(I_lcs*w_lcs))*-1);

        //increment angular velocity in local frame
        w_lcs = w_lcs + wd_lcs*h;

        //Update orientation. A more computational efficient methods was used involving time derivative of the rotation matrices.
        //Refer to README for the formula for time derivative of rotation matrices.
        R_l2g = R_l2g + (R_l2g*w_lcs.skew())*h;

        // record the simulation results to the file
        if (i_sim % rec_steps == 0)
        {
            //record the cosine values between the local and global axes (diagonal elements)
            cosx[count] = R_l2g.get_elem(0,0);
            cosy[count] = R_l2g.get_elem(1,1);
            cosz[count] = R_l2g.get_elem(2,2);
            time[count] = i_sim * h;
            count++;

            std::cout << i_sim*h << std::endl;

            file_results << i_sim * h << ", "; // record time
            // record the orientation of the local frame
            for (int i_row = 0; i_row < 3; i_row++) for (int i_col = 0; i_col < 3; i_col++) file_results << R_l2g.get_elem(i_row, i_col) << ", "; 
            file_results << "\n";
        }
    }
    std::cout << "Simulation end" << std::endl;

    // close the file
    file_results.close();

    // //plot (reconsider using matplotlibbcpp as it requires python)
    plt::title("Cosine values of local and global axes");
    plt::plot(time, cosx, "b", {{"label", "x-axis"}});
    plt::plot(time, cosy, "r", {{"label", "y-axis"}});
    plt::plot(time, cosz, "y", {{"label", "z-axis"}});
    plt::xlabel("time(s)");
    plt::ylabel("cos");
    plt::legend({{"upper right"}});

    plt::show();
    
    return 0;
}