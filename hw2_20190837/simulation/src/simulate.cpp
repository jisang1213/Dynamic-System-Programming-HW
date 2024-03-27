#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <../../mymat/include/mymat.hpp>
//#include "../include/matplotlibcpp.h"

//namespace plt = matplotlibcpp; 

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

    Mat33 R_l2g(1.0, 1.0, 1.0); // passive rotation matrix from local to global frame (orientation of local frame)
    std::cout << "R_l2g" << std::endl;
    R_l2g.display();

    Vec3 w_gcs(0.0, 1.0, 0.01); // angular velocity of the local coordinate system expressed in the global frame

    std::cout << "w_gcs" << std::endl;
    w_gcs.display();

    Vec3 wd_lcs; // dw/dt in the local frame
    Mat33 eye(1.0, 1.0, 1.0);

    Vec3 w_lcs; //angular velocity expressed in global frame

    int count =0;

    std::vector<double> cosx(num_rec), cosy(num_rec), cosz(num_rec), time(num_rec);

    std::cout << "Simulation start" << std::endl;
    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        // TODO: implement semi-implicit Euler method to simulate the tennis racket theorem

        //convert angular velocity to global frame
        w_lcs = R_l2g.transpose()*w_gcs;

        //find rate change of angular velocity in local frame
        wd_lcs = (I_lcs.inverse()*((Vec3)(I_lcs*w_lcs)).cross(w_lcs));

        //increment angular velocity in local frame using EOM
        w_lcs = w_lcs + wd_lcs*h;

        //transform updated angular velocity to global frame
        w_gcs = R_l2g*w_lcs;

        //Update orientation using angular velocity (dR/dt = W x R)
        R_l2g = R_l2g + (w_gcs.skew()*R_l2g)* h;



        // //compute axis, angle, then rotation matrix using Rodriguez's formula
        // K = w_gcs.normalized().skew();
        // angle = w_gcs.norm()*h;
        // rot = eye + K*sin(angle) + K*K*(1-cos(angle));



        // record the simulation results to the file
        if (i_sim % rec_steps == 0)
        {
            //record the cosine values between the local and global axes (diagonal elements)
            cosx[count] = R_l2g.get_elem(0,0);
            cosy[count] = R_l2g.get_elem(1,1);
            cosz[count] = R_l2g.get_elem(2,2);
            time[count] = i_sim * h;
            count++;

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
    // plt::title("Cosine values of local and global axes");
    // plt::plot(time, cosx);
    // plt::plot(time, cosy, "r");
    // plt::xlabel("time(s)");
    // plt::ylabel("cos");

    // plt::show();
    
    return 0;
}