#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

double L1_, L2_, L3_, L0_;
double m1_, m2_, m3_;
double ixx1_, iyy1_, izz1_;
double ixx2_, iyy2_, izz2_;
double ixx3_, iyy3_, izz3_;
double y1_init_, y2_init_, y3_init_, z1_init_,z2_init_,z3_init_;
double q1_init_, q2_init_, q3_init_;
double q1dot_init_, q2dot_init_, q3dot_init_;

Eigen::MatrixXd M_(9,9), M_inv_(9,9);

double g = 9.81;

void initialize(){

    ////////////////// TODO //////////////////////
    // Edit the value of the properties

    L0_ = 3.0;

    m1_ = 1.0;
    L1_ = 1.0;
    ixx1_ = m1_*L1_*L1_/12.0;
    iyy1_ = 0.0;
    izz1_ = 0.0;
    q1_init_ = 1.578;
    y1_init_ = L1_/2.0 * cos(q1_init_);
    z1_init_ = L1_/2.0 * sin(q1_init_);

    m2_ = 4.0;
    L2_ = 4.0;
    ixx2_ = m2_*L2_*L2_/12.0;
    iyy2_ = 0.0;
    izz2_ = 0.0;
    q2_init_ = 0.3533;
    y2_init_ = L2_/2.0 * cos(q2_init_);
    z2_init_ = L1_ + L2_/2.0 * sin(q2_init_);

    m3_ = 2.5;
    L3_ = 2.5;
    ixx3_ = m3_*L3_*L3_/12.0;
    iyy3_ = 0.0;
    izz3_ = 0.0;
    q3_init_ = 1.2649;
    y3_init_ = L0_ + L3_/2.0 * cos(q3_init_);;
    z3_init_ = L3_/2.0 * sin(q3_init_);

    // ToDo: set the mass matrix and the inversion of the mass matrix
    M_.setZero();
    M_inv_.setZero();

    M_(0,0) = m1_;
    M_(1,1) = m1_;
    M_(2,2) = ixx1_;

    M_(3,3) = m2_;
    M_(4,4) = m2_;
    M_(5,5) = ixx2_;

    M_(6,6) = m3_;
    M_(7,7) = m3_;
    M_(8,8) = ixx3_;

    M_inv_ = M_.inverse();
    ////////////////// TODO END //////////////////////

}

Eigen::MatrixXd calculate_jacobian(Eigen::VectorXd q){
    Eigen::MatrixXd J(8,9);
    J.setZero();
    ////////////////// TODO //////////////////////
    /// Calculate the Jacobian using the input state vector q
    J(0, 0) = 1;
    J(0, 2) = L1_/2 * sin(q(2));

    J(1, 1) = 1;
    J(1, 2) = -L1_/2 * cos(q(2));

    J(2, 2) = L1_ * sin(q(2));
    J(2, 3) = 1;
    J(2, 5) = L2_/2 * sin(q(5));

    J(3, 2) = -L1_ * cos(q(2));
    J(3, 4) = 1;
    J(3, 5) = -L2_/2 * cos(q(5));

    J(4, 6) = 1;
    J(4, 8) = L3_/2 * sin(q(8));

    J(5, 7) = 1;
    J(5, 8) = -L3_/2 * cos(q(8));

    J(6, 2) = L1_ * sin(q(2));
    J(6, 5) = L2_ * sin(q(5));
    J(6, 8) = -L3_ * sin(q(8));

    J(7, 2) = L1_ * cos(q(2));
    J(7, 5) = L2_ * cos(q(5));
    J(7, 8) = -L3_ * cos(q(8));
    ////////////////// TODO END //////////////////////
    return J;
}

Eigen::MatrixXd calculate_jacobian_dot(Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::MatrixXd Jdot(8,9);
    Jdot.setZero();

    ////////////////// TODO //////////////////////
    /// Calculate the time derivative of Jacobian using the input state vectors q and qdot
    Jdot(0, 2) = L1_/2 * cos(q(2)) * qdot(2);

    Jdot(1, 2) = L1_/2 * sin(q(2)) * qdot(2);

    Jdot(2, 2) = L1_ * cos(q(2)) * qdot(2);
    Jdot(2, 5) = L2_/2 * cos(q(5)) * qdot(5);

    Jdot(3, 2) = L1_ * sin(q(2)) * qdot(2);
    Jdot(3, 5) = L2_/2 * sin(q(5)) * qdot(5);

    Jdot(4, 8) = L3_/2 * cos(q(8)) * qdot(8);

    Jdot(5, 8) = L3_/2 * sin(q(8)) * qdot(8);

    Jdot(6, 2) = L1_ * cos(q(2)) * qdot(2);
    Jdot(6, 5) = L2_ * cos(q(5)) * qdot(5);
    Jdot(6, 8) = -L3_ * cos(q(8)) * qdot(8);

    Jdot(7, 2) = -L1_ * sin(q(2)) * qdot(2);
    Jdot(7, 5) = -L2_ * sin(q(5)) * qdot(5);
    Jdot(7, 8) = L3_ * sin(q(8)) * qdot(8);
    ////////////////// TODO END //////////////////////
    return Jdot;
}

Eigen::VectorXd calculate_Fext(Eigen::VectorXd q){
    Eigen::VectorXd Fext(9);
    Fext.setZero();
    ////////////////// TODO //////////////////////
    /// Calculate the external force using the input state vector q
    Fext(1) = -m1_ * g;
    Fext(4) = -m2_ * g;
    Fext(7) = -m3_ * g;

    ////////////////// TODO END //////////////////////
    return Fext;
}

Eigen::VectorXd calculate_lambda(Eigen::MatrixXd J, Eigen::MatrixXd Jdot, Eigen::VectorXd Fext, Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::VectorXd lambda;
    ////////////////// TODO //////////////////////
    /// Calculate lambda which connects the constraint force and the jacobian
    lambda = -(J*M_inv_*J.transpose()).inverse()*(Jdot*qdot + J*M_inv_*Fext);

    ////////////////// TODO END //////////////////////
    return lambda;
}

Eigen::VectorXd calculate_Fc(Eigen::MatrixXd J, Eigen::VectorXd lambda){
    Eigen::VectorXd Fc;
    ////////////////// TODO //////////////////////
    /// Calculate the constraint force using the Jacobian and lambda
    Fc = J.transpose() * lambda;

    ////////////////// TODO END //////////////////////
    return Fc;
}

Eigen::VectorXd calculate_constraint_error(Eigen::VectorXd q){
    Eigen::VectorXd ConstraintErr(8);
    ////////////////// TODO //////////////////////
    /// Calculate the constraint errors under given state vector q
    double c11, c12, c21, c22, c31, c32, c41, c42;

    c11 = q(0) - L1_/2*cos(q(2));
    c12 = q(1) - L1_/2*sin(q(2));

    c21 = q(3) - (L1_*cos(q(2)) + L2_/2*cos(q(5)));
    c22 = q(4) - (L1_*sin(q(2)) + L2_/2*sin(q(5)));

    c31 = q(6) - (L0_ + L3_/2*cos(q(8)));
    c32 = q(7) - (L3_/2*sin(q(8)));

    c41 = L0_ - (L1_*cos(q(2)) + L2_*cos(q(5)) - L3_*cos(q(8))); 
    c42 = L1_*sin(q(2)) + L2_*sin(q(5)) - L3_*sin(q(8)); 

    ConstraintErr << c11, c12, c21, c22, c31, c32, c41, c42;
    ////////////////// TODO END //////////////////
    return ConstraintErr;
}

int main()
{
    std::ofstream q_results("q_results_cpp_20190837.csv");
    std::ofstream qdot_results("qdot_results_cpp_20190837.csv");
    std::ofstream constraint_error_results("constraint_error_cpp_20190837.csv");

    double h = 0.001; // simulation timestep, 0.001 seconds (1 ms)
    int n_sim = 50000; // number of simulation steps (1000 s)
    int rec_steps = 10; // record the result every n steps (every 0.1 s)
    initialize();

    Eigen::VectorXd q(9);
    Eigen::VectorXd qdot(9);
    Eigen::VectorXd qdotdot(9);

    std::cout << "Simulation start" << std::endl;
    
    //////////////// TODO ////////////////
    // TODO: you can set yout own variavble.

    double t = 0;
    initialize();
    q << y1_init_, z1_init_, q1_init_, y2_init_, z2_init_, q2_init_, y3_init_, z3_init_, q3_init_;
    qdot.setZero();
    Eigen::MatrixXd J(8,9);
    Eigen::MatrixXd Jdot(8,9);
    Eigen::VectorXd Fext(9);
    Eigen::VectorXd lambda(8);
    Eigen::VectorXd Fc(9);
    Eigen::VectorXd error;

    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        
        // TODO: implement semi-implicit Euler method to simulate the four bar linkage motion
        J = calculate_jacobian(q);
        Jdot = calculate_jacobian_dot(q, qdot);
        Fext = calculate_Fext(q);
        lambda = calculate_lambda(J, Jdot, Fext, q, qdot);
        Fc = calculate_Fc(J, lambda);
        qdotdot = M_inv_*(Fext+Fc);
        
        qdot = qdot + h * qdotdot;
        q = q + h * qdot;
        
        error = calculate_constraint_error(q);

        if (i_sim % rec_steps == 0)
        {
            q_results << t << ",";
            qdot_results << t << ",";
            // TODO: record q_results_cpp.csv contains the value of the angle q1, q2, and q3 in radian (q1,q2,q3)
            
            q_results << q(2) << "," << q(5) << "," << q(8) << "," << "\n";

            // TODO: record qdot_results_cpp.csv contains the value of the angular velocity qdot1, qdot2, and qdot3 in radian (qdot1,qdot2,qdot3)

            qdot_results << qdot(2) << "," << qdot(5) << "," << qdot(8) << "," << "\n";

            // TODO: record constraint_error_cpp.csv contains the constraint error at this time step (error1,error2,...)

            constraint_error_results << error(0) << "," << error(1) << "," << error(2) << "," <<  error(3) << "," <<  error(4) << "," <<  error(5) << "," <<  error(6) << "," <<  error(7) << "," << "\n";
        }
        //////////////// TODO end ////////////////
        t = t + h;
    }
    std::cout << "Simulation end" << std::endl;

    // close the file
    q_results.close();
    qdot_results.close();
    constraint_error_results.close();

    return 0;
}