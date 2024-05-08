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
    ixx1_ = 1.0;
    iyy1_ = 1.0;
    izz1_ = 1.0/12.0;
    q1_init_ = 1.578;
    y1_init_ = 0.0;
    z1_init_ = 0.5;

    m2_ = 4.0;
    L2_ = 4.0;
    ixx2_ = 4.0;
    iyy2_ = 4.0;
    izz2_ = 16.0/3.0;
    q2_init_ = 0.3533;
    y2_init_ = 2*cos(0.3533);
    z2_init_ = 1 + 2*sin(0.3533);

    m3_ = 2.5;
    L3_ = 2.5;
    ixx3_ = 2.5;
    iyy3_ = 2.5;
    izz3_ = 125.0/96.0;
    q3_init_ = 1.2649;
    y3_init_ = 4*cos(0.3533) - 1.25*cos(1.2649);
    z3_init_ = 1.25*sin(1.2649);

    // ToDo: set the mass matrix and the inversion of the mass matrix
    M_.setZero();
    M_(0,0) = ixx1_;
    M_(1,1) = iyy1_;
    M_(2,2) = izz1_;
    M_(3,3) = ixx2_;
    M_(4,4) = iyy2_;
    M_(5,5) = izz2_;
    M_(6,6) = ixx3_;
    M_(7,7) = iyy3_;
    M_(8,8) = izz3_;
    M_inv_ = M_.inverse();

    ////////////////// TODO END //////////////////////

}

Eigen::MatrixXd calculate_jacobian(Eigen::VectorXd q){
    Eigen::MatrixXd J;
    ////////////////// TODO //////////////////////
    /// Calculate the Jacobian using the input state vector q
    double theta1 = q(2,0);
    double theta2 = q(5,0);
    double theta3 = q(8,0);
    J.setZero(8,9);
    J(0,0) = 1;
    J(0,2) = L1_/2 * sin(theta1);

    J(1,1) = 1;
    J(1,2) = -L1_/2 * cos(theta1);

    J(2,2) = L1_ * sin(theta1);
    J(2,3) = 1;
    J(2,5) = L2_/2 * sin(theta2);

    J(3,2) = -L1_ * cos(theta1);
    J(3,4) = 1;
    J(3,5) = -L2_/2 * cos(theta2);

    J(4,6) = 1;
    J(4,8) = L3_/2 * sin(theta3);

    J(5,7) = 1;
    J(5,8) = -L3_/2 * cos(theta3);

    J(6,2) = -L1_ * sin(theta1);
    J(6,5) = -L2_ * sin(theta2);
    J(6,8) = L3_ * sin(theta3);

    J(7,2) = L1_ * cos(theta1);
    J(7,5) = L2_ * cos(theta2);
    J(7,8) = -L3_ * cos(theta3);

    ////////////////// TODO END //////////////////////
    return J;
}

Eigen::MatrixXd calculate_jacobian_dot(Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::MatrixXd Jdot;

    ////////////////// TODO //////////////////////
    /// Calculate the time derivative of Jacobian using the input state vectors q and qdot
    double theta1 = q(2,0);
    double theta2 = q(5,0);
    double theta3 = q(8,0);
    double omega1 = qdot(2,0);
    double omega2 = qdot(5,0);
    double omega3 = qdot(8,0);

    Jdot.setZero(8,9);
    Jdot(0,2) = L1_/2 * cos(theta1) * omega1;

    Jdot(1,2) = L1_/2 * sin(theta1) * omega1;

    Jdot(2,2) = L1_ * cos(theta1) * omega1;
    Jdot(2,5) = L2_/2 * cos(theta2) * omega2;

    Jdot(3,2) = L1_ * sin(theta1) * omega1;
    Jdot(3,5) = L2_/2 * sin(theta2) * omega2;

    Jdot(4,8) = L3_/2 * cos(theta3) * omega3;

    Jdot(5,8) = L3_/2 * sin(theta3) * omega3;

    Jdot(6,2) = -L1_ * cos(theta1) * omega1;
    Jdot(6,5) = -L2_ * cos(theta2) * omega2;
    Jdot(6,8) = L3_ * cos(theta3) * omega3;
    
    Jdot(7,2) = -L1_ * sin(theta1) * omega1;
    Jdot(7,5) = -L2_ * sin(theta2) * omega2;
    Jdot(7,8) = L3_ * sin(theta3) * omega3;


    ////////////////// TODO END //////////////////////
    return Jdot;
}

Eigen::VectorXd calculate_Fext(Eigen::VectorXd q){
    Eigen::VectorXd Fext;
    ////////////////// TODO //////////////////////
    /// Calculate the external force using the input state vector q
    double theta1 = q(2,0);
    double theta2 = q(5,0);
    double theta3 = q(8,0);
    Fext.setZero(9);
    Fext(1,0) = -m1_ * g;
    //Fext(2,0) = L1_/2 * m1_ * g * cos(theta1);
    Fext(4,0) = -m2_ * g;
    //Fext(5,0) = (L1_*cos(theta1) + L2_/2 * cos(theta2)) * m2_ * g;
    Fext(7,0) = -m3_ * g;
    //Fext(8,0) = (L1_*cos(theta1) + L2_*cos(theta2) - L3_/2 * cos(theta3)) * m3_ * g;

    ////////////////// TODO END //////////////////////
    return Fext;
}

Eigen::VectorXd calculate_lambda(Eigen::MatrixXd J, Eigen::MatrixXd Jdot, Eigen::VectorXd Fext, Eigen::VectorXd q, Eigen::VectorXd qdot){
    Eigen::VectorXd lambda;
    ////////////////// TODO //////////////////////
    /// Calculate lambda which connects the constraint force and the jacobian
    Eigen::MatrixXd temp = J*M_inv_*(J.transpose());

    lambda = temp.inverse() * (-1*Jdot*qdot - J*M_inv_*Fext);


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
    Eigen::VectorXd ConstraintErr;
    ////////////////// TODO //////////////////////
    /// Calculate the constraint errors under given state vector q
    double y1 = q(0,0);
    double z1 = q(1,0);
    double theta1 = q(2,0);
    double y2 = q(3,0);
    double z2 = q(4,0);
    double theta2 = q(5,0);
    double y3 = q(6,0);
    double z3 = q(7,0);
    double theta3 = q(8,0);

    ConstraintErr.setZero(8);
    ConstraintErr(0,0) = y1 - L1_/2 * cos(theta1);
    ConstraintErr(1,0) = z1 - L1_/2 * sin(theta1);
    ConstraintErr(2,0) = y2 - (L1_*cos(theta1) + L2_/2 * cos(theta2));
    ConstraintErr(3,0) = z2 - (L1_*sin(theta1) + L2_/2 * sin(theta2));
    ConstraintErr(4,0) = y3 - (L0_ + L3_/2 * cos(theta3));
    ConstraintErr(5,0) = z3 - L3_/2 * sin(theta3);
    ConstraintErr(6,0) = L1_*cos(theta1) + L2_*cos(theta2) - L3_*cos(theta3) - L0_;
    ConstraintErr(7,0) = L1_*sin(theta1) + L2_*sin(theta2) - L3_*sin(theta3);

    ////////////////// TODO END //////////////////
    return ConstraintErr;
}

int main()
{
    std::ofstream q_results("q_results_cpp.csv");
    std::ofstream qdot_results("qdot_results_cpp.csv");
    std::ofstream constraint_error_results("constraint_error_cpp.csv");

    double h = 0.001; // simulation timestep, 0.001 seconds (1 ms)
    int n_sim = 50000; // number of simulation steps (1000 s)
    int rec_steps = 10; // record the result every n steps (every 0.1 s)
    initialize();

    Eigen::VectorXd q(9);
    Eigen::VectorXd qdot(9);

    std::cout << "Simulation start" << std::endl;
    
    //////////////// TODO ////////////////
    // TODO: you can set yout own variavble.
    Eigen::VectorXd qdd;
    Eigen::MatrixXd J;
    Eigen::MatrixXd Jdot;
    Eigen::VectorXd lambda;
    Eigen::VectorXd Fext;
    Eigen::VectorXd Fc;
    Eigen::VectorXd constraint;
    q(0,0) = y1_init_;
    q(1,0) = z1_init_;
    q(2,0) = q1_init_;
    q(3,0) = y2_init_;
    q(4,0) = z2_init_;
    q(5,0) = q2_init_;
    q(6,0) = y3_init_;
    q(7,0) = z3_init_;
    q(8,0) = q3_init_;
    qdot.setZero();

    double t = 0;

    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        
        
        // TODO: implement semi-implicit Euler method to simulate the four bar linkage motion
        J = calculate_jacobian(q);
        Jdot = calculate_jacobian_dot(q, qdot);
        Fext = calculate_Fext(q);
        lambda = calculate_lambda(J, Jdot, Fext, q, qdot);
        Fc = calculate_Fc(J, lambda);
        qdd = M_inv_*(Fext + Fc);
        
        qdot = qdot + qdd*h;
        q = q + qdot*h;
        constraint = calculate_constraint_error(q);

        if (i_sim % rec_steps == 0)
        {
            q_results << t << ",";
            qdot_results << t << ",";
            // TODO: record q_results_cpp.csv contains the value of the angle q1, q2, and q3 in radian (q1,q2,q3)
            q_results << q(2,0) << "," << q(5,0) << "," << q(8,0) << ",";
            q_results <<"\n";

            // TODO: record qdot_results_cpp.csv contains the value of the angular velocity qdot1, qdot2, and qdot3 in radian (qdot1,qdot2,qdot3)
            qdot_results << qdot(2,0) << "," << qdot(5,0) << "," << qdot(8,0) << ",";
            qdot_results <<"\n";

            // TODO: record constraint_error_cpp.csv contains the constraint error at this time step (error1,error2,...)
            constraint_error_results << constraint(0,0) << "," << constraint(1,0) << "," << constraint(2,0) << "," << constraint(3,0) << "," 
            << constraint(4,0) << "," << constraint(5,0) << "," << constraint(6,0) << "," << constraint(7,0) << ",";
            constraint_error_results << "\n";
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


