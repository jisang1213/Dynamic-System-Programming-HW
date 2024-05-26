#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>


class Logger {
public:
    std::ofstream file;
    Logger(const std::string& filename) {
        file.open(filename, std::ios::out);
        file << "Time,Position,theta,Force\n";
    }
    ~Logger() {
        if (file.is_open()) {
            file.close();
        }
    }
    void log(double time, double position, double theta, double Force) {
        file << time << "," << position << "," << theta << "," << Force << "\n";
    }
};

class CartPole {
public:
    double x1, d_x1, theta2, d_theta2;
    double error_theta_old, error_x_old;
    double h = 0.001;
    double integral_theta=0, integral_x=0;
    double F;
    double m_1 = 1.0, m_2 = 8.0, I_1 = 0.024167, I_2 = 1.01, length = 0.6, g = 9.81; // Model parameters
    int count = 0;
    Logger logger;

    CartPole() : x1(0.0), d_x1(0.0), theta2(0.523599), d_theta2(0.0), F(0.0), logger("results_20240000.csv") {} // Initial condition

    void semi_implicit_integration() {

        pid_controller();

        //F=0;

        ////////////////// TODO //////////////////////
        /// Implement the semi-implicit Euler method and calculate x1, d_x1, theta2, d_theta2 (Part2. 2-2)

        Eigen::VectorXd state = Calculate_Left_Matrix().inverse() * Calculate_Right_Vector();

        double dd_x1 = state(0);
        double dd_theta2 = state(5);

        d_x1 += h * dd_x1;
        d_theta2 += h * dd_theta2;

        x1 += h * d_x1;
        theta2 += h * d_theta2;

        ////////////////// TODO END //////////////////

        // loogging
        logger.log(h * count, x1, theta2, F);
        count++;
    }

    Eigen::MatrixXd Calculate_Left_Matrix() {
        Eigen::MatrixXd matrix(10, 10);
        matrix.setZero();

        ////////////////// TODO //////////////////////
        /// Calculate the matrix on the left of the motion equation (Part2. 2-1)

        //fill mass matrix
        Eigen::MatrixXd M = Eigen::Matrix::Zero(6,6);
        M(0,0) = m_1;
        M(1,1) = m_1;
        M(2,2) = I_1;
        M(3,3) = m_2;
        M(4,4) = m_2;
        M(5,5) = I_2;

        //fill Jacobian
        Eigen::MatrixXd J = Eigen::Matrix::Zero(4,6);
        J(0,0) = -1;
        J(0,3) = 1;
        J(0,5) = -length * cos(theta2);
        J(1,1) = -1;
        J(1,4) = 1;
        J(1,5) = length * sin(theta2);
        J(2,1) = 1;
        J(3,2) = 1;

        matrix.block(0,0,6,6) = M;
        matrix.block(6,0,4,6) = J;
        matrix.block(0,6,6,4) = J.transpose();

        ////////////////// TODO END //////////////////

        return matrix;
    }

    Eigen::VectorXd Calculate_Right_Vector() {
        Eigen::VectorXd vec(10);
        vec.setZero();

        ////////////////// TODO //////////////////////
        /// Calculate the vector on the right of the motion equation (Part2. 2-1)

        //vec 0~5 are for q"
        //vec 6~9 are for lamdas
        vec[0] = F;
        //vec[1] = -9.81*m_1;   ??
        vec[4] = -9.81*m_2;
        vec[6] = -length * sin(theta2) * d_theta2 * d_theta2;
        vec[7] = -length * cos(theta2) * d_theta2 * d_theta2;

        ////////////////// TODO END //////////////////

        return vec;
    }

    void pid_controller()
    {
        ////////////////// TODO //////////////////////
        /// Calculate the force to keep the pole vertical (Part3. 3-1)
        double P_gain_x = -51.675;
        double I_gain_x = 0.1;
        double D_gain_x = -39.677;

        double P_gain_theta = -316.23;
        double I_gain_theta = 0.2;
        double D_gain_theta = -68.454;
        
        double target_x = 0.0;
        double target_theta = 0.0;

        double error_x = target_x - x1;
        double derror_x = (error_x - error_x_old)/h;
        integral_x += error_x * h;
        error_x_old = error_x;

        double error_theta = target_theta - theta2;
        double derror_theta = (error_theta - error_theta_old)/h;
        integral_theta += error_theta * h;
        error_theta_old = error_theta;

        F = P_gain_x * error_x + I_gain_x * integral_x + D_gain_x * derror_x;
        F += P_gain_theta * error_theta + I_gain_theta * integral_theta + D_gain_theta * derror_theta;
        ////////////////// TODO END //////////////////
    }
};

int main() {
    CartPole cartpole;
    for (int i = 0; i <= 30000; ++i) {
        cartpole.semi_implicit_integration();
    }
    return 0;
}
