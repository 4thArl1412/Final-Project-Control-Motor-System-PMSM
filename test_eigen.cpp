#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

int main() {
    // Test the problematic line
    // Wout is (2, 8) in Speed_CTRL.cpp
    MatrixXd Wout = MatrixXd::Random(2, 8);
    double error_speed = 0.5;

    // OLD WAY (causes template error):
    // VectorXd delta_h = Wout.transpose() * (error_speed * VectorXd::Ones(2));

    // NEW WAY (should work):
    // Wout.transpose() is (8, 2)
    // error_vec is (2, 1)
    // Result is (8, 1)
    VectorXd error_vec(2);
    error_vec.setConstant(error_speed);
    VectorXd delta_h = Wout.transpose() * error_vec;

    std::cout << "Success! delta_h computed (size " << delta_h.size() << "):" << std::endl;
    std::cout << delta_h.head(3) << std::endl;  // Print first 3 elements

    return 0;
}


