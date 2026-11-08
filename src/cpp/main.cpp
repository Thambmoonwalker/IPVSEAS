
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

int main()
{
    std::cout << "IPVSEAS C++ environment initialized!" << std::endl;

    std::cout << "OpenCV version: "
              << CV_VERSION << std::endl;

    // Eigen matrix test
    Eigen::Matrix2d matrix;

    matrix << 1, 2,
              3, 4;

    std::cout << "\nEigen Matrix:\n";
    std::cout << matrix << std::endl;

    return 0;
}
