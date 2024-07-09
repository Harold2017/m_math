#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <iomanip>

// this requires kszie should be odd
cv::Mat fspecial_GaussianCV(double* kernel_size, double sigma)
{
    const double eps = 2.2204e-16;

    auto rk = cv::getGaussianKernel(kernel_size[0], sigma, CV_64FC1);
    auto ck = cv::getGaussianKernel(kernel_size[1], sigma, CV_64FC1);
    auto k = rk * ck.t();

    double MaxValue;
    cv::minMaxLoc(k, nullptr, &MaxValue, nullptr, nullptr);
    cv::Mat condition = ~(k < eps * MaxValue) / 255;
    condition.convertTo(condition, CV_64FC1);
    k = k.mul(condition);

    cv::Scalar SUM = cv::sum(k);
    if (SUM[0] != 0) k /= SUM[0];
    return k;
}

// exactly same as `fspecial('gaussian', size, sigma)` in MATLAB
// it can accept even ksize
cv::Mat fspecial_GaussianHM(double* kernel_size, double sigma)
{
    const double eps = 2.2204e-16;

    double size[2] = { (kernel_size[0] - 1) / 2, (kernel_size[1] - 1) / 2 };
    cv::Mat kernel(kernel_size[0], kernel_size[1], CV_64FC1, 0.0);
    int row = 0, col = 0;

    for (double y = -size[0]; y <= size[0]; ++y, ++row)
    {
        col = 0;
        for (double x = -size[1]; x <= size[1]; ++x, ++col)
            kernel.at<double>(row, col) = exp(-(pow(x, 2) + pow(y, 2)) / (2 * pow(sigma, 2)));
    }

    double MaxValue;
    cv::minMaxLoc(kernel, nullptr, &MaxValue, nullptr, nullptr);
    cv::Mat condition = ~(kernel < eps * MaxValue) / 255;
    condition.convertTo(condition, CV_64FC1);
    kernel = kernel.mul(condition);

    cv::Scalar SUM = cv::sum(kernel);
    if (SUM[0] != 0) kernel /= SUM[0];

    return kernel;
}

// laplacian of gaussian kernel (`fspecial('log', size, sigma)` in MATLAB)
// notice, in MATLAB, the result kernel's sum is not zero, which means
// this function should be equal to `fspecial('log', size, sigma) / sum(sum(fspecial('log', size, sigma)))`
// if only want to apply LOG to image with OpenCV, no need to use this kernel,
// can simply apply `cv::Laplacian` after `cv::GaussianBlur`
cv::Mat fspecial_LOG(double* kernel_size, double sigma)
{
    double size[2] = { (kernel_size[0] - 1) / 2, (kernel_size[1] - 1) / 2 };
    const double eps = 2.2204e-16;
    cv::Mat kernel(kernel_size[0], kernel_size[1], CV_64FC1, 0.0);
    int row = 0, col = 0;
    for (double y = -size[0]; y <= size[0]; ++y, ++row)
    {
        col = 0;
        for (double x = -size[1]; x <= size[1]; ++x, ++col)
        {
            // https://homepages.inf.ed.ac.uk/rbf/HIPR2/log.htm
            auto xy = (pow(x, 2) + pow(y, 2)) / (2 * pow(sigma, 2));
            kernel.at<double>(row, col) = -1.0 / (CV_PI * pow(sigma, 4)) * (1.0 - xy) * exp(-xy);
        }
    }

    double MaxValue;
    cv::minMaxLoc(kernel, nullptr, &MaxValue, nullptr, nullptr);
    cv::Mat condition = ~(abs(kernel) < abs(eps * MaxValue)) / 255;
    condition.convertTo(condition, CV_64FC1);
    kernel = kernel.mul(condition);

    cv::Scalar SUM = cv::sum(kernel);
    if (SUM[0] != 0) kernel /= SUM[0];

    return kernel;
}

int main()
{
    double kernel_size[2] = { 5, 5 };
    double sigma = 2.1;
    auto& cout = std::cout << std::setprecision(10);

    auto gk_hm = fspecial_GaussianHM(kernel_size, sigma);
    cout << gk_hm << '\n';

    auto gk_cv = fspecial_GaussianCV(kernel_size, sigma);
    cout << gk_cv << '\n';

    auto logk = fspecial_LOG(kernel_size, sigma);
    cout << logk << '\n';

    return 0;
}