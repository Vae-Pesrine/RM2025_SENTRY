#include <gaus_blur.h>

/**
 * @brief Calculates the value of a Gaussian function for a given input x and sigma.
 *
 * This function computes the value of a Gaussian function at a given point x, using the provided
 * standard deviation sigma. The Gaussian function is defined as:
 *
 * f(x) = (1 / (sqrt(2 * pi) * sigma)) * exp(-(x^2) / (2 * sigma^2))
 *
 * @param sigma The standard deviation of the Gaussian distribution.
 * @param x The input value at which to evaluate the Gaussian function.
 *
 * @return The value of the Gaussian function at the given input x and sigma.
 */ 
double gauss(double sigma, double x)
{
    double  expVal = -1 * (pow(x, 2) / pow(2*sigma, 2));
    double divider = sqrt(2 * M_PI * pow(sigma, 2));

    return (1 / divider) * exp(expVal);
}


/**
 * @brief Calculates a Gaussian kernel for smoothing operations.
 *
 * This function generates a Gaussian kernel of a specified size and standard deviation. The kernel
 * is used in convolution operations to smooth data. The kernel is normalized to ensure the sum of
 * all weights is 1.
 *
 * @param samples The number of samples to use in the Gaussian kernel. This determines the size of
 *                the kernel.
 * @param sigma The standard deviation of the Gaussian distribution used to generate the kernel.
 *
 * @return A vector containing the Gaussian kernel weights. The size of the vector is equal to the
 *         number of samples.
 */
std::vector<double> gaussKernel(int samples, double sigma)
{
    std::vector<double> kernel(samples);
    double mean = samples / 2;
    double sum = 0.0;
    for(auto i = 0; i < samples; ++i){
        kernel[i] = exp( -0.5 * (pow((i - mean) / sigma, 2.0)))/(2 * M_PI * sigma * sigma);  // 高斯核
        sum += kernel[i];
    }
    for(auto i = 0; i < samples; ++i){
        kernel[i] /= sum;  // 归一化
    }
    for(auto i = kernel.begin(); i != kernel.end(); ++i){

    }

    // ASSERT(kernel.size() == samples);
    if (kernel.size() != samples)
    {
        return std::vector<double>();
    }
    return kernel;
}

/**
 * @brief Applies Gaussian smoothing to an array of Cell objects.
 *
 * This function calculates a smoothed height for each Cell object in the input array by convolving
 * the heights with a Gaussian kernel. The kernel is generated using the provided sigma and samples.
 *
 * @param values The array of Cell objects to be smoothed. This array is passed by reference to allow
 *               modifications to the objects.
 * @param sigma The standard deviation of the Gaussian distribution used to generate the kernel.
 * @param samples The number of samples to use in the Gaussian kernel.
 *
 * @return void
 */
void gaussSmoothen(std::array<Cell, numBin> &values, double sigma, int samples)
{
    auto kernel = gaussKernel(samples, sigma);
    int sampleSide = samples / 2;
    unsigned long ubound = values.size();

    for(long i = 0; i < ubound; ++i){
        double smoothed = 0;
        for(long j = i - sampleSide; j <= i + sampleSide; ++j){
            if(j >= 0 && j < ubound){
                int sampleWeightIndex = sampleSide + (j - i);
                smoothed += kernel[sampleWeightIndex] * values[j].getHeight();
            }
        }
        values[i].updateSmoothed(smoothed);    
    }
}


