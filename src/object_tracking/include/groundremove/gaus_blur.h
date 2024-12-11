#ifndef GAUS_BLUR_H
#define GAUS_BLUR_H

#include "ground_removal.h"

#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <cassert>

double gauss(double sigma, double x);

std::vector<double> gaussKernel(int sample, double sigma);

void gaussSmoothen(std::array<Cell, numBin>& values, double sigma, int samples);

#endif // GAUS_BLUR_H