#pragma once

#include <random>
#include "Vector.h"
#include <cmath>

extern std :: default_random_engine gen;
extern std::uniform_real_distribution<double> unif;
void boxMuller(double& z0, double& z1);
Vector random_cos(const Vector &N);