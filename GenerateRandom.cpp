#include "GenerateRandom.h"
#include <cmath>
#include "Vector.h"


std::default_random_engine gen(12345);
std::uniform_real_distribution<double> unif(0.0, 1.0);

void boxMuller(double& z0, double& z1) {
    double u1 = unif(gen);
    double u2 = unif(gen);    
    z0 = 0.3 * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2); 
    z1 = 0.3 * sqrt(-2.0 * log(u1)) * sin(2.0 * M_PI * u2); 
}

Vector random_cos(const Vector &N) {
    float r1 = unif(gen), r2 = unif(gen), phi = 2.0f * M_PI * r1;
    float x = cos(phi) * sqrt(1.0f - r2), y = sin(phi) * sqrt(1.0f - r2), z = sqrt(r2);
    
    Vector T1;
    if (fabs(N[0]) < fabs(N[1]) && fabs(N[0]) < fabs(N[2]))
        T1 = Vector(0, -N[2], N[1]); 
    else if (fabs(N[1]) < fabs(N[2]))
        T1 = Vector(-N[2], 0, N[0]);
    else
        T1 = Vector(-N[1], N[0], 0); 
    T1.normalize();
    Vector T2 = cross(N, T1);
    Vector Result = x * T1 + y * T2 + z * N; Result.normalize();
    return Result;
}
