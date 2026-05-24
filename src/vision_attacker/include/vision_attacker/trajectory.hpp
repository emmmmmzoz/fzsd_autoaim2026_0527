#pragma once
#include <chrono>

// Solve ballistic/pitch given x,z,muzzle speed, air drag and lag time.
double solveTrajectory(double &flyTime, const double x, const double z, const double lagTime, const double muzzleSpeed, const double air_k);
