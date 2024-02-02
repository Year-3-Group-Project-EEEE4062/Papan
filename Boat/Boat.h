#include <math.h>
#include "motorSpeed.h"


double calculateDistance(const double targetX, const double x, const double targetY, const double y);
double calculateYawAngle(const double Ux, const double Uy);
double outerController(const double setpointX, const double setpointY, const double sensorX, const double sensorY);
void innerController(const double distance, const double setpointYaw, const double yaw, double *normSpeed);
