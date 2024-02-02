#include "Boat.h"


/**
 * Calculate distance from actual point to desired point.
 *
 * @param const double targetX, targeted x direction.
 * @param const double x, actual x direction.
 * @param const double targetY, targeted y direction.
 * @param const double y, actual y direction.
 *
 * @return double distance, distance between targeted and actual point.
 */
double calculateDistance(const double targetX, const double x, const double targetY, const double y) {
    // Calculate the squared differences.
    double deltaX = x - targetX;
    double deltaY = y - targetY;

    // Calculate the sum of squared differences.
    double sumOfSquares = deltaX * deltaX + deltaY * deltaY;

    // Calculate the square root to get the distance.
    double distance = sqrt(sumOfSquares);

    return distance;
}


/**
 * Calculate heading of the boat.
 *
 * @param const double Ux, control action in x direction.
 * @param const double Uy, control action in y direction.
 *
 * @return double angleInDegrees, heading for the boat.
 */
double calculateYawAngle(const double Ux, const double Uy) {
    // Get the absolute values of Ux and Uy.
    double absUx = fabs(Ux);
    double absUy = fabs(Uy);

    // Calculate the arctangent of Uy/Ux and convert to degrees.
    double angleInRadians = atan2(absUy, absUx);

    // Convert radians to degrees.
    double angleInDegrees = angleInRadians * (180.0 / M_PI);

    // Adjust the angle based on the signs of Ux and Uy.
    if (Ux >= 0 && Uy >= 0) angleInDegrees = 90 - angleInDegrees;       // Quadrant 1.
    else if (Ux < 0 && Uy >= 0) angleInDegrees = angleInDegrees - 90;   // Quadrant 2.
    else if (Ux < 0 && Uy < 0) angleInDegrees = -1*angleInDegrees - 90; // Quadrant 3.
    else if (Ux >= 0 && Uy < 0) angleInDegrees = angleInDegrees + 90;   // Quadrant 4.
    else angleInDegrees = 0;

    return angleInDegrees;
}


/**
 * Function for outer loop controller.
 *
 * @param const double setpointX, targeted x direction.
 * @param const double setpointY, targeted y direction.
 * @param const double sensorX, actual x direction.
 * @param const double sensorY, actual y direction.
 *
 * @return double setpointHeading, desired heading for the boat.
 */
double outerController(const double setpointX, const double setpointY, const double sensorX, const double sensorY) {
    // Error in Ux.
    double Ux = setpointX - sensorX;

    // Error in Uy.
    double Uy = setpointY - sensorY;

    // Get the desired heading of the boat.
    double setpointHeading = calculateYawAngle(Ux, Uy);

  return setpointHeading;
}



/**
 * // Function for outer loop controller.
 *
 * @param const double distance, distance between targeted and actual point.
 * @param const double setpointYaw, desired heading.
 * @param const double yaw, actual heading.
 * @param double *normSpeed, normalized speed for left and right motor.
 */
void innerController(const double distance, const double setpointYaw, const double yaw, double *normSpeed) {
    // Error in yaw.
    double errorYaw = setpointYaw - yaw;

    // Get the left and motor speed stored in header file.
    int column = 50*distance;
    int row = 25 + 25 * errorYaw;

    // Ensure that the value is within the range [0, 100].
    if (row < 0) row = 0;
    else if (row > 50) row = 50;

    if (column < 0) column = 0;
    else if (column > 50) column = 50;

    // Pack speed into array.
    normSpeed[0] = normLeftMotorSpeed[column][row] * 255;
    normSpeed[1] = normRightMotorSpeed[column][row] * 255;
}
