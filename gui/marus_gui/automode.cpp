#include "automode.h"
#include "mainwindow.h"
#include "./ui_automode.h"
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <QString>
#include <QTimer>
#include <cmath>
#include <std_msgs/Float32MultiArray.h>
#include "tuningplot1.h"
#include <utility>
#include <vector>
#include "pwmpublisher.h"
#include <chrono>
#include <fstream>

// Constructor
PIDController::PIDController(float p, float i, float d)
    : Kp(p), Ki(i), Kd(d), prev_error(0), integral(0) {}

// Set Kp gain
void PIDController::setKp(float p) {
    Kp = p;
}

// Set Ki gain
void PIDController::setKi(float i) {
    Ki = i;
}

// Set Kd gain
void PIDController::setKd(float d) {
    Kd = d;
}

// Clear integral term
void PIDController::clearIntegral() {
    integral = 0.0;
}

// Clear previous error term
void PIDController::clearPrevError() {
    prev_error = 0.0;
}

// Function to compute the PID output
double PIDController::compute(double setpoint, double actual) {
    // Calculate the error term
    double error = actual - setpoint;
    double current_time = ros::Time::now().toSec(); // Current time in seconds (for ROS)
    double dt = current_time - prev_time;          // Time difference since last update
    prev_time = current_time;                      // Update previous time

    // Define thresholds
    const double maxErrorLimit = 500.0;            // Maximum allowable error limit
    const double targetReachedThreshold = 5.0;     // Threshold for considering the target reached

    // Limit the error to the max allowable value
    if (error > maxErrorLimit) {
        error = maxErrorLimit;
    } else if (error < -maxErrorLimit) {
        error = -maxErrorLimit;
    }

    // Compute the integral term with anti-windup (clamping the integral)
    integral += error * dt;
    const double integralLimit = 5.0;
    if (integral > integralLimit) {
        integral = integralLimit;
    } else if (integral < -integralLimit) {
        integral = -integralLimit;
    }

    // Compute the derivative term using time-based differentiation
    double derivative = (error - prev_error) / dt;

    // Update previous error
    prev_error = error;

    // Compute the PID output
    double rawOutput = Kp * error + Ki * integral + Kd * derivative;

    double rawOutputLimit = 1000;
    float returnVal;

    if (fabs(rawOutput) > rawOutputLimit)
    {
        returnVal = (rawOutput > 0) ? 1.0 : -1.0;
    }
    else
    {
        returnVal = std::tanh(rawOutput / rawOutputLimit);
    }


//    ROS_INFO("Error:%f  p:%f  i:%f  d:%f finalretvalue: %f])",
//             error, Kp * error, Ki * integral,
//             Kd * derivative, returnVal);

    return returnVal;

}

// Constructor
automode::automode(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::automode),
    nh(new ros::NodeHandle),
    plotDialog(new tuningplot1(this)),
    isPidActive(false), // Initialize PID state as inactive
    pwmValues(3,0),
    rotations(0)
{
    ui->setupUi(this);

    // Initialize the ROS publisher for /pwm_out topic
    pwm_publisher = nh->advertise<std_msgs::Float32MultiArray>("marus_boat/pwm_out", 10);

    gpsSubscriber = nh->subscribe("/marus_boat/gps", 100, &automode::gpsCallback, this);

    imuSubscriber = nh->subscribe("/marus_boat/imu", 100, &automode::imuCallback, this);

    // Initialize AsyncSpinner with 2 threads
    spinner = new ros::AsyncSpinner(2);
    spinner->start(); // Start the spinner

    // Initialize the QTimer for continuous publishing
    publishTimer = new QTimer(this);
    connect(publishTimer, &QTimer::timeout, this, &automode::autoPublishValuesPeriodically);
    publishTimer->start(100);  // 100 ms interval for publishing


    // Initialize PID Controllers for steering and throttle
    steeringPID = new PIDController(68, 28, 0);
    throttlePID = new PIDController(100, 50, 0);

    automode::initializeLogFile(); // Initialize the log file with a header

    plotDialog->show(); // Show the plot once at the start
}

// Destructor
automode::~automode()
{
    delete ui;
}

// ROS Callback to handle /tf messages
void automode::tfCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    // Check if we have at least one transform in the message
    if (!msg->transforms.empty()) {

        const auto &transform = msg->transforms[0];

        // Check the frame IDs to ensure we are receiving the correct data
        if (transform.header.frame_id == "marus_boat/base_link" && transform.child_frame_id == "marus_boat/gps_frame") {
            // Extract translation and rotation
            const auto &translation = transform.transform.translation;
            const auto &rotation = transform.transform.rotation;

//            // Update the labels within the boatPosition QFrame
//            // Directly update the labels in the boatPosition QFrame
//            ui->xTranslation->setText(QString::number(translation.x, 'f', 16));
//            ui->yTranslation->setText(QString::number(translation.y, 'f', 16));
//            ui->zTranslation->setText(QString::number(translation.z, 'f', 16));
//            ui->xRotation->setText(QString::number(rotation.x, 'f', 16));
//            ui->yRotation->setText(QString::number(rotation.y, 'f', 16));
//            ui->zRotation->setText(QString::number(rotation.z, 'f', 16));
//            ui->wRotation->setText(QString::number(rotation.w, 'f', 16));
        }
    }
}

double currentLat = 0;
double currentLon = 0;
int targetIndex;
void automode::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    // Check if the NavSatFix message contains valid GPS data
    if (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        // Extract latitude, longitude, and altitude
        const auto &latitude = msg->latitude;
        const auto &longitude = msg->longitude;
        const auto &altitude = msg->altitude;

        // Update the labels within the boatPosition QFrame
        ui->latitude->setText(QString::number(latitude, 'f', 16) + "°");  // Latitude in degrees
        ui->longitude->setText(QString::number(longitude, 'f', 16) + "°");  // Longitude in degrees
        ui->altitude->setText(QString::number(altitude, 'f', 2) + " m");    // Altitude in meters

        // display the GPS fix type for debugging or user information
        QString fixStatus;
        switch (msg->status.status) {
            case sensor_msgs::NavSatStatus::STATUS_NO_FIX:
                fixStatus = "No Fix";
                break;
            case sensor_msgs::NavSatStatus::STATUS_FIX:
                fixStatus = "2D/3D Fix";
                break;
            case sensor_msgs::NavSatStatus::STATUS_SBAS_FIX:
                fixStatus = "SBAS Fix";
                break;
            case sensor_msgs::NavSatStatus::STATUS_GBAS_FIX:
                fixStatus = "GBAS Fix";
                break;
            default:
                fixStatus = "Unknown";
                break;
        }
        ui->gpsFixStatus->setText(fixStatus);  // Display the GPS fix status

        currentLat = msg->latitude;
        currentLon = msg->longitude;
        QString targetText;

        // Check if we have reached the target waypoint
        if (!targets.empty()) {
            targetIndex = 0;  // Get the first target
            double distance = calculateDistance(currentLat, currentLon, targets[targetIndex].first, targets[targetIndex].second);

            ui->distToWaypoint->setText(QString::number(distance, 'f', 2) + " m");

//            ROS_INFO("Distance to waypoint: %f km (Current: [%f, %f], Target: [%f, %f])",
//                     distance, currentLat, currentLon,
//                     targets[targetIndex].first, targets[targetIndex].second);

            if (distance < 0.1f) {  // Threshold distance for waypoint reach
                ROS_INFO("Reached waypoint: (%f, %f)", targets[targetIndex].first, targets[targetIndex].second);
                targets.erase(targets.begin());  // Remove the reached waypoint

                resetPIDValues();

                targetText = QString("Target Reached: %1, %2").arg(targets[targetIndex].first, 0, 'f', 6).arg(targets[targetIndex].second, 0, 'f', 6);

                // Append the formatted text to the QPlainTextEdit
                ui->targetStatusText->appendPlainText(targetText);
            }
        }
    }

    if(isPidActive)
    {
        automode::logDataToFile();
    }

}

double heading;
double roll;
double pitch;
void automode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // Extract the quaternion orientation from the IMU message
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

    // 1. Calculate Heading (Yaw) from quaternion
    heading = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // 2. Calculate Roll from quaternion
    roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));

    // 3. Calculate Pitch from quaternion
    pitch = std::asin(2.0 * (qw * qy - qz * qx));

    // 4. Calculate Linear Speed
    double linear_speed = std::sqrt(std::pow(msg->linear_acceleration.x, 2) +
                                    std::pow(msg->linear_acceleration.y, 2));

    // 5. Extract Angular Velocity
    double angular_velocity_x = msg->angular_velocity.x;
    double angular_velocity_y = msg->angular_velocity.y;
    double angular_velocity_z = msg->angular_velocity.z;

    // Print or update the GUI with the calculated values
    // Update the labels or use these values as needed
    ui->heading->setText(QString::number(heading, 'f', 2) + " rad");   // Heading in radians
    ui->roll->setText(QString::number(roll, 'f', 2) + " rad");         // Roll in radians
    ui->pitch->setText(QString::number(pitch, 'f', 2) + " rad");       // Pitch in radians
    ui->speed->setText(QString::number(linear_speed, 'f', 2) + " m/s"); // Linear speed (m/s)

    // Format the components into a single string with units for angular velocity
    QString angular_velocity_str = QString("X: %1 rad/s, Y: %2 rad/s, Z: %3 rad/s")
        .arg(angular_velocity_x, 0, 'f', 4)  // 'f' for fixed-point, 2 digits after decimal
        .arg(angular_velocity_y, 0, 'f', 4)
        .arg(angular_velocity_z, 0, 'f', 4);

    // Set the concatenated string to the angular velocity label
    ui->angularvelocity->setText(angular_velocity_str);

    currentHeading = heading;

    if( isPidActive )
    {
        automode::logDataToFile();
    }

}

QVector<QPointF> headingErrors;
QVector<QPointF> distanceErrors;
QVector<QPointF> headingCurrent;
QVector<QPointF> headingDesired;
QVector<QPointF> posCurrent;
QVector<QPointF> posDesired;
int timeStep = 0;  // To track time for plotting
// Calculate the desired heading angle towards the target
double desiredHeading;
double headingError;
double distance;
void automode::computePIDAndPublish(int targetIndex, double currentLat, double currentLon)
{
    if (!isPidActive || targets.empty()) {
        pwmValues[0] = 0;
        pwmValues[1] = 0;
        pwmValues[2] = 0;
        // Skip computation if PID is not active
        return;
    }

    double targetLat = targets[targetIndex].first;
    double targetLon = targets[targetIndex].second;

    // Calculate distance to the target
    distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);

    desiredHeading = -normalizeAngle(calculateDesiredHeading(currentLat, currentLon, targetLat, targetLon));

    // Calculate the heading error
    headingError = normalizeAngle(currentHeading - desiredHeading);

    // Store heading and distance errors for plotting
    //headingErrors.append(QPointF(timeStep, angleDifference));
    headingErrors.append(QPointF(timeStep, headingError));
    distanceErrors.append(QPointF(timeStep, distance));
    headingCurrent.append(QPointF(timeStep, normalizeAngle(currentHeading)));
    headingDesired.append(QPointF(timeStep, normalizeAngle(desiredHeading)));
    posCurrent.append(QPointF(timeStep, distance));
    posDesired.append(QPointF(timeStep, 0));

    // Increment the time step for plotting
    timeStep++;

    // Define a larger threshold for heading alignment to stop small oscillations
    const double headingThreshold = 0.02; // ~2.86 degrees (adjust as necessary)
    const double minHeadingErrorForCorrection = 0.01; // A smaller threshold to consider for heading alignment

    double steer = 0.0; // Initialize steering output

    // Variables to hold the adjusted angles
    double adjustedDesired, adjustedCurrent;

    processAngles(desiredHeading, normalizeAngle(currentHeading), adjustedDesired, adjustedCurrent);

    // If the heading error is large enough, calculate steering using PID controller
    steer = steeringPID->compute(adjustedDesired*100, adjustedCurrent*100);

    // For intermediate waypoints, we won't slow down
    bool isFinalTarget = (targetIndex == targets.size() - 1); // Check if this is the last waypoint
    double throttle;
    if (isFinalTarget) {
        // Reduce speed when approaching the final target
        throttle = throttlePID->compute(0, distance * 10);
    } else {
        // For intermediate waypoints, keep the throttle constant
        throttle = 1.0;  // Full speed for intermediate waypoints
    }

    ROS_INFO("headingError: %f", headingError);

    if (std::abs(headingError) > M_PI/2) {
        // Gradually reduce the throttle to a minimal value for a U-turn
        throttle = throttle * 0.25; // Reduce throttle by 25%
        ROS_INFO("throttleReduced to %f", throttle);
        if (throttle < 0.2) { // Minimum throttle limit for U-turn
            throttle = 0.2;  // Prevent throttle from dropping too low
        }
    }

    // Publish the PWM values
    pwmValues[0] = throttle;
    pwmValues[1] = throttle;
    pwmValues[2] = steer;

    ui->throttleGUI->setText(QString::number(throttle, 'f', 2));
    ui->steerGUI->setText(QString::number(steer, 'f', 2));
    //ROS_INFO("Desired Heading: %f rad, Current Heading: %f rad, Heading Error: %f rad, Steer: %f, Throttle: %f",
    //         desiredHeading, currentHeading, headingError, steer, throttle);

    //ROS_INFO("Desired: %f, Current: %f, headingError: %f Steer: %f", desiredHeading, currentHeading, headingError, steer);


    plotDialog->updatePlot(headingErrors, distanceErrors, posCurrent, posDesired, headingCurrent, headingDesired); // Update the existing plot

    automode::logDataToFile();
}

void automode::processAngles(double desired, double current, double &adjustedDesired, double &adjustedCurrent) {

    // Detect wrap-around in desired angle (crossing from +π to -π or -π to +π)
    if (previousDesired > 2.0  && desired < -2.0 ) {
        // Crossing from +π to -π
        rotations = std::min(rotations + 1, 1);
        crossingPositive = true;
    } else if (previousDesired < -2.0  && desired > 2.0 ) {
        // Crossing from -π to +π
        rotations = std::max(rotations - 1, -1);
        crossingPositive = false;
    }

    // Update previousDesired for next iteration
    previousDesired = desired;

    // Adjust desired angle based on number of rotations
    adjustedDesired = desired + (rotations * 2 * M_PI);  // Adjust by 2π * rotations to keep it consistent with desired range

    // Adjust current angle based on rotations and crossing direction
    adjustedCurrent = current;

    if (rotations != 0) {
        if (crossingPositive) {
            // When crossing from +π to -π, adjust current angle if it hasn't wrapped yet
            if (current > 0) {
                adjustedCurrent = current + ((rotations - 1) * 2 * M_PI);
            } else {
                adjustedCurrent = current + (rotations * 2 * M_PI);
                rotations = 0; // Reset rotations when the current angle crosses into the correct range
            }
        } else {
            // When crossing from -π to +π, adjust current angle if it hasn't wrapped yet
            if (current < 0) {
                adjustedCurrent = current + ((rotations + 1) * 2 * M_PI);
            } else {
                rotations = 0; // Reset rotations when the current angle crosses into the correct range
                adjustedCurrent = current + (rotations * 2 * M_PI);
            }
        }
    }


}


// Function to calculate the shortest angle difference
double automode::calculateAngleDifference(double desired, double current) {
    // Normalize both angles to the range [-180, 180]
    double normalizedDesired = -normalizeAngle(desired);
    double normalizedCurrent = normalizeAngle(current);

    // Calculate the difference and wrap it to the shortest path
    double angleDiff = normalizedDesired - normalizedCurrent;
    //if (angleDiff > 180) angleDiff -= 360;
    //if (angleDiff < -180) angleDiff += 360;

    return angleDiff;
}

double automode::calculateDesiredHeading(double lat1, double lon1, double lat2, double lon2)
{
    // Calculate the desired heading towards the target point
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double y = sin(dlon) * cos(lat2 * M_PI / 180.0);
    double x = cos(lat1 * M_PI / 180.0) * sin(lat2 * M_PI / 180.0) -
               sin(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * cos(dlon);
    double desiredHeading = atan2(y, x);

    // Normalize the heading to the range [-π, π]
    return normalizeAngle(desiredHeading);
}

double automode::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double earthRadiusm = 6371000.0;

    // Convert degrees to radians
    double lat1Rad = lat1 * M_PI / 180.0;
    double lon1Rad = lon1 * M_PI / 180.0;
    double lat2Rad = lat2 * M_PI / 180.0;
    double lon2Rad = lon2 * M_PI / 180.0;

    // Haversine formula
    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1Rad) * cos(lat2Rad) *
               sin(dLon / 2) * sin(dLon / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // Distance in meters
    return earthRadiusm * c;
}


double automode::normalizeAngle(double angle)
{
//    while (angle > M_PI) angle -= 2 * M_PI;
//    while (angle < -M_PI) angle += 2 * M_PI;
//    return angle;

    int maxIterations = 100;  // Limit to avoid infinite loop

    int iterationCount = 0;
    while (angle > M_PI && iterationCount < maxIterations) {
        angle -= 2 * M_PI;
        iterationCount++;
    }

    iterationCount = 0;  // Reset for the second loop
    while (angle < -M_PI && iterationCount < maxIterations) {
        angle += 2 * M_PI;
        iterationCount++;
    }

    if (iterationCount >= maxIterations) {
        ROS_WARN("Angle normalization took too many iterations, possible issue with input.");
    }

    return angle;

}

void automode::autoPublishValuesPeriodically()
{
    // Create a Float32MultiArray message to store slider values
    std_msgs::Float32MultiArray msg;

    computePIDAndPublish(targetIndex, currentLat, currentLon);

    msg.data = pwmValues;

    // Publish the message to the /pwm_out topic
    pwm_publisher.publish(msg);
}

void automode::on_kpGain_textChanged()
{
    QString text = ui->kpGain->text();
    float kp = text.toDouble();

    steeringPID->setKp(kp);
}

void automode::on_kiGain_textChanged()
{
    QString text = ui->kiGain->text();
    float ki = text.toDouble();

    steeringPID->setKi(ki);
}

void automode::on_kdGain_textChanged()
{
    QString text = ui->kdGain->text();
    float kd = text.toDouble();

    steeringPID->setKd(kd);
}

void automode::on_kpGain_throttle_textChanged()
{
    QString text = ui->kpGain_throttle->text();
    float kp = text.toDouble();

    throttlePID->setKp(kp);
}

void automode::on_kiGain_throttle_textChanged()
{
    QString text = ui->kpGain_throttle->text();
    float ki = text.toDouble();

    throttlePID->setKi(ki);
}

void automode::on_kdGain_throttle_textChanged()
{
    QString text = ui->kpGain_throttle->text();
    float kd = text.toDouble();

    throttlePID->setKd(kd);
}

void automode::on_Start_released()
{
    isPidActive = true; // Activate PID controller

    // Clear all data vectors
    headingErrors.clear();
    distanceErrors.clear();
    headingCurrent.clear();
    headingDesired.clear();
    posCurrent.clear();
    posDesired.clear();
    timeStep = 0;  // Reset the time step counter
    plotDialog->clearGraphs();

    ui->Start->setEnabled(false);  // Disable Start button
    ui->Stop->setEnabled(true);   // Enable Stop button
}

void automode::on_Stop_released()
{
    //targets.clear(); // Clear all waypoints

    isPidActive = false; // deactivate PID controller

    resetPIDValues();

    ui->Start->setEnabled(true);  // Enable Start button
    ui->Stop->setEnabled(false); // Disable Stop button
}

void automode::on_addTargetPos_released()
{
    // Extract text from QLineEdit and convert to float
    double lat = ui->targetLatitude->text().toDouble(); // Convert latitude to double
    double lon = ui->targetLongitude->text().toDouble(); // Convert longitude to double

    // Add the target coordinates to the list
    targets.push_back({lat, lon});

    // Format the text to display with floating-point values
    QString targetText = QString("Target 1: %1, %2")
                             .arg(lat, 0, 'f', 6)  // Latitude with 6 decimal places
                             .arg(lon, 0, 'f', 6); // Longitude with 6 decimal places

    // Append the formatted text to the QPlainTextEdit
    ui->targetStatusText->appendPlainText(targetText);

}

void automode::on_clearAllTargetPos_released()
{
    targets.clear(); // Clear all waypoints

    resetPIDValues();

    ui->targetStatusText->clear();
}

void automode::resetPIDValues()
{
    // Reset PWM values to zero
    pwmValues[0] = 0;
    pwmValues[1] = 0;
    pwmValues[2] = 0;

    steeringPID->clearIntegral();
    steeringPID->clearPrevError();

    throttlePID->clearIntegral();
    throttlePID->clearPrevError();

    ROS_INFO("All targets have been cleared.");

}

void automode::on_loadSavedPos_released()
{
    QString targetText;

    //U-track
    targets.push_back({45.00033187f, 15.00085662f});
    targets.push_back({45.00041886f, 15.00074625f});
    targets.push_back({45.00057393f, 15.00040801f});
    targets.push_back({45.00059663f, 15.00006978f});
    targets.push_back({45.00045038f, 14.99985794f});
    targets.push_back({45.00025875f, 14.99989532f});
    targets.push_back({45.00005956f, 15.00016769f});
    targets.push_back({44.99995618f, 15.00044540f});

    // zigzag-track:
//    targets.push_back({45.000188150000000, 15.000991910000000});
//    targets.push_back({45.000280651650897, 15.001117926875928});
//    targets.push_back({45.000439225909574, 15.001052584792113});
//    targets.push_back({45.000554852973188, 15.001218273647501});
//    targets.push_back({45.000762981687700, 15.001117926875928});
//    targets.push_back({45.000908341424818, 15.001292950314717});
//    targets.push_back({45.001080130205054, 15.001262612918660});

    for (int i = 0; i < targets.size(); ++i) {
        double lat = targets[i].first;   // Extract latitude
        double lon = targets[i].second;  // E
        
        
        
        // Iterate over all targets and format thextract longitude
        targetText.append(QString("Target %1: %2, %3\n")
                           .arg(i + 1)       // Target number
                           .arg(lat, 0, 'f', 6)  // Latitude with 6 decimal places
                           .arg(lon, 0, 'f', 6)); // Longitude with 6 decimal places
    }

    ui->targetStatusText->appendPlainText(targetText);
}


void automode::initializeLogFile() {
    std::ofstream logFile;
    logFile.open("ui_data_log.csv");

    if (logFile.is_open()) {
        logFile << "Timestamp,CurrentLat,CurrentLon,CurrentHeading,DesiredHeading,Roll,Pitch,"
                << "Distance,DesiredDistance,"
                << "PWM1,PWM2,PWM3,TargetLat,TargetLon,DistanceToTarget,HeadingError\n";
        logFile.close();
    } else {
        ROS_ERROR("Unable to initialize log file");
    }
}

double desiredDistance = 0;

void automode::logDataToFile() {
    std::ofstream logFile;

    logFile.open("ui_data_log.csv", std::ios::app);

    if (logFile.is_open()) {
        // Log timestamp
        double current_time = ros::Time::now().toSec();
        logFile << std::fixed << std::setprecision(6) << current_time << ",";

        // Log GPS data
        logFile << currentLat << "," << currentLon << ",";

        // Log IMU data
        logFile << currentHeading << "," << desiredHeading << "," << roll << "," << pitch << ",";

        logFile << distance << "," << desiredDistance << ",";

        // Log PWM values
        logFile << pwmValues[0] << "," << pwmValues[1] << "," << pwmValues[2] << ",";

        // Log target waypoint data
        if (!targets.empty()) {
            logFile << targets[targetIndex].first << "," << targets[targetIndex].second << ",";
        } else {
            logFile << "N/A,N/A,";
        }

        // Log distance to target
        logFile << calculateDistance(currentLat, currentLon, targets[targetIndex].first, targets[targetIndex].second) << ",";

        // Log heading error
        logFile << headingError << "," <<"\n";

        logFile.close();
    } else {
        ROS_ERROR("Unable to open log file");
    }
}
