#ifndef AUTOMODE_H
#define AUTOMODE_H

#include <QDialog>
#include <QVector>
#include <QPointF>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <QTimer>
#include <vector>
#include <utility>
#include "tuningplot1.h"
#include "ui_tuningplot1.h"
#include "ui_automode.h"


class TuningPlot;

class PIDController {
public:
    PIDController(float p, float i, float d);
    double compute(double setpoint, double actual);
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void clearIntegral();
    void clearPrevError();

private:
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain
    double prev_error;  // Previous error for derivative term
    double integral;  // Integral term
    double prev_time;  // Time at the previous update
};

// Main class for Auto Mode
class automode : public QDialog
{
    Q_OBJECT

public:
    explicit automode(QWidget *parent = nullptr);
    ~automode();

private:
    Ui::automode *ui;

    ros::NodeHandle *nh;  // ROS NodeHandle
    ros::Publisher pwm_publisher;
    ros::Subscriber gpsSubscriber;  // ROS Subscriber for GPS data
    ros::Subscriber imuSubscriber;  // ROS Subscriber for IMU data
    ros::AsyncSpinner *spinner; // AsyncSpinner for handling callbacks
    QTimer *publishTimer;  // Timer for publishing PWM values periodically
    tuningplot1 *plotDialog;  // Plotting dialog for tuning PID controllers

    PIDController *steeringPID;  // PID controller for steering
    PIDController *throttlePID;  // PID controller for throttle

    bool isPidActive;  // Boolean flag to indicate if PID is active
    double currentHeading;  // Current heading based on IMU data

    std::vector<std::pair<double, double>> targets;  // List of target waypoints (latitude, longitude)
    std::vector<float> pwmValues;  // PWM values for controlling the vehicle

    QVector<QPointF> headingErrors;  // Store heading errors for plotting
    QVector<QPointF> distanceErrors;  // Store distance errors for plotting
    QVector<QPointF> headingCurrent;  // Store current heading values for plotting
    QVector<QPointF> headingDesired;  // Store desired heading values for plotting
    QVector<QPointF> posCurrent;  // Store current position values for plotting
    QVector<QPointF> posDesired;  // Store desired position values for plotting

    int timeStep;  // Time step for plotting
    double desiredHeading;  // Desired heading towards the target
    int rotations;  // Track rotations for heading correction
    bool crossingPositive;  // Track whether heading is crossing from positive to negative
    double previousDesired;  // Store the previous desired heading

    // ROS Callbacks
    void tfCallback(const tf2_msgs::TFMessage::ConstPtr &msg);  // Callback for /tf messages
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);  // Callback for GPS messages
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);  // Callback for IMU messages

    // Helper functions
    double calculateDistance(double lat1, double lon1, double lat2, double lon2);  // Calculate distance between two GPS coordinates
    void resetPIDValues();  // Reset PID controller values
    void computePIDAndPublish(int targetIndex, double currentLat, double currentLon);  // Compute PID and publish PWM values
    double normalizeAngle(double angle);  // Normalize angle between -π and π
    double calculateAngleDifference(double desired, double current);  // Calculate the shortest angle difference
    double calculateDesiredHeading(double lat1, double lon1, double lat2, double lon2);  // Calculate the desired heading
    void logDataToFile();
    void initializeLogFile();
    void processAngles(double desired, double current, double &adjustedDesired, double &adjustedCurrent);  // Adjust angles for PID control

private slots:
    void autoPublishValuesPeriodically();  // Periodic function to publish PWM values
    void on_kpGain_textChanged();
    void on_kiGain_textChanged();
    void on_kdGain_textChanged();
    void on_kpGain_throttle_textChanged();
    void on_kiGain_throttle_textChanged();
    void on_kdGain_throttle_textChanged();
    void on_Start_released();
    void on_Stop_released();
    void on_addTargetPos_released();
    void on_clearAllTargetPos_released();
    void on_loadSavedPos_released();

};

#endif // AUTOMODE_H
