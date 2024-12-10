#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <QTimer>  // Include QTimer for periodic publishing

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , nh(new ros::NodeHandle)  // Initialize ROS NodeHandle
    , sliderValues(3, 0)  // Initialize the data array with 3 zeros
{
    ui->setupUi(this);

    // Initialize the ROS publisher for /pwm_out topic
    pwm_publisher = nh->advertise<std_msgs::Float32MultiArray>("marus_boat/pwm_out", 10);

    // Log initialization success (optional)
    ROS_INFO("MainWindow initialized and publisher setup for marus_boat/pwm_out");

    // Initialize the QTimer for continuous publishing
    publishTimer = new QTimer(this);
    connect(publishTimer, &QTimer::timeout, this, &MainWindow::publishValuesPeriodically);
    publishTimer->start(100);  // 100 ms interval for publishing (adjust as needed)
}

MainWindow::~MainWindow()
{
    // Clean up resources
    delete ui;
    delete nh;
}

void MainWindow::on_verticalSlider_valueChanged(int value)
{
    // Update the first index of the array with the new value from the slider
    sliderValues[0] = value/10.0f;
}

void MainWindow::on_verticalSlider_2_valueChanged(int value)
{
    // Update the second index of the array with the new value from the slider
    sliderValues[1] = value/10.0f;
}

void MainWindow::on_verticalSlider_3_valueChanged(int value)
{
    // Update the third index of the array with the new value from the slider
    sliderValues[2] = value/10.0f;
}

void MainWindow::on_pushButton_clicked()
{
    // Reset all sliders to 0 when the button is clicked
    ui->verticalSlider->setValue(0);
    ui->verticalSlider_2->setValue(0);
    ui->verticalSlider_3->setValue(0);

    // Reset the data array to zeros
    sliderValues = {0, 0, 0};

    // Publish updated values after reset
    publishSliderValues();

    // Log the reset action (optional)
    ROS_INFO("All sliders reset to 0 and values published.");
}

void MainWindow::publishSliderValues()
{
    // Create a Float32MultiArray message to store slider values
    std_msgs::Float32MultiArray msg;

    msg.data = sliderValues;

    // Publish the message to the /pwm_out topic
    pwm_publisher.publish(msg);

    // Log the published values (optional for debugging)
//    ROS_INFO("Published slider values: [%f, %f, %f]",
//             static_cast<float>(sliderValues[0]/10.0f),
//             static_cast<float>(sliderValues[1]/10.0f),
//             static_cast<float>(sliderValues[2]/10.0f));
}

void MainWindow::publishValuesPeriodically()
{
    // This function will be called every 100 ms by the QTimer
    publishSliderValues();  // Publish the current slider values
}


