#include <QApplication>
#include <QTimer>
#include "mainwindow.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    // Initialize the ROS node before the Qt application
    ros::init(argc, argv, "ros_gui_node");

    QApplication a(argc, argv);
    MainWindow w;

    // Use a QTimer to periodically call ros::spinOnce() to process ROS callbacks
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [](){
        ros::spinOnce();  // This will handle any ROS messages/callbacks
    });
    timer.start(67);  // Call ros::spinOnce() every 67 ms ¬ 15Hz

    w.show();
    return a.exec();  // Start the Qt event loop to keep the GUI responsive
}
