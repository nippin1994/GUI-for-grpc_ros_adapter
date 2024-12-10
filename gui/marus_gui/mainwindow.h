#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>  // For using std::vector
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_verticalSlider_valueChanged(int value);
    void on_verticalSlider_2_valueChanged(int value);
    void on_verticalSlider_3_valueChanged(int value);
    void on_pushButton_clicked();
    void publishSliderValues();
    void publishValuesPeriodically();

private:
    Ui::MainWindow *ui;
    ros::NodeHandle *nh; // ROS NodeHandle
    ros::Publisher pwm_publisher; // ROS Publisher
    std::vector<float> sliderValues; // Array to store slider values
    QTimer *publishTimer;
};

#endif // MAINWINDOW_H
