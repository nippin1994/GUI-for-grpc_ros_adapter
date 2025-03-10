/********************************************************************************
** Form generated from reading UI file 'automode.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_AUTOMODE_H
#define UI_AUTOMODE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_automode
{
public:
    QFrame *frame;
    QLabel *label;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_2;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *latitude;
    QLabel *longitude;
    QLabel *altitude;
    QLabel *gpsFixStatus;
    QLabel *heading;
    QLabel *roll;
    QLabel *pitch;
    QLabel *speed;
    QLabel *angularvelocity;
    QLabel *distToWaypoint;
    QFrame *frame_2;
    QLabel *label_16;
    QLabel *label_19;
    QLabel *label_20;
    QLabel *label_21;
    QLineEdit *kpGain;
    QLabel *label_22;
    QLabel *label_23;
    QLineEdit *kpGain_throttle;
    QLineEdit *kiGain_throttle;
    QLineEdit *kiGain;
    QLineEdit *kdGain_throttle;
    QLineEdit *kdGain;
    QLabel *label_24;
    QLabel *label_25;
    QLabel *label_26;
    QLabel *label_27;
    QLabel *label_28;
    QFrame *frame_3;
    QLabel *label_15;
    QLabel *label_17;
    QLabel *label_18;
    QLineEdit *targetLatitude;
    QLineEdit *targetLongitude;
    QPushButton *addTargetPos;
    QPushButton *clearAllTargetPos;
    QPushButton *loadSavedPos;
    QPushButton *Start;
    QPushButton *Stop;
    QPlainTextEdit *targetStatusText;

    void setupUi(QDialog *automode)
    {
        if (automode->objectName().isEmpty())
            automode->setObjectName(QString::fromUtf8("automode"));
        automode->resize(644, 379);
        frame = new QFrame(automode);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(20, 220, 601, 151));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        label = new QLabel(frame);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(250, 0, 101, 20));
        QFont font;
        font.setPointSize(10);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        label_3 = new QLabel(frame);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(380, 30, 161, 16));
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        label_3->setFont(font1);
        label_4 = new QLabel(frame);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(220, 30, 161, 16));
        label_4->setFont(font1);
        label_5 = new QLabel(frame);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 30, 161, 16));
        label_5->setFont(font1);
        label_2 = new QLabel(frame);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 50, 71, 16));
        label_6 = new QLabel(frame);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 70, 71, 16));
        label_7 = new QLabel(frame);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 90, 71, 16));
        label_8 = new QLabel(frame);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 110, 71, 16));
        label_9 = new QLabel(frame);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(220, 70, 71, 16));
        label_10 = new QLabel(frame);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(220, 90, 71, 16));
        label_11 = new QLabel(frame);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(220, 50, 71, 16));
        label_12 = new QLabel(frame);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(380, 70, 71, 31));
        label_12->setWordWrap(true);
        label_13 = new QLabel(frame);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(380, 110, 71, 41));
        label_13->setWordWrap(true);
        label_14 = new QLabel(frame);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(380, 50, 71, 16));
        latitude = new QLabel(frame);
        latitude->setObjectName(QString::fromUtf8("latitude"));
        latitude->setGeometry(QRect(80, 50, 131, 16));
        longitude = new QLabel(frame);
        longitude->setObjectName(QString::fromUtf8("longitude"));
        longitude->setGeometry(QRect(80, 70, 131, 16));
        altitude = new QLabel(frame);
        altitude->setObjectName(QString::fromUtf8("altitude"));
        altitude->setGeometry(QRect(80, 90, 131, 16));
        gpsFixStatus = new QLabel(frame);
        gpsFixStatus->setObjectName(QString::fromUtf8("gpsFixStatus"));
        gpsFixStatus->setGeometry(QRect(80, 110, 131, 16));
        heading = new QLabel(frame);
        heading->setObjectName(QString::fromUtf8("heading"));
        heading->setGeometry(QRect(280, 50, 131, 16));
        roll = new QLabel(frame);
        roll->setObjectName(QString::fromUtf8("roll"));
        roll->setGeometry(QRect(280, 70, 131, 16));
        pitch = new QLabel(frame);
        pitch->setObjectName(QString::fromUtf8("pitch"));
        pitch->setGeometry(QRect(280, 90, 131, 16));
        speed = new QLabel(frame);
        speed->setObjectName(QString::fromUtf8("speed"));
        speed->setGeometry(QRect(460, 50, 131, 16));
        angularvelocity = new QLabel(frame);
        angularvelocity->setObjectName(QString::fromUtf8("angularvelocity"));
        angularvelocity->setGeometry(QRect(460, 70, 131, 41));
        angularvelocity->setWordWrap(true);
        distToWaypoint = new QLabel(frame);
        distToWaypoint->setObjectName(QString::fromUtf8("distToWaypoint"));
        distToWaypoint->setGeometry(QRect(460, 120, 131, 16));
        frame_2 = new QFrame(automode);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setGeometry(QRect(410, 10, 221, 191));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        label_16 = new QLabel(frame_2);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(70, 10, 101, 20));
        label_16->setFont(font);
        label_19 = new QLabel(frame_2);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setGeometry(QRect(10, 60, 71, 16));
        label_20 = new QLabel(frame_2);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setGeometry(QRect(10, 90, 71, 16));
        label_21 = new QLabel(frame_2);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setGeometry(QRect(10, 120, 71, 16));
        kpGain = new QLineEdit(frame_2);
        kpGain->setObjectName(QString::fromUtf8("kpGain"));
        kpGain->setGeometry(QRect(70, 60, 41, 23));
        label_22 = new QLabel(frame_2);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setGeometry(QRect(60, 30, 71, 20));
        label_23 = new QLabel(frame_2);
        label_23->setObjectName(QString::fromUtf8("label_23"));
        label_23->setGeometry(QRect(130, 30, 81, 20));
        kpGain_throttle = new QLineEdit(frame_2);
        kpGain_throttle->setObjectName(QString::fromUtf8("kpGain_throttle"));
        kpGain_throttle->setGeometry(QRect(150, 60, 41, 23));
        kiGain_throttle = new QLineEdit(frame_2);
        kiGain_throttle->setObjectName(QString::fromUtf8("kiGain_throttle"));
        kiGain_throttle->setGeometry(QRect(150, 90, 41, 23));
        kiGain = new QLineEdit(frame_2);
        kiGain->setObjectName(QString::fromUtf8("kiGain"));
        kiGain->setGeometry(QRect(70, 90, 41, 23));
        kdGain_throttle = new QLineEdit(frame_2);
        kdGain_throttle->setObjectName(QString::fromUtf8("kdGain_throttle"));
        kdGain_throttle->setGeometry(QRect(150, 120, 41, 23));
        kdGain = new QLineEdit(frame_2);
        kdGain->setObjectName(QString::fromUtf8("kdGain"));
        kdGain->setGeometry(QRect(70, 120, 41, 23));
        label_24 = new QLabel(frame_2);
        label_24->setObjectName(QString::fromUtf8("label_24"));
        label_24->setGeometry(QRect(30, 170, 41, 20));
        QFont font2;
        font2.setPointSize(8);
        label_24->setFont(font2);
        label_25 = new QLabel(frame_2);
        label_25->setObjectName(QString::fromUtf8("label_25"));
        label_25->setGeometry(QRect(70, 170, 41, 20));
        label_25->setFont(font2);
        label_26 = new QLabel(frame_2);
        label_26->setObjectName(QString::fromUtf8("label_26"));
        label_26->setGeometry(QRect(130, 170, 51, 20));
        label_26->setFont(font2);
        label_27 = new QLabel(frame_2);
        label_27->setObjectName(QString::fromUtf8("label_27"));
        label_27->setGeometry(QRect(180, 170, 41, 20));
        label_27->setFont(font2);
        label_28 = new QLabel(frame_2);
        label_28->setObjectName(QString::fromUtf8("label_28"));
        label_28->setGeometry(QRect(50, 150, 121, 20));
        label_28->setFont(font1);
        frame_3 = new QFrame(automode);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        frame_3->setGeometry(QRect(10, 10, 391, 191));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        label_15 = new QLabel(frame_3);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(100, 10, 161, 20));
        label_15->setFont(font);
        label_17 = new QLabel(frame_3);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(10, 70, 71, 16));
        label_18 = new QLabel(frame_3);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(10, 40, 71, 16));
        targetLatitude = new QLineEdit(frame_3);
        targetLatitude->setObjectName(QString::fromUtf8("targetLatitude"));
        targetLatitude->setGeometry(QRect(80, 40, 113, 23));
        targetLongitude = new QLineEdit(frame_3);
        targetLongitude->setObjectName(QString::fromUtf8("targetLongitude"));
        targetLongitude->setGeometry(QRect(80, 70, 113, 23));
        addTargetPos = new QPushButton(frame_3);
        addTargetPos->setObjectName(QString::fromUtf8("addTargetPos"));
        addTargetPos->setGeometry(QRect(10, 100, 80, 23));
        addTargetPos->setFont(font2);
        clearAllTargetPos = new QPushButton(frame_3);
        clearAllTargetPos->setObjectName(QString::fromUtf8("clearAllTargetPos"));
        clearAllTargetPos->setGeometry(QRect(100, 100, 80, 23));
        clearAllTargetPos->setFont(font2);
        loadSavedPos = new QPushButton(frame_3);
        loadSavedPos->setObjectName(QString::fromUtf8("loadSavedPos"));
        loadSavedPos->setGeometry(QRect(20, 130, 141, 23));
        loadSavedPos->setFont(font2);
        Start = new QPushButton(frame_3);
        Start->setObjectName(QString::fromUtf8("Start"));
        Start->setGeometry(QRect(10, 160, 80, 23));
        Start->setFont(font1);
        Stop = new QPushButton(frame_3);
        Stop->setObjectName(QString::fromUtf8("Stop"));
        Stop->setGeometry(QRect(110, 160, 80, 23));
        Stop->setFont(font1);
        targetStatusText = new QPlainTextEdit(frame_3);
        targetStatusText->setObjectName(QString::fromUtf8("targetStatusText"));
        targetStatusText->setGeometry(QRect(200, 40, 171, 141));

        retranslateUi(automode);

        QMetaObject::connectSlotsByName(automode);
    } // setupUi

    void retranslateUi(QDialog *automode)
    {
        automode->setWindowTitle(QApplication::translate("automode", "Dialog", nullptr));
        label->setText(QApplication::translate("automode", "Boat Metrics", nullptr));
        label_3->setText(QApplication::translate("automode", "Speed & Navigation:", nullptr));
        label_4->setText(QApplication::translate("automode", "Orientation:", nullptr));
        label_5->setText(QApplication::translate("automode", "Position:", nullptr));
        label_2->setText(QApplication::translate("automode", "Latitude:", nullptr));
        label_6->setText(QApplication::translate("automode", "Longitude:", nullptr));
        label_7->setText(QApplication::translate("automode", "Altitude:", nullptr));
        label_8->setText(QApplication::translate("automode", "GpsFix:", nullptr));
        label_9->setText(QApplication::translate("automode", "Roll:", nullptr));
        label_10->setText(QApplication::translate("automode", "Pitch:", nullptr));
        label_11->setText(QApplication::translate("automode", "Heading:", nullptr));
        label_12->setText(QApplication::translate("automode", "Angular velocity:", nullptr));
        label_13->setText(QApplication::translate("automode", "Distance to WayPoint:", nullptr));
        label_14->setText(QApplication::translate("automode", "Speed:", nullptr));
        latitude->setText(QApplication::translate("automode", "0", nullptr));
        longitude->setText(QApplication::translate("automode", "0", nullptr));
        altitude->setText(QApplication::translate("automode", "0", nullptr));
        gpsFixStatus->setText(QApplication::translate("automode", "0", nullptr));
        heading->setText(QApplication::translate("automode", "0", nullptr));
        roll->setText(QApplication::translate("automode", "0", nullptr));
        pitch->setText(QApplication::translate("automode", "0", nullptr));
        speed->setText(QApplication::translate("automode", "0", nullptr));
        angularvelocity->setText(QApplication::translate("automode", "0", nullptr));
        distToWaypoint->setText(QApplication::translate("automode", "0", nullptr));
        label_16->setText(QApplication::translate("automode", "PID Tuning", nullptr));
        label_19->setText(QApplication::translate("automode", "P-Gain:", nullptr));
        label_20->setText(QApplication::translate("automode", "I-Gain:", nullptr));
        label_21->setText(QApplication::translate("automode", "D-Gain:", nullptr));
        label_22->setText(QApplication::translate("automode", "For Steer:", nullptr));
        label_23->setText(QApplication::translate("automode", "For Throttle:", nullptr));
        label_24->setText(QApplication::translate("automode", "Steer:", nullptr));
        label_25->setText(QApplication::translate("automode", "0", nullptr));
        label_26->setText(QApplication::translate("automode", "Throttle:", nullptr));
        label_27->setText(QApplication::translate("automode", "0", nullptr));
        label_28->setText(QApplication::translate("automode", "PWM sent to Boat", nullptr));
        label_15->setText(QApplication::translate("automode", "Set Target Positions", nullptr));
        label_17->setText(QApplication::translate("automode", "Longitude:", nullptr));
        label_18->setText(QApplication::translate("automode", "Latitude:", nullptr));
        addTargetPos->setText(QApplication::translate("automode", "Add Position", nullptr));
        clearAllTargetPos->setText(QApplication::translate("automode", "Clear All", nullptr));
        loadSavedPos->setText(QApplication::translate("automode", "Load Saved Positions", nullptr));
        Start->setText(QApplication::translate("automode", "START", nullptr));
        Stop->setText(QApplication::translate("automode", "STOP", nullptr));
    } // retranslateUi

};

namespace Ui {
    class automode: public Ui_automode {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_AUTOMODE_H
