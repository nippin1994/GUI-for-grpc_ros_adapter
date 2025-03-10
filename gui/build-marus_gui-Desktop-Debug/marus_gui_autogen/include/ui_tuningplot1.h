/********************************************************************************
** Form generated from reading UI file 'tuningplot1.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TUNINGPLOT1_H
#define UI_TUNINGPLOT1_H

#include <QtCore/QVariant>
#include <QtQuickWidgets/QQuickWidget>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>

QT_BEGIN_NAMESPACE

class Ui_tuningplot1
{
public:
    QQuickWidget *headingPlot;
    QQuickWidget *distancePlot;
    QQuickWidget *positionPlot;
    QQuickWidget *anglePlot;

    void setupUi(QDialog *tuningplot1)
    {
        if (tuningplot1->objectName().isEmpty())
            tuningplot1->setObjectName(QString::fromUtf8("tuningplot1"));
        tuningplot1->resize(607, 438);
        headingPlot = new QQuickWidget(tuningplot1);
        headingPlot->setObjectName(QString::fromUtf8("headingPlot"));
        headingPlot->setGeometry(QRect(0, 0, 300, 200));
        headingPlot->setResizeMode(QQuickWidget::SizeRootObjectToView);
        distancePlot = new QQuickWidget(tuningplot1);
        distancePlot->setObjectName(QString::fromUtf8("distancePlot"));
        distancePlot->setGeometry(QRect(300, 0, 300, 200));
        distancePlot->setResizeMode(QQuickWidget::SizeRootObjectToView);
        positionPlot = new QQuickWidget(tuningplot1);
        positionPlot->setObjectName(QString::fromUtf8("positionPlot"));
        positionPlot->setGeometry(QRect(0, 200, 300, 200));
        positionPlot->setResizeMode(QQuickWidget::SizeRootObjectToView);
        anglePlot = new QQuickWidget(tuningplot1);
        anglePlot->setObjectName(QString::fromUtf8("anglePlot"));
        anglePlot->setGeometry(QRect(300, 200, 300, 200));
        anglePlot->setResizeMode(QQuickWidget::SizeRootObjectToView);

        retranslateUi(tuningplot1);

        QMetaObject::connectSlotsByName(tuningplot1);
    } // setupUi

    void retranslateUi(QDialog *tuningplot1)
    {
        tuningplot1->setWindowTitle(QApplication::translate("tuningplot1", "Dialog", nullptr));
    } // retranslateUi

};

namespace Ui {
    class tuningplot1: public Ui_tuningplot1 {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TUNINGPLOT1_H
