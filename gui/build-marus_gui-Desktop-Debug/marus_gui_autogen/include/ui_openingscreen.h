/********************************************************************************
** Form generated from reading UI file 'openingscreen.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OPENINGSCREEN_H
#define UI_OPENINGSCREEN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_openingscreen
{
public:
    QPushButton *manual;
    QPushButton *automode;
    QLabel *label;

    void setupUi(QDialog *openingscreen)
    {
        if (openingscreen->objectName().isEmpty())
            openingscreen->setObjectName(QString::fromUtf8("openingscreen"));
        openingscreen->resize(320, 240);
        manual = new QPushButton(openingscreen);
        manual->setObjectName(QString::fromUtf8("manual"));
        manual->setGeometry(QRect(30, 110, 101, 61));
        automode = new QPushButton(openingscreen);
        automode->setObjectName(QString::fromUtf8("automode"));
        automode->setGeometry(QRect(179, 110, 101, 61));
        label = new QLabel(openingscreen);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(60, 40, 231, 41));
        QFont font;
        font.setPointSize(11);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);

        retranslateUi(openingscreen);

        QMetaObject::connectSlotsByName(openingscreen);
    } // setupUi

    void retranslateUi(QDialog *openingscreen)
    {
        openingscreen->setWindowTitle(QApplication::translate("openingscreen", "Dialog", nullptr));
        manual->setText(QApplication::translate("openingscreen", "Manual mode", nullptr));
        automode->setText(QApplication::translate("openingscreen", "Auto mode", nullptr));
        label->setText(QApplication::translate("openingscreen", "MARUS BOAT SIMULATOR", nullptr));
    } // retranslateUi

};

namespace Ui {
    class openingscreen: public Ui_openingscreen {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OPENINGSCREEN_H
