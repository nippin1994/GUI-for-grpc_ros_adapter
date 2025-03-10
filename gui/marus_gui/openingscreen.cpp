#include "openingscreen.h"
#include "ui_openingscreen.h"
#include "mainwindow.h"
#include "automode.h"
#include <QApplication>

openingscreen::openingscreen(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::openingscreen),
    w(new MainWindow(this)),
    a(new automode(this))
{
    ui->setupUi(this);

    ui->manual->setEnabled(false);  // Disable the manual button
}

openingscreen::~openingscreen()
{
    delete ui;
}

void openingscreen::on_manual_released()
{
    w->show();
}

void openingscreen::on_automode_released()
{
    a->show();
}
