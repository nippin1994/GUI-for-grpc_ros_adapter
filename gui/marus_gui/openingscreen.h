#ifndef OPENINGSCREEN_H
#define OPENINGSCREEN_H

#include <QDialog>
#include "mainwindow.h"
#include "automode.h"

namespace Ui {
class openingscreen;
}

class openingscreen : public QDialog
{
    Q_OBJECT

public:
    explicit openingscreen(QWidget *parent = nullptr);
    ~openingscreen();

private slots:
    void on_manual_released();

    void on_automode_released();

private:
    Ui::openingscreen *ui;
    MainWindow *w;
    automode *a;
};

#endif // OPENINGSCREEN_H
