#ifndef TUNINGPLOT1_H
#define TUNINGPLOT1_H

#include <QDialog>
#include <QVector>
#include <QPointF>
#include "qcustomplot.h"

namespace Ui {
class tuningplot1;
}

class tuningplot1 : public QDialog
{
    Q_OBJECT

public:
    explicit tuningplot1(QWidget *parent = nullptr);
    ~tuningplot1();
    void clearGraphs();
    void updatePlot(const QVector<QPointF>& headingErrors,
                    const QVector<QPointF>& distanceErrors,
                    const QVector<QPointF>& currentPositions,
                    const QVector<QPointF>& desiredPositions,
                    const QVector<QPointF>& currentAngle,
                    const QVector<QPointF>& desiredAngle);
    void appendPlotData(const QPointF& headingError,
                                     const QPointF& distanceError,
                                     const QPointF& currentPos,
                                     const QPointF& desiredPos,
                                     const QPointF& currentAng,
                                     const QPointF& desiredAng);
private:
    Ui::tuningplot1 *ui;
    QCustomPlot *headingplot;
    QCustomPlot *distanceplot;
    QCustomPlot *positionPlot;
    QCustomPlot *anglePlot;
};

#endif // TUNINGPLOT1_H
