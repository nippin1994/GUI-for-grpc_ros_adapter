#include "tuningplot1.h"
#include "ui_tuningplot1.h"

tuningplot1::tuningplot1(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::tuningplot1)
{
    ui->setupUi(this);
    // Assign QCustomPlot to the widgets created in the UI
    headingplot = new QCustomPlot(ui->headingPlot);
    headingplot->setGeometry(ui->headingPlot->rect());  // Match the geometry of the widget
    headingplot->setParent(ui->headingPlot);  // Set as a child of the widget

    distanceplot = new QCustomPlot(ui->distancePlot);
    distanceplot->setGeometry(ui->distancePlot->rect());  // Match the geometry of the widget
    distanceplot->setParent(ui->distancePlot);  // Set as a child of the widget

    anglePlot = new QCustomPlot(ui->anglePlot);
    anglePlot->setGeometry(ui->anglePlot->rect());  // Match the geometry of the widget
    anglePlot->setParent(ui->anglePlot);  // Set as a child of the widget

    positionPlot = new QCustomPlot(ui->positionPlot);
    positionPlot->setGeometry(ui->positionPlot->rect());  // Match the geometry of the widget
    positionPlot->setParent(ui->positionPlot);  // Set as a child of the widget

    // set the window title
    setWindowTitle("PID Tuning Plot");

}

tuningplot1::~tuningplot1()
{
    delete ui;
}

// Method to clear existing graphs and replot new data
void tuningplot1::clearGraphs()
{
    // Clear all existing graphs from both plots
    headingplot->clearGraphs();
    distanceplot->clearGraphs();
    positionPlot->clearGraphs();
    anglePlot->clearGraphs();

    // Clear legends
    headingplot->legend->clearItems();
    distanceplot->legend->clearItems();
    positionPlot->legend->clearItems();
    anglePlot->legend->clearItems();

    // Heading Error
    headingplot->addGraph();
    headingplot->graph(0)->setPen(QPen(Qt::blue));
    headingplot->xAxis->setLabel("Time Step");
    headingplot->yAxis->setLabel("Heading Error");

    // Distance Error
    distanceplot->addGraph();
    distanceplot->graph(0)->setPen(QPen(Qt::red));
    distanceplot->xAxis->setLabel("Time Step");
    distanceplot->yAxis->setLabel("Distance Error");

    // Position
    positionPlot->addGraph();  // Current
    positionPlot->graph(0)->setPen(QPen(Qt::green));
    positionPlot->graph(0)->setName("Current Position");

    positionPlot->addGraph();  // Desired
    positionPlot->graph(1)->setPen(QPen(Qt::magenta));
    positionPlot->graph(1)->setName("Desired Position");

    positionPlot->legend->setVisible(true);
    positionPlot->xAxis->setLabel("Time Step");
    positionPlot->yAxis->setLabel("Position");

    // Angle
    anglePlot->addGraph();  // Current
    anglePlot->graph(0)->setPen(QPen(Qt::green));
    anglePlot->graph(0)->setName("Current Angle");

    anglePlot->addGraph();  // Desired
    anglePlot->graph(1)->setPen(QPen(Qt::magenta));
    anglePlot->graph(1)->setName("Desired Angle");

    anglePlot->legend->setVisible(true);
    anglePlot->xAxis->setLabel("Time Step");
    anglePlot->yAxis->setLabel("Heading Angle");

    // Replot the cleared plot
    headingplot->replot();
    distanceplot->replot();
    positionPlot->replot();
    anglePlot->replot();
}

void tuningplot1::updatePlot(const QVector<QPointF>& headingErrors, const QVector<QPointF>& distanceErrors,
                            const QVector<QPointF>& currentPositions, const QVector<QPointF>& desiredPositions,
                            const QVector<QPointF>& currentAngle, const QVector<QPointF>& desiredAngle)
{
    // Clear previous graphs
    headingplot->clearGraphs();
    distanceplot->clearGraphs();

    distanceplot->clearGraphs();
    distanceplot->legend->clearItems();

    positionPlot->clearGraphs();
    positionPlot->legend->clearItems();

    anglePlot->clearGraphs();
    anglePlot->legend->clearItems();

    // Add new graph for heading error
    headingplot->addGraph();
    QVector<double> headingX, headingY;
    for (const auto& point : headingErrors) {
        headingX.push_back(point.x());
        headingY.push_back(point.y());
    }
    headingplot->graph(0)->setData(headingX, headingY);
    headingplot->graph(0)->setPen(QPen(Qt::blue));  // Set the color for the heading error graph
    headingplot->xAxis->setLabel("Time Step");
    headingplot->yAxis->setLabel("Heading Error");
    headingplot->rescaleAxes();
    headingplot->replot();

    // Add new graph for distance error
    distanceplot->addGraph();
    QVector<double> distanceX, distanceY;
    for (const auto& point : distanceErrors) {
        distanceX.push_back(point.x());
        distanceY.push_back(point.y());
    }
    distanceplot->graph(0)->setData(distanceX, distanceY);
    distanceplot->graph(0)->setPen(QPen(Qt::red));  // Set the color for the distance error graph
    distanceplot->xAxis->setLabel("Time Step");
    distanceplot->yAxis->setLabel("Distance Error");
    distanceplot->rescaleAxes();
    distanceplot->replot();

    //Add new graph for position
    // Add new graphs for current and desired positions
    positionPlot->addGraph();  // Current position graph
    QVector<double> currentPosX, currentPosY;
    for (const auto& point : currentPositions) {
        currentPosX.push_back(point.x());
        currentPosY.push_back(point.y());
    }
    positionPlot->graph(0)->setData(currentPosX, currentPosY);
    positionPlot->graph(0)->setPen(QPen(Qt::green));  // Set the color for the current position graph
    // Set name
    QString currentPosName = "Current Position";
    if (!currentPosName.isEmpty()) {
        positionPlot->graph(0)->setName(currentPosName);
    } else {
        positionPlot->graph(0)->setSelectable(QCP::phNone); // Prevent it from appearing in the legend
        positionPlot->graph(0)->setVisible(false); // Or hide it
    }

    positionPlot->addGraph();  // Desired position graph
    QVector<double> desiredPosX, desiredPosY;
    for (const auto& point : desiredPositions) {
        desiredPosX.push_back(point.x());
        desiredPosY.push_back(point.y());
    }
    positionPlot->graph(1)->setData(desiredPosX, desiredPosY);
    positionPlot->graph(1)->setPen(QPen(Qt::magenta));  // Set the color for the desired position graph
    // Set name only if needed
    QString desiredPosName = "Desired Position";
    if (!desiredPosName.isEmpty()) {
        positionPlot->graph(1)->setName(desiredPosName);
    } else {
        positionPlot->graph(1)->setSelectable(QCP::phNone); // Prevent it from appearing in the legend
        positionPlot->graph(1)->setVisible(false); // Or hide it
    }

    // Set axis labels and legends for the position plot
    positionPlot->xAxis->setLabel("Time Step");
    positionPlot->yAxis->setLabel("Position");
    positionPlot->legend->setVisible(true);
    positionPlot->rescaleAxes();
    positionPlot->replot();

    //Add new graph for heading angle plots
    // Add new graphs for current and desired positions
    anglePlot->addGraph();  // Current position graph
    QVector<double> currentAngleX, currentAngleY;
    for (const auto& point : currentAngle) {
        currentAngleX.push_back(point.x());
        currentAngleY.push_back(point.y());
    }
    anglePlot->graph(0)->setData(currentAngleX, currentAngleY);
    anglePlot->graph(0)->setPen(QPen(Qt::green));  // Set the color for the current position graph
    // Set name
    QString currentAngleName = "Current Angle";
    if (!currentAngleName.isEmpty()) {
        anglePlot->graph(0)->setName(currentAngleName);
    } else {
        anglePlot->graph(0)->setSelectable(QCP::phNone); // Disable from appearing in the legend
        anglePlot->graph(0)->setVisible(false); // Or hide it
    }

    anglePlot->addGraph();  // Desired position graph
    QVector<double> desiredAngleX, desiredAngleY;
    for (const auto& point : desiredAngle) {
        desiredAngleX.push_back(point.x());
        desiredAngleY.push_back(point.y());
    }
    anglePlot->graph(1)->setData(desiredAngleX, desiredAngleY);
    anglePlot->graph(1)->setPen(QPen(Qt::magenta));  // Set the color for the desired position graph
    // Set name only if needed
    QString desiredAngleName = "Desired Angle";
    if (!desiredAngleName.isEmpty()) {
        anglePlot->graph(1)->setName(desiredAngleName);
    } else {
        anglePlot->graph(1)->setSelectable(QCP::phNone); // Disable from appearing in the legend
        anglePlot->graph(1)->setVisible(false); // Or hide it
    }

    // Set axis labels and legends for the position plot
    anglePlot->xAxis->setLabel("Time Step");
    anglePlot->yAxis->setLabel("Heading Angle");
    anglePlot->legend->setVisible(true);
    anglePlot->rescaleAxes();
    anglePlot->replot();
}

void tuningplot1::appendPlotData(const QPointF& headingError,
                                 const QPointF& distanceError,
                                 const QPointF& currentPos,
                                 const QPointF& desiredPos,
                                 const QPointF& currentAng,
                                 const QPointF& desiredAng)
{
    // Append data
    headingplot->graph(0)->addData(headingError.x(), headingError.y());
    distanceplot->graph(0)->addData(distanceError.x(), distanceError.y());

    positionPlot->graph(0)->addData(currentPos.x(), currentPos.y());
    positionPlot->graph(1)->addData(desiredPos.x(), desiredPos.y());

    anglePlot->graph(0)->addData(currentAng.x(), currentAng.y());
    anglePlot->graph(1)->addData(desiredAng.x(), desiredAng.y());

    // Optional: remove old data (e.g., keep only last 500 points)
    const double rangeSize = 500;
    double xRight = headingError.x();
    double xLeft = xRight - rangeSize;

    headingplot->graph(0)->removeDataBefore(xLeft);
    distanceplot->graph(0)->removeDataBefore(xLeft);
    positionPlot->graph(0)->removeDataBefore(xLeft);
    positionPlot->graph(1)->removeDataBefore(xLeft);
    anglePlot->graph(0)->removeDataBefore(xLeft);
    anglePlot->graph(1)->removeDataBefore(xLeft);

    // Rescale the axes after adding the data
    headingplot->rescaleAxes();
    distanceplot->rescaleAxes();
    positionPlot->rescaleAxes();
    anglePlot->rescaleAxes();

    // Keep the x-range sliding
    headingplot->xAxis->setRange(xLeft, xRight);
    distanceplot->xAxis->setRange(xLeft, xRight);
    positionPlot->xAxis->setRange(xLeft, xRight);
    anglePlot->xAxis->setRange(xLeft, xRight);

    // Defer repaints
    headingplot->replot();
    distanceplot->replot();
    positionPlot->replot();
    anglePlot->replot();
}

