#ifndef DIALOG_H
#define DIALOG_H

#include <QMainWindow>
#include "qcustomplot.h"
#include "axistag.h"

namespace Ui {
class PlotWindow;
}

class PlotWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PlotWindow(QWidget *parent = nullptr);
    ~PlotWindow();

private slots:

  void setPlotImu();
  void setPlotJoint();


  void plotJoint();

private:
    Ui::PlotWindow *ui;

    QTimer mDataTimer;



    //plot joint //
    QCustomPlot *jPlot;
    QPointer<QCPGraph> jGraph1;
    QPointer<QCPGraph> jGraph2;
    QPointer<QCPGraph> jGraph3;
    AxisTag *jTag1;
    AxisTag *jTag2;
    AxisTag *jTag3;

};

#endif // DIALOG_H
