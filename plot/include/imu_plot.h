#ifndef IMU_PLOT_H
#define IMU_PLOT_H


#include <QMainWindow>
#include "qcustomplot.h"
#include "axistag.h"

namespace Ui {
class ImuPlotWindow;
}

class ImuPlotWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ImuPlotWindow(QWidget *parent = nullptr);
    ~ImuPlotWindow();

private slots:

  void plotImu();


private:
    Ui::ImuPlotWindow *ui;

    QTimer mDataTimer;

    // plot imu //
    QCustomPlot *mPlot;
    QPointer<QCPGraph> mGraph1;
    QPointer<QCPGraph> mGraph2;
    QPointer<QCPGraph> mGraph3;
    AxisTag *mTag1;
    AxisTag *mTag2;
    AxisTag *mTag3;



};

#endif // DIALOG_H
