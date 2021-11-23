#include "./plot/include/plot.h"
#include "./plot/include/imu_plot.h"
#include "ui_plot.h"
#include "./control/include/data.h"


std::atomic<double> joint_pos[12] = {0,0,0};
extern std::atomic<double> gamepad[2];
extern double input_angle[12];      //任意时刻关节角度
PlotWindow::PlotWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PlotWindow),

    jPlot(nullptr),
    jTag1(nullptr),
    jTag2(nullptr),
    jTag3(nullptr)
{
    ui->setupUi(this);


    //setCentralWidget(mPlot);

    mDataTimer.start(40);
    connect(ui->act_imu,SIGNAL(triggered()),this,SLOT(setPlotImu()));
    connect(ui->act_position,SIGNAL(triggered()),this,SLOT(setPlotJoint()));

}

void PlotWindow::setPlotImu()
{
    ImuPlotWindow* window = new ImuPlotWindow;
    window->show();
    window->setAttribute(Qt::WA_DeleteOnClose);
    window->resize(600,400);

}


void PlotWindow::setPlotJoint()
{
    qDebug() << "joint";

  // plot joint pos //
    jPlot = new QCustomPlot;
    setCentralWidget(jPlot);



    // configure plot to have two right axes:
    jPlot->yAxis->setTickLabels(false);
    connect(jPlot->yAxis2, SIGNAL(rangeChanged(QCPRange)), jPlot->yAxis, SLOT(setRange(QCPRange))); // left axis only mirrors inner right axis
    jPlot->yAxis2->setVisible(true);
    jPlot->axisRect()->addAxis(QCPAxis::atRight);
    jPlot->axisRect()->addAxis(QCPAxis::atRight);
    jPlot->axisRect()->axis(QCPAxis::atRight, 0)->setPadding(30); // add some padding to have space for tags
    jPlot->axisRect()->axis(QCPAxis::atRight, 1)->setPadding(30); // add some padding to have space for tags
    jPlot->axisRect()->axis(QCPAxis::atRight, 2)->setPadding(30); // add some padding to have space for tags

    // create graphs:
    jGraph1 = jPlot->addGraph(jPlot->xAxis, jPlot->axisRect()->axis(QCPAxis::atRight, 0));
    jGraph2 = jPlot->addGraph(jPlot->xAxis, jPlot->axisRect()->axis(QCPAxis::atRight, 1));
    jGraph3 = jPlot->addGraph(jPlot->xAxis, jPlot->axisRect()->axis(QCPAxis::atRight, 2));
    jGraph1->setPen(QPen(QColor(Qt::red)));
    jGraph2->setPen(QPen(QColor(Qt::blue)));
    jGraph3->setPen(QPen(QColor(Qt::green)));
    // create tags with newly introduced AxisTag class (see axistag.h/.cpp):
    jTag1 = new AxisTag(jGraph1->valueAxis());
    jTag1->setPen(jGraph1->pen());
    jTag2 = new AxisTag(jGraph2->valueAxis());
    jTag2->setPen(jGraph2->pen());
    jTag3 = new AxisTag(jGraph3->valueAxis());
    jTag3->setPen(jGraph3->pen());



    connect(&mDataTimer, SIGNAL(timeout()), this, SLOT(plotJoint()));



}
void PlotWindow::plotJoint()
{

    jPlot->resize(this->size().width(),this->size().height());
    // calculate and add a new data point to each graph:
    jGraph1->addData(jGraph1->dataCount(), input_angle[0]);
    jGraph2->addData(jGraph2->dataCount(), input_angle[1]);
    jGraph3->addData(jGraph3->dataCount(), input_angle[2]);
    // make key axis range scroll with the data:
    jPlot->xAxis->rescale();
    jGraph1->rescaleValueAxis(false, true);
    jGraph2->rescaleValueAxis(false, true);
    jGraph3->rescaleValueAxis(false, true);
    jPlot->xAxis->setRange(jPlot->xAxis->range().upper, 100, Qt::AlignRight);

    // update the vertical axis tag positions and texts to match the rightmost data point of the graphs:
    double graph1Value = jGraph1->dataMainValue(jGraph1->dataCount()-1);
    double graph2Value = jGraph2->dataMainValue(jGraph2->dataCount()-1);
    double graph3Value = jGraph3->dataMainValue(jGraph3->dataCount()-1);
    jTag1->updatePosition(graph1Value);
    jTag2->updatePosition(graph2Value);
    jTag3->updatePosition(graph3Value);
    jTag1->setText(QString::number(graph1Value, 'f', 2));
    jTag2->setText(QString::number(graph2Value, 'f', 2));
    jTag3->setText(QString::number(graph3Value, 'f', 2));

    jPlot->replot();
}

PlotWindow::~PlotWindow()
{
    delete ui;
}
