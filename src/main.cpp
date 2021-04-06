#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QQuickView>
#include <QtQuick>
#include <QString>
#include <QColor>
#include <QTimer>
#include <QWidget>
#include <QtCharts/QtCharts>
#include <QVector>

#include <map>
#include <thread>
#include <deque>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <csignal>

#define CHART_POINTS 30

// CHARTS
QChart *chart;
QLineSeries* series;
QChartView *chartView;
QVector<QPointF> current_queue;

// GAUGES
struct gauge{
    QQuickView *view;
    QWidget *container;
    QObject *obj;
};

// create gauge map to store a global reference to all our gauges
std::map<std::string, gauge*> gauges;
// typedef for commodity
typedef std::pair<std::string, gauge*> MyPair;

// ROS and Signal handlers
void ros_spin_func() {
    ROS_INFO("starting spinner");
    ros::spin();
}

void sigHandler(int s){
    std::signal(s, SIG_DFL);
    qApp->quit();
}

void servo_current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["servo_current"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
}

void computer_current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["computer_current"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
}

void current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["current"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
    QPointF p;
    p.setY(float_msg->data);
    p.setX(CHART_POINTS);
    current_queue.push_back(p);
    if(current_queue.size() > CHART_POINTS){
        current_queue.pop_front();
    }
    for(auto& elem : current_queue){
        elem.setX(elem.x() - 1);
    }
    series->replace(current_queue);
}

void voltage_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["voltage"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
}

int main(int argc, char **argv){
    // initialize ros
    ros::init(argc, argv, "myviz");
    ros::NodeHandle n;

    // create app and window
    QApplication app(argc, argv);

    // signal handlers
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);

    QWidget window;
    window.setMinimumSize(250, 600);
    window.setStyleSheet("background:rgb(49, 54, 59);");

    // create layout objects
    QHBoxLayout hlayout;
    QVBoxLayout lvlayout;
    QGridLayout left_grid;

    // create gauges
    gauges.insert(MyPair(((std::string)"computer_current"), new gauge()));
    gauges.insert(MyPair(((std::string)"servo_current"), new gauge()));
    gauges.insert(MyPair(((std::string)"current"), new gauge()));
    gauges.insert(MyPair(((std::string)"voltage"), new gauge()));

    // - iterate on the gauge map to initialize all our gauges
    std::map<std::string, gauge*>::iterator it;
    for(auto& [key, val] : gauges){
        val->view = new QQuickView();
        val->container = QWidget::createWindowContainer(val->view, &window);
        val->view->setSource(QUrl::fromLocalFile("src/knob.qml"));
        val->view->setResizeMode(QQuickView::SizeRootObjectToView);
        val->view->setColor(QColor(49, 54, 59));

        val->obj = val->view->rootObject();

        // set gauge options to be applied to all gauges
        val->obj->setProperty("knobColor", QColor(106, 255, 206));
        val->obj->setProperty("fontSize", 14);
        val->obj->setProperty("titleFontSize", 12);

        val->container->setMinimumSize(130, 130);
        val->container->setMaximumSize(130, 130);
        val->container->setFocusPolicy(Qt::TabFocus);
    }

    // set gauges styles and properties
    gauges["servo_current"]->obj->setProperty("to", 15.0);
    gauges["servo_current"]->obj->setProperty("title", QString("Servo Current"));
    gauges["computer_current"]->obj->setProperty("to", 4.0);
    gauges["computer_current"]->obj->setProperty("title", QString("CPU Current"));
    gauges["current"]->obj->setProperty("to", 15.0);
    gauges["current"]->obj->setProperty("title", QString("Total Current"));
    gauges["voltage"]->obj->setProperty("suffix", QString("V"));
    gauges["voltage"]->obj->setProperty("to", 10.0);
    gauges["voltage"]->obj->setProperty("title", QString("Voltage"));

    // edit chart
    chart = new QChart();
    series = new QLineSeries();
    chartView  = new QChartView(chart);
    // series->setUseOpenGL(true);
    QPen pen;
    QFont font;
    pen.setColor(QColor(106, 255, 206));
    pen.setWidth(2);
    font.setPointSize(12);
    series->setPen(pen);
    chart->setTitle("Current");
    chart->setTitleBrush(QBrush(QColor(106, 255, 206)));
    chart->setTitleFont(font);
    chart->legend()->hide();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->axes(Qt::Horizontal).first()->setRange(0, CHART_POINTS);
    chart->axes(Qt::Horizontal).first()->setLabelsVisible(false);
    chart->axes(Qt::Horizontal).first()->setGridLineVisible(false);
    chart->axes(Qt::Vertical).first()->setRange(0, 15);
    chart->axes(Qt::Vertical).first()->setLabelsBrush(QBrush(QColor(106, 255, 206)));
    chart->setBackgroundBrush(QBrush(QColor("transparent")));

    // create ros subscribers
    ros::Subscriber sub1 = n.subscribe("/servo_current", 10, servo_current_callback);
    ros::Subscriber sub2 = n.subscribe("/computer_current", 10, computer_current_callback);
    ros::Subscriber sub3 = n.subscribe("/current", 10, current_callback);
    ros::Subscriber sub4 = n.subscribe("/voltage", 10, voltage_callback);

    // start ros polling in another thread
    std::thread ros_spinner(ros_spin_func);

    // set layout
    QSpacerItem sp1(5, 5);
    QSpacerItem sp2(5, 5);
    QSpacerItem sp3(5, 5);
    QSpacerItem sp4(5, 5);
    left_grid.addWidget(gauges["computer_current"]->container, 0, 0);
    //left_grid.addItem(&sp1, 0, 1);
    left_grid.addWidget(gauges["servo_current"]->container, 2, 0);
    //left_grid.addItem(&sp2, 1, 0);
    left_grid.addWidget(gauges["current"]->container, 0, 2);
    //left_grid.addItem(&sp3, 1, 2);
    left_grid.addWidget(gauges["voltage"]->container, 2, 2);
    //left_grid.addItem(&sp4, 2, 1);

    lvlayout.addLayout(&left_grid);
    lvlayout.addWidget(chartView);

    hlayout.addLayout(&lvlayout);

    window.setLayout(&hlayout);
    window.show();

    // execute app
    int app_return = app.exec();

    ROS_INFO("window closed. exiting...");

    // kill ros thread
    ros::shutdown();
    ros_spinner.detach();

    return app_return;
}
