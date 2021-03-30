#include <QApplication>
#include <QHBoxLayout>
#include <QQuickView>
#include <QtQuick>
#include <QString>
#include <QColor>
#include <QTimer>

#include <map>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "myviz.h"

struct gauge{
    QQuickView *view;
    QWidget *container;
    QObject *obj;
};

// create gauge map to store a global reference to all our gauges
std::map<std::string, gauge*> gauges;
// typedef for commodity
typedef std::pair<std::string, gauge*> MyPair;

void ros_spin_func() {
    ROS_INFO("starting spinner");
    ros::spin();
}

void servo_current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["servo_current"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
}

void computer_current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["computer_current"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
}

void current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["current"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
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
    QWidget window;
    window.setStyleSheet("background:rgb(49, 54, 59);");

    // create layout objects
    QHBoxLayout hlayout;
    QGridLayout left_grid;

    // create gauges
    gauges.insert(MyPair(((std::string)"computer_current"), new gauge()));
    gauges.insert(MyPair(((std::string)"servo_current"), new gauge()));
    gauges.insert(MyPair(((std::string)"current"), new gauge()));
    gauges.insert(MyPair(((std::string)"voltage"), new gauge()));

    // - iterate on the gauge map to initialize all our gauges
    std::map<std::string, gauge*>::iterator it;
    for(auto& [key, val] : gauges){
        std::cout << key << std::endl;
        val->view = new QQuickView();
        val->container = QWidget::createWindowContainer(val->view, &window);
        val->view->setSource(QUrl::fromLocalFile("src/knob.qml"));
        val->view->setResizeMode(QQuickView::SizeRootObjectToView);
        val->view->setColor(QColor(49, 54, 59));

        val->obj = val->view->rootObject();

        val->container->setMinimumSize(200, 200);
        val->container->setMaximumSize(200, 200);
        val->container->setFocusPolicy(Qt::TabFocus);
    }

    // create ros subscribers
    ros::Subscriber sub1 = n.subscribe("/servo_current", 10, servo_current_callback);
    ros::Subscriber sub2 = n.subscribe("/computer_current", 10, computer_current_callback);
    ros::Subscriber sub3 = n.subscribe("/current", 10, current_callback);
    ros::Subscriber sub4 = n.subscribe("/voltage", 10, voltage_callback);

    // librviz instance
    MyViz myviz;

    // start ros polling in another thread
    std::thread ros_spinner(ros_spin_func);

    // set layout
    left_grid.addWidget(gauges["computer_current"]->container, 0, 0);
    left_grid.addWidget(gauges["servo_current"]->container, 1, 0);
    left_grid.addWidget(gauges["current"]->container, 0, 1);
    left_grid.addWidget(gauges["voltage"]->container, 1, 1);

    hlayout.addLayout(&left_grid);
    hlayout.addWidget(&myviz);

    window.setLayout(&hlayout);
    window.show();

    // execute app
    int app_return = app.exec();

    ROS_INFO("window closed. exiting...");

    // kill ros thread
    ros_spinner.detach();
    ros::shutdown();

    return app_return;
}
