#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
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
        val->obj->setProperty("fontSize", 24);
        val->obj->setProperty("titleFontSize", 18);

        val->container->setMinimumSize(350, 350);
        val->container->setMaximumSize(350, 350);
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

    // create joystick instance
    QQuickView jsl;
    QWidget* jcontainerl = QWidget::createWindowContainer(&jsl, &window);
    jsl.setSource(QUrl::fromLocalFile("src/joystick.qml"));
    jsl.resize(QSize(600, 600));
    // jsl.setResizeMode(QQuickView::SizeRootObjectToView);
    jsl.setColor(QColor(49, 54, 59));
    QObject* jobjl = jsl.rootObject();
    jcontainerl->setMinimumSize(600, 600);
    jcontainerl->setMaximumSize(600, 600);
    jcontainerl->setFocusPolicy(Qt::TabFocus);

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
    QSpacerItem sp1(50, 20);
    QSpacerItem sp2(50, 20);
    QSpacerItem sp3(50, 20);
    QSpacerItem sp4(50, 20);
    left_grid.addWidget(gauges["computer_current"]->container, 0, 0);
    left_grid.addItem(&sp1, 0, 1);
    left_grid.addWidget(gauges["servo_current"]->container, 2, 0);
    left_grid.addItem(&sp2, 1, 0);
    left_grid.addWidget(gauges["current"]->container, 0, 2);
    left_grid.addItem(&sp3, 1, 2);
    left_grid.addWidget(gauges["voltage"]->container, 2, 2);
    left_grid.addItem(&sp4, 2, 1);

    lvlayout.addLayout(&left_grid);
    lvlayout.addWidget(jcontainerl, Qt::AlignHCenter);

    hlayout.addLayout(&lvlayout);
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
