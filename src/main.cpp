#include <QApplication>
#include <QHBoxLayout>
#include <QQuickView>
#include <QtQuick>
#include <QString>
#include <QColor>
#include <QTimer>

#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "myviz.h"

QObject *servo_current_object;

void ros_spin_func() {
    ROS_INFO("starting spinner");
    ros::spin();
}

void servo_current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(servo_current_object, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
    // ROS_INFO("computer_current: [%f]", float_msg->data);
}

int main(int argc, char **argv){
    if(!ros::isInitialized()){
        ros::init(argc, argv, "myviz");
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/computer_current", 10, servo_current_callback);

    QApplication app(argc, argv);

    QWidget *window = new QWidget();
    window->setStyleSheet("background:rgb(49, 54, 59);");

    QHBoxLayout *hlayout = new QHBoxLayout();

    QQuickView *servo_current = new QQuickView();
    
    QWidget *container = QWidget::createWindowContainer(servo_current, window);

    servo_current->setSource(QUrl::fromLocalFile("src/knob.qml"));
    servo_current->setResizeMode(QQuickView::SizeRootObjectToView);
    servo_current->setColor(QColor(49, 54, 59));

    servo_current_object = servo_current->rootObject();

    container->setMinimumSize(300, 300);
    container->setMaximumSize(300, 300);
    container->setFocusPolicy(Qt::TabFocus);
    container->setAttribute(Qt::WA_TranslucentBackground, true);

    MyViz* myviz = new MyViz();

    std::thread ros_spinner(ros_spin_func);

    hlayout->addWidget(container);
    hlayout->addWidget(myviz);

    window->setLayout(hlayout);
    window->show();

    int app_return = app.exec();

    ROS_INFO("window closed. exiting...");

    ros_spinner.detach();
    ros::shutdown();

    return app_return;
}
