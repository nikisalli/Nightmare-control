#include <QApplication>
#include <QHBoxLayout>
#include <QQuickView>
#include <QtQuick>
#include <QString>

#include <ros/ros.h>
#include "myviz.h"

int main(int argc, char **argv){
    if(!ros::isInitialized()){
      ros::init(argc, argv, "myviz", ros::init_options::AnonymousName);
    }

    QApplication app(argc, argv);
    QWidget window;

    QHBoxLayout *vlayout = new QHBoxLayout();
    QQuickView *servo_current = new QQuickView();
    QWidget *container = QWidget::createWindowContainer(servo_current);

    container->setMinimumSize(300, 300);
    container->setMaximumSize(300, 300);

    servo_current->setSource(QUrl::fromLocalFile("src/knob.qml"));
    servo_current->setProperty("height", 300);
    QMetaObject::invokeMethod(servo_current, "update");

    MyViz* myviz = new MyViz();

    vlayout->addWidget(container);
    vlayout->addWidget(myviz);

    window.setLayout(vlayout);
    window.show();

    

    return app.exec();
}
