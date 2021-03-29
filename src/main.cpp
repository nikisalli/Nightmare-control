#include "mainwindow.h"

#include <QApplication>
#include <QVBoxLayout>

#include <ros/ros.h>
#include "myviz.h"

int main(int argc, char **argv){
  if(!ros::isInitialized()){
    ros::init(argc, argv, "myviz", ros::init_options::AnonymousName);
  }

  QApplication app(argc, argv);
  QWidget window;

  QVBoxLayout *vlayout = new QVBoxLayout();

  MyViz* myviz = new MyViz();

  vlayout->addWidget(myviz);

  window.setLayout(vlayout);
  window.show();

  return app.exec();
}
