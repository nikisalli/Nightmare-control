#include "mainwindow.h"

#include <QApplication>

#include <QApplication>
#include <ros/ros.h>
#include "myviz.h"

int main(int argc, char **argv){
  if( !ros::isInitialized() ){
    ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  MyViz* myviz = new MyViz();
  myviz->show();

  app.exec();

  delete myviz;
}
