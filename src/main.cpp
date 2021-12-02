#include <ros/ros.h>
#include "muskyRviz.h"
#include <QApplication>

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "muskyRviz", ros::init_options::AnonymousName );
  }
  // TODO: Integrate QT5 Application with ROS
  QApplication app( argc, argv );

  MuskyRviz* muskyRviz = new muskyRviz();
  muskyRviz->show();

  app.exec();

  delete muskyRviz;
}