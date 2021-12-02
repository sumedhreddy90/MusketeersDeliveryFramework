#ifndef MUSKYRVIZ_H
#define MUSKYRVIZ_H

#include <QWidget>

namespace rviz
{
class VisualizationManager;
class VisualizationFrame;
class DisplayGroup;
}

class MuskyRviz: public QWidget
{
Q_OBJECT
public:
  MuskyRviz( QWidget* parent = 0 );
  virtual ~MuskyRviz();

private:
  rviz::VisualizationFrame* vizFrame_;
  rviz::VisualizationManager* manager_;
  rviz::DisplayGroup* rootDisplayGroup_;
};

#endif // MUSKYRVIZ_H