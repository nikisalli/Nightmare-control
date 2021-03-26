#ifndef MYVIZ_H
#define MYVIZ_H

#include <QWidget>

namespace rviz{
    class Display;
    class RenderPanel;
    class VisualizationManager;
}

class MyViz: public QWidget{
    Q_OBJECT
    public:
        MyViz( QWidget* parent = 0 );
        virtual ~MyViz();

    private:
        rviz::VisualizationManager* manager_;
        rviz::RenderPanel* render_panel_;
        rviz::Display* grid_;
        rviz::Display* robot_model_;
        rviz::Display* tf_;
};

#endif // MYVIZ_H
