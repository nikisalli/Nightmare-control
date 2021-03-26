#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "myviz.h"

MyViz::MyViz( QWidget* parent ) : QWidget( parent ){
    render_panel_ = new rviz::RenderPanel();
    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->addWidget( render_panel_ );

    setLayout( main_layout );

    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    // Create rviz components
    grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
    ROS_ASSERT( grid_ != NULL );
    robot_model_ = manager_->createDisplay( "rviz/RobotModel", "robot model", true );
    ROS_ASSERT( robot_model_ != NULL );
    tf_ = manager_->createDisplay( "rviz/TF", "tf", true );
    ROS_ASSERT( tf_ != NULL );

    // Configure rviz
    manager_->setFixedFrame( "map" );

    // Configure Grid
    grid_->subProp( "Line Style" )->setValue( "Billboards" );
    grid_->subProp( "Color" )->setValue( QColor(Qt::white) );
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue(0.01);
    grid_->subProp( "Cell Size" )->setValue(1);

    // Configure RobotModel
    robot_model_->subProp( "Robot Description" )->setValue( "robot_description" );

    // Configure TF
    tf_->subProp( "Show Names" )->setValue(false);
    tf_->subProp( "Show Axes" )->setValue(true);
    tf_->subProp( "Show Arrows" )->setValue(true);
    tf_->subProp( "Marker Scale" )->setValue(0.3);
}

// Destructor.
MyViz::~MyViz(){
  delete manager_;
}