// QT
#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QQuickView>
#include <QtQuick>
#include <QString>
#include <QColor>
#include <QTimer>
#include <QWidget>
#include <QtCharts/QtCharts>
#include <QVector>

// std
#include <map>
#include <thread>
#include <deque>
#include <csignal>
#include <mutex>
#include <time.h>

//ros
#include <ros/ros.h>
#include <std_msgs/Float32.h>

//linux
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#define CHART_POINTS 50

// joysticks
int USB;
int vrx1 = 0;
int vry1 = 0;
int vrx2 = 0;
int vry2 = 0;
std::mutex smtx;

// CHARTS
QChart *chart;
QLineSeries* series;
QChartView *chartView;
QVector<QPointF> current_queue;
std::mutex mtx;
unsigned long prev_chart_ms;

// GAUGES
struct gauge{
    QQuickView *view;
    QWidget *container;
    QObject *obj;
};

// create gauge map to store a global reference to all our gauges
std::map<std::string, gauge*> gauges;
// typedef for commodity
typedef std::pair<std::string, gauge*> MyPair;

// ROS and Signal handlers
void ros_spin_func() {
    ROS_INFO("starting spinner");
    ros::spin();
}

void sigHandler(int s){
    std::signal(s, SIG_DFL);
    qApp->quit();
}

void servo_current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["servo_current"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
}

void computer_current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["computer_current"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
}

void current_callback(const std_msgs::Float32::ConstPtr& float_msg){
    mtx.lock();
    QMetaObject::invokeMethod(gauges["current"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    if((spec.tv_nsec / 1000000) - prev_chart_ms > 20){  // execute every 20ms
        QPointF p;
        p.setY(float_msg->data);
        p.setX(CHART_POINTS);
        current_queue.push_back(p);
        if(current_queue.size() > CHART_POINTS){
            current_queue.pop_front();
        }
        for(auto& elem : current_queue){
            elem.setX(elem.x() - 1);
        }
        series->replace(current_queue);
        prev_chart_ms = (spec.tv_nsec / 1000000);
    }
    mtx.unlock();
}

void voltage_callback(const std_msgs::Float32::ConstPtr& float_msg){
    QMetaObject::invokeMethod(gauges["voltage"]->obj, "update", QGenericReturnArgument(), Q_ARG(QVariant, float_msg->data));
}

uint8_t sread(int *ser){
    uint8_t buf = 0;
    read(USB, &buf, 1);
    return buf;
}

void serial_read_thread(){
    while(1){
        if(sread(&USB) != 0x55){
            continue;
        }
        if(sread(&USB) != 0x55){
            continue;
        }
        smtx.lock();
        vrx1 = sread(&USB);
        vrx2 = sread(&USB);
        vry1 = sread(&USB);
        vry2 = sread(&USB);
        smtx.unlock();
        printf("%d %d %d %d\n", vrx1, vry1, vrx2, vry2);
    }
    
}

int main(int argc, char **argv){
    // initialize ros
    ros::init(argc, argv, "nightmare_control");
    ros::NodeHandle n;

    // initialize serial port
    USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
    struct termios tty;
    struct termios tty_old;
    memset(&tty, 0, sizeof tty);
    // check for errors
    if ( tcgetattr ( USB, &tty ) != 0 ) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }
    tty_old = tty;
    // set speed
    cfsetospeed (&tty, (speed_t)B115200);
    cfsetispeed (&tty, (speed_t)B115200);

    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    // flush and apply settings
    cfmakeraw(&tty);
    tcflush( USB, TCIFLUSH );
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }

    // create app and window
    QApplication app(argc, argv);

    // signal handlers
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);

    QWidget window;
    window.setMinimumSize(250, 600);
    window.setStyleSheet("background:rgb(49, 54, 59);");

    // create layout objects
    QHBoxLayout hlayout;
    hlayout.setSpacing(0);
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
        val->obj->setProperty("fontSize", 14);
        val->obj->setProperty("titleFontSize", 12);

        val->container->setMinimumSize(130, 130);
        val->container->setMaximumSize(130, 130);
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

    // edit chart
    chart = new QChart();
    series = new QLineSeries();
    chartView  = new QChartView(chart);
    // series->setUseOpenGL(true);
    QPen pen;
    QFont font;
    pen.setColor(QColor(106, 255, 206));
    pen.setWidth(2);
    font.setPointSize(12);
    series->setPen(pen);
    chart->setTitle("Current");
    chart->setTitleBrush(QBrush(QColor(106, 255, 206)));
    chart->setTitleFont(font);
    chart->legend()->hide();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->axes(Qt::Horizontal).first()->setRange(0, CHART_POINTS);
    chart->axes(Qt::Horizontal).first()->setLabelsVisible(false);
    chart->axes(Qt::Horizontal).first()->setGridLineVisible(false);
    chart->axes(Qt::Vertical).first()->setRange(0, 15);
    chart->axes(Qt::Vertical).first()->setLabelsBrush(QBrush(QColor(106, 255, 206)));
    chart->setBackgroundBrush(QBrush(QColor("transparent")));

    // create ros subscribers
    ros::Subscriber sub1 = n.subscribe("/servo_current", 10, servo_current_callback);
    ros::Subscriber sub2 = n.subscribe("/computer_current", 10, computer_current_callback);
    ros::Subscriber sub3 = n.subscribe("/current", 10, current_callback);
    ros::Subscriber sub4 = n.subscribe("/voltage", 10, voltage_callback);

    // start ros polling in another thread
    std::thread ros_spinner(ros_spin_func);
    std::thread serial_thread(serial_read_thread);

    // set layout
    left_grid.addWidget(gauges["computer_current"]->container, 0, 0);
    left_grid.addWidget(gauges["servo_current"]->container, 0, 1);
    left_grid.addWidget(gauges["current"]->container, 1, 0);
    left_grid.addWidget(gauges["voltage"]->container, 1, 1);

    lvlayout.addLayout(&left_grid);
    lvlayout.addWidget(chartView);

    hlayout.addLayout(&lvlayout);

    window.setLayout(&hlayout);
    window.show();

    // execute app
    int app_return = app.exec();

    ROS_INFO("window closed. exiting...");

    // kill ros thread
    ros::shutdown();
    ros_spinner.detach();

    return app_return;
}
