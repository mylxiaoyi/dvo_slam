#include <ros/ros.h>
#include <utils/StackTrace.h>
#include <gui/MainWindow.h>
#include <QApplication>

int main(int argc, char **argv)
{
    dbg_stacktrace_setup();

    ros::init(argc, argv, "dvo_qt");

    QApplication app(argc, argv);

    MainWindow w;
    w.show();

    return app.exec();
}
