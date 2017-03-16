#include "MainWindow.h"
#include "RGBDDockPanel.h"
#include <glog/logging.h>

MainWindow::MainWindow (QWidget* parent) : QMainWindow (parent)
{
    ui.setupUi (this);

    setupLayout();

}

MainWindow::~MainWindow ()
{
}

void MainWindow::setupLayout()
{
    pGLViewer = new GLViewer(this);
    setCentralWidget(pGLViewer);

    RGBDDockPanel *rgbdDockPanel = new RGBDDockPanel(this);
    this->addDockWidget(Qt::RightDockWidgetArea, rgbdDockPanel);
    ui.menuView->addAction(rgbdDockPanel->toggleViewAction());

    connect(rgbdDockPanel, SIGNAL(start_signal()), this, SLOT(start_slot()));
}

void MainWindow::start_slot()
{
    timerID = startTimer(33);
}

void MainWindow::timerEvent(QTimerEvent *event)
{
    pGLViewer->updateGL();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    killTimer(timerID);
}
