#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "GLViewer.h"
#include "ui_MainWindow.h"
#include <QMainWindow>
#include <QTimerEvent>
#include <QCloseEvent>
#include <memory>

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow (QWidget* parent = 0);
    virtual ~MainWindow ();

public slots:
    void start_slot();

protected:
    void setupLayout ();
    void timerEvent(QTimerEvent *event);
    void closeEvent(QCloseEvent *event);

private:
    Ui::MainWindow ui;

    GLViewer* pGLViewer;

    int timerID;
};

#endif // MAINWINDOW_H
