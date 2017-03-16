#ifndef RGBDDOCKPANEL_H
#define RGBDDOCKPANEL_H

#include <QDockWidget>

class MainWindow;

namespace Ui {
class RGBDDockPanel;
}

class RGBDDockPanel : public QDockWidget
{
    Q_OBJECT

public:
    explicit RGBDDockPanel(QWidget *parent = 0);
    ~RGBDDockPanel();

private slots:

    void on_startButton_clicked();

signals:
    void start_signal();

private:
    Ui::RGBDDockPanel *ui;
};

#endif // RGBDDOCKPANEL_H
