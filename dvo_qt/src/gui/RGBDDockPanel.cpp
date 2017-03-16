#include "RGBDDockPanel.h"
#include "ui_RGBDDockPanel.h"

#include <glog/logging.h>

RGBDDockPanel::RGBDDockPanel (QWidget* parent)
: QDockWidget (parent), ui (new Ui::RGBDDockPanel)
{
    ui->setupUi (this);
}

RGBDDockPanel::~RGBDDockPanel ()
{
    delete ui;
}

void RGBDDockPanel::on_startButton_clicked()
{
    emit start_signal();
}
