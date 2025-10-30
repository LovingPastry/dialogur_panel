#ifndef DIALOGUE_PANEL_H
#define DIALOGUE_PANEL_H

#include <QWidget>
#include <QVBoxLayout>
#include <ros/ros.h>
#include <rviz/panel.h>
#include "mainwindow.h"

namespace dialogue_panel
{

class DialoguePanel: public rviz::Panel
{
  Q_OBJECT
public:
  DialoguePanel(QWidget* parent = 0);
  virtual ~DialoguePanel();

protected:
  // QWidget interface
  virtual void resizeEvent(QResizeEvent* event);

private:
  MainWindow* main_window_;
  QVBoxLayout* layout_;
};

} // namespace dialogue_panel

#endif // DIALOGUE_PANEL_H