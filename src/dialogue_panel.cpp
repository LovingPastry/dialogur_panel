#include <QVBoxLayout>
#include <QTimer>
#include "dialogue_panel.h"
#include <pluginlib/class_list_macros.h>

namespace dialogue_panel
{

DialoguePanel::DialoguePanel(QWidget* parent)
  : rviz::Panel(parent)
{

  // 创建主窗口实例
  main_window_ = new MainWindow();
  
  // 创建布局并添加主窗口
  layout_ = new QVBoxLayout;
  layout_->setContentsMargins(0,0,0,0);  // 设置边距为0
  layout_->addWidget(main_window_);
  
  // 设置面板的布局
  setLayout(layout_);
  QTimer::singleShot(1000, this, [this]() {
      main_window_->InitASRWorker();  // 原初始化逻辑
  });
}

DialoguePanel::~DialoguePanel()
{
  if(main_window_)
  {
    delete main_window_;
    main_window_ = nullptr;
  }
}

void DialoguePanel::resizeEvent(QResizeEvent* event)
{
  // 确保主窗口随面板大小调整
  if(main_window_)
  {
    main_window_->resize(width(), height());
  }
  rviz::Panel::resizeEvent(event);
}

} // namespace dialogue_panel

PLUGINLIB_EXPORT_CLASS(dialogue_panel::DialoguePanel, rviz::Panel)