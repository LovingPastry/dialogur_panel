/*-------------------------------------------------
#
# Project created by QtCreator
# Author: 沙振宇
# CreateTime: 2018-07-23
# UpdateTime: 2019-12-27
# Info: Qt5气泡式聊天框——QListWidget+QPainter实现
# Url:https://shazhenyu.blog.csdn.net/article/details/81505832
# Github:https://github.com/ShaShiDiZhuanLan/Demo_MessageChat_Qt
#
#-------------------------------------------------*/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "funASR.h"
#include "KokoroTTS.h"
#include <QMainWindow>
#include <QListWidgetItem>
#include <QPushButton>
#include <QAudioRecorder>
#include <QDir>
#include <QUrl>
#include "qnchatmessage.h"
#include <QThread>
#include <QCoreApplication> 
#include <QMutex>         
#include <QMutexLocker>  
#include <QDebug>  // 如果使用qDebug
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/service_client.h>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void InitASRWorker();
    void InitTTSWorker();
    void bot_msg_Callback(const std_msgs::String::ConstPtr& msg);
    void dealMessage(QNChatMessage *messageW, QListWidgetItem *item, QString text, QString time, QNChatMessage::User_Type type);
    void dealMessageTime(QString curMsgTime);
protected:
    virtual void resizeEvent(QResizeEvent* event) override;
    // void resizeEvent(QResizeEvent *event);`
private slots:

    void on_pushButton_clicked();
    void on_recordButton_pressed(); 
    void on_recordButton_released(); 
private:
    QMutex m_initMutex;
    Ui::MainWindow *ui;
    QPushButton* m_recordBtn;
    QAudioRecorder* m_audioRecorder;
    QThread asrThread;
    AsrWorker *asrWorker;
    ros::Subscriber m_botMsgSub;
    ros::Publisher m_userMsgPub;
    ros::ServiceClient tts_client;
};

#endif // MAINWINDOW_H
