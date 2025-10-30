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
#include "funASR.h"
#include "KokoroTTS.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDateTime>
#include <QDebug>
#include <QCoreApplication>
#include <QMutex>         
#include <QMutexLocker>   
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dialogue_panel/TTS.h>

MainWindow::MainWindow(QWidget *parent) : 
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    resize(600, 800);

    qDebug() << "主线程ID:" << QThread::currentThreadId();

    InitASRWorker();
    InitTTSWorker();
    static ros::NodeHandle nh;
    m_botMsgSub = nh.subscribe("/bot_msg", 10, &MainWindow::bot_msg_Callback, this);
    m_userMsgPub = nh.advertise<std_msgs::String>("/user_msg", 10);
    // 初始化ROS服务客户端（替换原TTS线程初始化逻辑）
    tts_client = nh.serviceClient<dialogue_panel::TTS>("tts_request");
}

void MainWindow::InitASRWorker()
{
    // 创建ASR工作对象
    asrWorker = new AsrWorker();
    
    // 启动线程并移动工作对象到该线程
    asrThread.start();
    asrWorker->moveToThread(&asrThread);
    
    // 线程启动信号
    connect(&asrThread, &QThread::started, [this](){
        qDebug() << "ASR线程已启动，ID:" << QThread::currentThreadId();
    });
    
    // 关联工作对象的信号和槽
    // 1. 初始化完成信号
    connect(asrWorker, &AsrWorker::initFinished, this, [this](bool success){
        qDebug() << "[跨线程信号] ASR初始化完成状态:" << success;
        
        // 只有在此处检查初始化状态才是可靠的
        if(success) {
            qDebug() << "ASR初始化成功:"
                << "\n\tFunASRModel实例:" << (pFunASRInstance ? "已创建" : "未创建");
            
            // 启用UI控件
            ui->recordButton->setEnabled(true);
        } else {
            qDebug() << "ASR初始化失败";
            // 禁用依赖于ASR的UI控件
            ui->recordButton->setEnabled(false);
        }
    });
    
    // 2. 识别完成信号（如果需要）
    connect(asrWorker, &AsrWorker::recognitionFinished, this, [this](){
        qDebug() << "语音识别会话结束";
        // 可以在这里添加会话结束后的处理
    });
    
    // 3. 文本更新信号 - 通过ASRNotifier接收识别结果
    connect(ASRNotifier::instance(), &ASRNotifier::textUpdated,
        this, [this](const QString& text){
            // 更新UI文本
            ui->textEdit->setText(text);
            // 强制刷新UI，确保文本立即显示
            QApplication::processEvents();
    });
    
    // 在工作线程中启动初始化
    QMetaObject::invokeMethod(asrWorker, "initialize", Qt::QueuedConnection);
    
    qDebug() << "ASR初始化请求已发送";
}
void MainWindow::InitTTSWorker() {

    // 连接TTS通知器的信号到服务调用
    connect(TTSNotifier::instance(), &TTSNotifier::textSpeaking,
            this, [this](const QString& text) {
        dialogue_panel::TTS srv;
        srv.request.text = text.toStdString();
        if (tts_client.call(srv)) {
            if (srv.response.success) {
                qDebug() << "TTS播放成功";
            } else {
                qDebug() << "TTS播放失败：" << QString::fromStdString(srv.response.error_msg);
            }
        } else {
            qDebug() << "TTS服务调用失败";
        }
    });
}

MainWindow::~MainWindow() 
{
    // 先停止所有线程
    asrThread.quit();
    
    // 等待线程完成
    asrThread.wait();
    
    // 释放资源 - 先释放worker对象
    delete asrWorker;
    
    // 然后释放Python对象
    if (pFunASRInstance) {
        Py_XDECREF(pFunASRInstance);
        pFunASRInstance = nullptr;
    }

    
    // 最后释放其他资源
    if (m_audioRecorder && m_audioRecorder->state() == QMediaRecorder::RecordingState) {
        m_audioRecorder->stop();
    }
    
    delete ui;
    
    // 完成Python清理
    Py_Finalize();
}

// Subscriber Callback函数：订阅bot_msg
void MainWindow::bot_msg_Callback(const std_msgs::String::ConstPtr& msg)
{
    qDebug() << "bot_msg_Callback";
    QMetaObject::invokeMethod(this, [this, msg]()
    {
        QString message = QString::fromStdString(msg->data);
        // 获取当前时间
        QString time = QString::number(QDateTime::currentDateTime().toTime_t());
        qDebug() << "添加消息:" << message << time;
        dealMessageTime(time);
        
        // 创建并添加消息项
        QNChatMessage* messageW = new QNChatMessage(ui->listWidget->parentWidget());
        QListWidgetItem* item = new QListWidgetItem(ui->listWidget);
        dealMessage(messageW, item, message, time, QNChatMessage::User_She);
        ui->listWidget->setCurrentRow(ui->listWidget->count()-1); 
    }, Qt::QueuedConnection);
}

void MainWindow::on_pushButton_clicked()
{
    // 获取文本并清空输入框
    QString msg = ui->textEdit->toPlainText();
    if (msg.isEmpty()) return;  // 避免处理空消息
    
    ui->textEdit->setText("");
    
    // 获取当前时间
    QString time = QString::number(QDateTime::currentDateTime().toTime_t());
    qDebug() << "添加消息:" << msg << time;
    dealMessageTime(time);
    
    // 创建并添加消息项
    QNChatMessage* messageW = new QNChatMessage(ui->listWidget->parentWidget());
    QListWidgetItem* item = new QListWidgetItem(ui->listWidget);
    dealMessage(messageW, item, msg, time, QNChatMessage::User_Me);
    
    // 发送消息到ROS
    std_msgs::String msgData;
    msgData.data = msg.toStdString();
    m_userMsgPub.publish(msgData);

    // 滚动到最新消息
    ui->listWidget->setCurrentRow(ui->listWidget->count()-1);
}

void MainWindow::dealMessage(QNChatMessage *messageW, QListWidgetItem *item, QString text, QString time,  QNChatMessage::User_Type type)
{
    messageW->setFixedWidth(this->width());
    QSize size = messageW->fontRect(text);
    item->setSizeHint(size);
    messageW->setText(text, time, size, type);
    ui->listWidget->setItemWidget(item, messageW);
}

void MainWindow::dealMessageTime(QString curMsgTime)
{
    bool isShowTime = false;
    if(ui->listWidget->count() > 0) {
        QListWidgetItem* lastItem = ui->listWidget->item(ui->listWidget->count() - 1);
        QNChatMessage* messageW = (QNChatMessage*)ui->listWidget->itemWidget(lastItem);
        int lastTime = messageW->time().toInt();
        int curTime = curMsgTime.toInt();
        qDebug() << "curTime lastTime:" << curTime - lastTime;
        isShowTime = ((curTime - lastTime) > 60); // 两个消息相差一分钟
//        isShowTime = true;
    } else {
        isShowTime = true;
    }
    if(isShowTime) {
        QNChatMessage* messageTime = new QNChatMessage(ui->listWidget->parentWidget());
        QListWidgetItem* itemTime = new QListWidgetItem(ui->listWidget);

        QSize size = QSize(this->width(), 40);
        messageTime->resize(size);
        itemTime->setSizeHint(size);
        messageTime->setText(curMsgTime, curMsgTime, size, QNChatMessage::User_Time);
        ui->listWidget->setItemWidget(itemTime, messageTime);
    }
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);


    ui->textEdit->resize(this->width() - 20, ui->widget->height() - 20);
    ui->textEdit->move(10, 10);

    ui->pushButton->move(ui->textEdit->width()+ui->textEdit->x() - ui->pushButton->width() - 10,
                         ui->textEdit->height()+ui->textEdit->y() - ui->pushButton->height() - 10);


    for(int i = 0; i < ui->listWidget->count(); i++) {
        QNChatMessage* messageW = (QNChatMessage*)ui->listWidget->itemWidget(ui->listWidget->item(i));
        QListWidgetItem* item = ui->listWidget->item(i);

        dealMessage(messageW, item, messageW->text(), messageW->time(), messageW->userType());
    }

}

void MainWindow::on_recordButton_pressed()
{
    ui->recordButton->setText("松开停止");

    QMetaObject::invokeMethod(asrWorker, "startRecognition", 
                            Qt::QueuedConnection);
}

void MainWindow::on_recordButton_released()
{
    ui->recordButton->setText("按住说话");

    QMetaObject::invokeMethod(asrWorker, "stopRecognition", 
                            Qt::QueuedConnection);
}