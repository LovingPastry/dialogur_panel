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
#include "mainwindow.h"
#include <QApplication>
#include <QTimer>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <cstdlib>  // 新增：用于setenv

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "dialogue_panel");
    QApplication a(argc, argv);
    
    if (!Py_IsInitialized()) {  // 仅在未初始化时设置环境并初始化
        const char* python_home = "/home/sumi/anaconda3/envs/igibson";  // 替换为你的虚拟环境路径
        const char* python_site_packages = "/home/sumi/anaconda3/envs/igibson/lib/python3.8/site-packages";  // 替换为你的site-packages路径

        // 设置PYTHONHOME（指定Python主目录）
        setenv("PYTHONHOME", python_home, 1);
        // 设置PYTHONPATH（包含目标环境的site-packages）
        setenv("PYTHONPATH", python_site_packages, 1);
        // 通过Py_SetPythonHome强化路径（需在Py_Initialize前调用）
        wchar_t* w_python_home = Py_DecodeLocale(python_home, nullptr);
        Py_SetPythonHome(w_python_home);
        PyMem_RawFree(w_python_home);  // 释放临时内存

        Py_Initialize();
        PyEval_InitThreads(); 

    }

    PyEval_SaveThread(); // 释放GIL
    
    // 添加调试锚点
    #ifdef _DEBUG
    QThread::sleep(2); // 给调试器附加时间
    #endif

    
    // 添加事件循环调试钩子
    QObject::connect(&a, &QApplication::aboutToQuit, [](){
        qDebug() << "Event loop exited"; // 调试输出
    });
    
    MainWindow w;
    #ifdef STANDALONE_DEBUG
        QMainWindow standaloneWindow;
        w.setParent(&standaloneWindow);
        standaloneWindow.show();
    #else
        w.show();
    #endif


    // 添加ROS消息处理定时器
    QTimer rosSpinTimer;
    QObject::connect(&rosSpinTimer, &QTimer::timeout, [](){
        ros::spinOnce(); // 每10ms处理一次ROS消息
    });
    rosSpinTimer.start(10); // 启动定时器

    int ret = a.exec();
    
    // 添加线程同步
    w.~MainWindow(); // 显式调用析构保证析构顺序
    Py_Finalize();   // 确保所有线程结束后才调用
    return ret;
}
