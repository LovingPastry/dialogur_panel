#ifdef __cplusplus
extern "C" {
#endif

#include <Python.h>

#ifdef __cplusplus

}

#endif

#include "funASR.h"
#include <QCoreApplication>
#include <QDebug>
#include <QMutex>
#include <QDir>
#include <QString>
#include <QThread>
#include <ros/ros.h>
#include <ros/package.h>

PyObject* pModule = nullptr;
PyObject* pFunASRInstance = nullptr;

ASRNotifier* ASRNotifier::instance() {
    static ASRNotifier notifier;
    return &notifier;
}

// 添加静态回调函数封装
static PyObject* cpp_callback_wrapper(PyObject* self, PyObject* args) {
    char* text;
    qDebug() << "进入cpp_callback_wrapper";
    if (!PyArg_ParseTuple(args, "s", &text)) {
        return NULL;
    }
    qDebug() << "参数解析成功";
    // 通过Qt信号机制传递结果到主线程
    Q_EMIT ASRNotifier::instance()->textUpdated(QString(text));
    qDebug() << "Qt信号传递结果到主线程";
    Py_RETURN_NONE;
}

static PyMethodDef callbackDef = {
    "cpp_callback",
    cpp_callback_wrapper,
    METH_VARARGS,
    "C++ callback wrapper"
};

AsrWorker::AsrWorker() : running(false), initialized(false) {}

AsrWorker::~AsrWorker() {
    // 确保释放资源
    if (running) {
        stopRecognition();
    }
}

void AsrWorker::initialize() {
    qDebug() << "正在执行AsrWorker初始化，线程ID:" << QThread::currentThreadId();
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
        PyEval_SaveThread(); // 释放GIL

    }

    
    
    PyGILState_STATE gstate = PyGILState_Ensure();
    qDebug() << "进入initialize函数，线程ID:" << QThread::currentThreadId();
    try
    {
        // 获取包路径（添加日志验证）
        std::string package_path = ros::package::getPath("dialogue_panel");
        qDebug() << "dialogue_panel包路径:" << QString::fromStdString(package_path);  // 新增日志
        
        std::string scriptPath = package_path + "/script";
        std::string srcPath = package_path + "/src";
        qDebug() << "script路径:" << QString::fromStdString(scriptPath);  // 新增日志
        qDebug() << "src路径:" << QString::fromStdString(srcPath);  // 新增日志
        
        std::string cmd = 
            "import sys\n"
            "sys.path.insert(0, r'" + scriptPath + "')\n" 
            "sys.path.insert(0, r'" + srcPath + "')\n";
        
        // 执行路径设置并验证（添加错误日志）
        if (PyRun_SimpleString(cmd.c_str()) == -1) {
            PyErr_Print();  // 打印Python错误栈
            throw std::runtime_error("Failed to set Python path");
        }
        qDebug() << "Python路径设置成功";  // 新增日志
        
        // 导入模块并获取类（添加加载状态日志）
        pModule = PyImport_ImportModule("asr");
        if (!pModule) {
            PyErr_Print();  // 打印Python错误栈
            throw std::runtime_error("Failed to load asr module");
        }
        qDebug() << "asr模块导入成功";  // 新增日志
        
        // 获取FunASRModel类
        PyObject* pClass = PyObject_GetAttrString(pModule, "FunASRModel");
        if (!pClass) {
            PyErr_Print();
            throw std::runtime_error("Failed to get FunASRModel class");
        }
        qDebug() << "FunASRModel类获取成功";  // 新增日志
        
        pFunASRInstance = PyObject_CallObject(pClass, nullptr);
        Py_DECREF(pClass);
        if (!pFunASRInstance) {
            PyErr_Print();
            throw std::runtime_error("Failed to create FunASRModel instance");
        }
        qDebug() << "FunASRModel实例创建成功";  // 新增日志

        // 设置回调函数
        PyObject* pModule = PyImport_AddModule("__main__");  
        PyObject* pCallback = PyCFunction_NewEx(&callbackDef, nullptr, pModule);
        PyObject_SetAttrString(pFunASRInstance, "cpp_callback", pCallback);
        Py_DECREF(pCallback);
        
        if (PyObject_HasAttrString(pFunASRInstance, "cpp_callback")) {
            qDebug() << "回调函数设置成功";
        } else {
            qDebug() << "回调函数设置失败";
        }
        
        initialized = true;
    }
    catch (const std::exception& e) {
        qDebug() << "Error in initialize:" << e.what();  // 新增异常日志
        
        // 清理
        Py_XDECREF(pModule);
        Py_XDECREF(pFunASRInstance);
        pModule = nullptr;
        pFunASRInstance = nullptr;
        
        PyGILState_Release(gstate);
        emit initFinished(false);
        return;
    }

    PyGILState_Release(gstate);
    emit initFinished(true);
}

void AsrWorker::startRecognition() {
    if (!initialized || running) {
        return;
    }
    
    running = true;
    
    // 使用Qt线程机制调用处理函数
    QMetaObject::invokeMethod(this, "processMicrophoneInput", 
                             Qt::QueuedConnection);
}

void AsrWorker::processMicrophoneInput() {
    if (!pFunASRInstance) {
        qDebug() << "ASR实例未初始化";
        running = false;
        return;
    }

    // 设置环境变量确保Python输出不被缓存
    putenv((char*)"PYTHONUNBUFFERED=1");

    // 启动独立线程执行推理（避免阻塞ASR工作线程的事件循环）
    QThread* inferThread = new QThread();
    QObject* worker = new QObject();
    worker->moveToThread(inferThread);

    QMetaObject::Connection* conn = new QMetaObject::Connection();
    *conn = connect(inferThread, &QThread::started, [this, worker, inferThread]() {
        PyGILState_STATE gstate = PyGILState_Ensure();
        try {
            // 调用Python的infer_by_microphone方法（阻塞当前子线程）
            PyObject* pResult = PyObject_CallMethodObjArgs(
                pFunASRInstance, 
                PyUnicode_FromString("infer_by_microphone"), 
                Py_True, 
                NULL
            );
            if (!pResult) {
                PyErr_Print();
                qDebug() << "ASR方法调用失败";
            } else {
                Py_DECREF(pResult);
            }
        } catch (const std::exception& e) {
            qDebug() << "Error in processMicrophoneInput:" << e.what();
        }
        PyGILState_Release(gstate);

        // 推理完成后清理资源
        running = false;
        emit recognitionFinished();
        inferThread->quit();
        worker->deleteLater();
        inferThread->deleteLater();
    });

    inferThread->start();
}

void AsrWorker::stopRecognition() {
    if (!pFunASRInstance) {
        qDebug() << "stopRecognition: Python ASR 实例不存在";
        return;
    }

    qDebug() << "stopRecognition: 调用Python的stop方法";

    // 调用 Python 的 stop() 方法
    PyGILState_STATE gstate = PyGILState_Ensure();
    PyObject* result = PyObject_CallMethod(pFunASRInstance, "stop", NULL);
    if (!result) {
        PyErr_Print();
        qDebug() << "stopRecognition: 调用Python的stop方法失败";
        return;
    } else {
        Py_DECREF(result);
        qDebug() << "stopRecognition: 调用Python的stop方法成功";
    }

    // 等待 Python 的识别线程退出（self.running == False）
    while (true) {
        PyObject* is_running = PyObject_CallMethod(pFunASRInstance, "is_running", NULL);
        if (!is_running) {
            PyErr_Print();
            qDebug() << "stopRecognition: 获取 is_running 状态失败";
            break;
        }

        int running = PyObject_IsTrue(is_running);
        Py_DECREF(is_running);

        if (running == 0) {
            // 已停止
            qDebug() << "stopRecognition: Python识别线程已停止";
            break;
        }

        // 等待一会儿再检查
        QThread::msleep(50);
    }
    PyGILState_Release(gstate);

    qDebug() << "stopRecognition: 完成";
}
