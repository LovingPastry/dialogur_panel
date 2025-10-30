#ifndef FUNASR_H
#define FUNASR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Python.h>

#ifdef __cplusplus
}
#endif

#include <QCoreApplication>
#include <functional>
#include <QObject>

// 全局实例
extern PyObject* pFunASRInstance;
extern PyObject* pModule;

class ASRNotifier : public QObject {
    Q_OBJECT
public:
    static ASRNotifier* instance();
signals:
    void textUpdated(const QString& text);
};

class AsrWorker : public QObject {
    Q_OBJECT
public:
    AsrWorker();
    ~AsrWorker();

public slots:  // 新增：声明为public slots
    void initialize();
    void startRecognition();
    void stopRecognition();
    void processMicrophoneInput();  // 移动到public slots，并调整访问权限为public

private:
    bool running;
    bool initialized;

signals:
    void initFinished(bool success);
    void recognitionFinished();
};

#endif // FUNASR_H

