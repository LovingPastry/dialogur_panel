#ifndef KOKOROTTS_H
#define KOKOROTTS_H

#include <QObject>
#include <QString>
#include <QDebug>

// TTS通知器类 - 单例模式（仅保留信号通知功能）
class TTSNotifier : public QObject {
    Q_OBJECT
public:
    static TTSNotifier *instance();

signals:
    void ttsStarted();
    void ttsFinished();
    void textSpeaking(const QString& text);
};

#endif // KOKOROTTS_H
