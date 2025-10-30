

#include "KokoroTTS.h"

TTSNotifier* TTSNotifier::instance() {
    static TTSNotifier notifier;
    return &notifier;
}
