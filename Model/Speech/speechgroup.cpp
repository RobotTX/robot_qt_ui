#include "speechgroup.h"
#include "Model/Speech/speech.h"

SpeechGroup::SpeechGroup(QObject *parent) : QObject(parent), speechVector(QVector<QPointer<Speech>>()) {}

void SpeechGroup::addSpeech(const QString name, const QString tts) {
    speechVector.push_back(QPointer<Speech>(new Speech(name, tts, this)));
}

void SpeechGroup::addSpeech(const QPointer<Speech> speech) {
    speechVector.push_back(speech);
}

void SpeechGroup::deleteSpeech(const QString name) {
    for (int i = 0; i < speechVector.size(); i++) {
        if (speechVector.at(i)->getName().compare(name) == 0)
            speechVector.remove(i);
    }
}

QPointer<Speech> SpeechGroup::takeSpeech(const QString name) {
    for (int i = 0; i < speechVector.size(); i++) {
        if (speechVector.at(i)->getName().compare(name) == 0)
            return speechVector.takeAt(i);
    }

    /// not supposed to get gere atm as this function is only called to move a speech from a group to another
    Q_UNREACHABLE();
    return Q_NULLPTR;
}
