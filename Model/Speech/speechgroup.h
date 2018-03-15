#ifndef SPEECHGROUP_H
#define SPEECHGROUP_H

class Speech;

#include <QObject>
#include <QPointer>
#include <QVector>

class SpeechGroup : public QObject {

public:
    SpeechGroup(QObject *parent);

    QVector<QPointer<Speech>> getSpeechVector(void) const { return speechVector; }

    void addSpeech(const QPointer<Speech> speech);
    void addSpeech(const QString name, const QString tts);
    void deleteSpeech(const QString name);
    QPointer<Speech> takeSpeech(const QString name);

private:
    QVector<QPointer<Speech>> speechVector;

};

#endif // SPEECHGROUP_H
