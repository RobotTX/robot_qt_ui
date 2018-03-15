#ifndef SPEECH_H
#define SPEECH_H

#include <QObject>

/**
 * @brief The Speech class
 * This class provides a model for a speech
 * A speech is identified by a name that is unique, and a text
 */

class Speech : public QObject {

public:
    Speech(const QString _name, const QString _tts, QObject *parent);

    /// Getters
    QString getName(void) const { return name; }
    QString getTts(void) const { return tts; }

    /// Setters
    void setName(const QString _name) { name  = _name; }
    void setTts(const QString _tts) { tts = _tts; }

private:
    QString name;
    QString tts;
};

#endif // SPEECH_H
