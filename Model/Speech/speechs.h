#ifndef SPEECHS_H
#define SPEECHS_H

class SpeechGroup;

#include <QObject>
#include <QPointer>
#include <QVariant>

/**
 * @brief The Speechs class
 * This class provides a model for a list of speechs organized in groups
 * A Speechs object is identified by a vector of pointers in a map of group identified
 * by their group name
 */

class Speechs : public QObject {

    Q_OBJECT

public:

    Speechs(QObject *parent);

    QMap<QString, QPointer<SpeechGroup>> getGroups(void) const { return groups; }

    void addGroup(const QString groupName);
    void deleteGroup(const QString groupName);
    void addSpeech(const QString groupName, const QString name, const QString tts);
    void deleteSpeech(const QString groupName, const QString name);
    void renameGroup(const QString groupName, const QString name);
    void moveSpeech(const QString name, const QString oldGroup, const QString newGroup);

    /**
     * @brief checkGroupName
     * @param name
     * @return if the given name of group is taken
     */
    bool checkGroupName(const QString name);
    void clearGoups(void);

private:
    QMap<QString, QPointer<SpeechGroup>> groups;
};

#endif // SPEECHS_H
