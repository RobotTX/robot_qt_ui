#ifndef SPEECHCONTROLLER_H
#define SPEECHCONTROLLER_H

class Speechs;
class MainController;

#include <QObject>
#include <QVariant>
#include <QPointer>

class SpeechController : public QObject {

    Q_OBJECT

public:
    SpeechController(QObject *applicationWindow, MainController *parent);

    /// Getter
    QPointer<Speechs> getSpeechs(void) const { return speechs; }

    /**
     * @brief checkErrorSpeech
     * @param name
     * Check if there is any error in the speech name
     */

    void checkErrorSpeech(const QString name, const QString oldName);

    /**
     * @brief checkSpeechName
     * @param name
     * @return if the given name of speech is taken
     */
    bool checkSpeechName(const QString name);

    /**
     * @brief clearSpeechs
     * Delete all the speechs on the c++ and qml side
     */
    void clearSpeechs(void);

public slots:
    /**
     * @brief addSpeech
     * @param name
     * @param groupName
     * @param tts
     * Add a speech to the model
     */
    void addSpeech(const QString name, const QString groupName, const QString tts, const QString oldName = "", const QString oldGroup = "", const bool saveXML = true);

    /**
     * @brief addGroup
     * @param groupName
     * Add a group to the model
     */
    void addGroup(QString groupName, bool saveXML = true);

private:
    /**
     * @brief loadSpeechs
     * @param fileName
     * Load the speechs form the XML file
     */
    void loadSpeechs(const QString fileName);

private slots:
    /**
     * @brief checkGroup
     * @param name
     * Check if the given name is already taken by a group and send a signal to qml
     */
    void checkGroup(QString name);

    /**
     * @brief deleteSpeech
     * @param name
     * @param groupName
     * Delete a speech from the model
     */
    void deleteSpeech(QString groupName, QString name);

    /**
     * @brief deleteGroup
     * @param name
     * Delete a group and its points from the model
     */
    void deleteGroup(QString name);

    /**
     * @brief renameGroup
     * @param newName
     * @param oldName
     * Rename a group from oldName to newName
     */
    void renameGroup(QString newName, QString oldName);

    /**
     * @brief moveTo
     * @param name
     * @param oldName
     * @param newGroup
     * Move a speech from oldGroup to newGroup
     */
    void moveTo(QString name, QString oldGroup, QString newGroup);

signals:
    /**
     * @brief enableSpeechSaveQml
     * @param enable
     * Signal to enable the save button while creating/editing a speech
     */
    void enableSpeechSaveQml(QVariant nameError);

    /**
     * @brief enableGroupSaveQml
     * @param enable
     * Signal to enable the save button while creating/editing a group
     */
    void enableGroupSaveQml(QVariant enable);

    /**
     * @brief addGroupQml
     * @param name
     * Tell the qml model that we added a new group
     */
    void addGroupQml(QVariant name);

    /**
     * @brief addSpeechQml
     * @param name
     * @param groupName
     * @param tts
     * Tell the qml model that we added a new speech
     */
    void addSpeechQml(QVariant name, QVariant groupName, QVariant tts);

    /**
     * @brief editSpeechQml
     * @param oldName
     * @param oldGroup
     * @param name
     * @param groupName
     * @param tts
     * Tell the qml model that we edited a speech
     */
    void editSpeechQml(QVariant oldName, QVariant oldGroup, QVariant name, QVariant groupName, QVariant tts);

    /**
     * @brief renameGroupQml
     * @param newName
     * @param oldName
     * Tell the qml model that we renamed the group oldName into newName
     */
    void renameGroupQml(QVariant newName, QVariant oldName);

    /**
     * @brief deleteGroupQml
     * @param groupName
     * Tells the qml model to delete the group <groupName>
     */
    void deleteAllGroupsQml();

private:
    QPointer<Speechs> speechs;
    QString currentSpeechsFile;
    QString robotSpeechsFile;


};

#endif // SPEECHCONTROLLER_H
