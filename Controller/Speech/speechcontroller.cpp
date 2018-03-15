#include "speechcontroller.h"
#include <QApplication>
#include <QDebug>
#include <QImage>
#include <QDir>
#include <QStandardPaths>
#include "Controller/maincontroller.h"
#include "Model/Speech/speech.h"
#include "Model/Speech/speechgroup.h"
#include "Model/Speech/speechs.h"
#include "Model/Speech/speechxmlparser.h"
#include "Helper/helper.h"

SpeechController::SpeechController(QObject *applicationWindow, MainController *parent) : QObject(parent) {
    speechs = QPointer<Speechs>(new Speechs(this));

    QObject *speechModel = applicationWindow->findChild<QObject*>("speechModel");

    if (speechModel) {
        /// Tell the qml speech model that we just added a new group
        connect(this, SIGNAL(addGroupQml(QVariant)), speechModel, SLOT(addGroup(QVariant)));
        /// Tell the qml speech model that we just added a new speech
        connect(this, SIGNAL(addSpeechQml(QVariant, QVariant, QVariant)),
                speechModel, SLOT(addSpeech(QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(editSpeechQml(QVariant, QVariant, QVariant, QVariant, QVariant)),
                speechModel, SLOT(editSpeech(QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(deleteAllGroupsQml()), speechModel, SLOT(deleteAllGroups()));
        /// Tell the qml speech model that we just renamed a group
        connect(this, SIGNAL(renameGroupQml(QVariant, QVariant)), speechModel, SLOT(renameGroup(QVariant, QVariant)));
        connect(speechModel, SIGNAL(deleteSpeechSignal(QString, QString)), this, SLOT(deleteSpeech(QString, QString)));
        connect(speechModel, SIGNAL(deleteGroupSignal(QString)), this, SLOT(deleteGroup(QString)));
        connect(speechModel, SIGNAL(moveToSignal(QString, QString, QString)), this, SLOT(moveTo(QString, QString, QString)));

    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "SpeechController::SpeechController could not find the qml speech model";
        Q_UNREACHABLE();
    }

    QObject *createSpeechMenuFrame = applicationWindow->findChild<QObject*>("createSpeechMenuFrame");
    if (createSpeechMenuFrame){
        /// Tell the menu where we create this that we enable the save button
        connect(this, SIGNAL(enableSpeechSaveQml(QVariant)), createSpeechMenuFrame, SLOT(enableSave(QVariant)));
        /// Got a modification of the name of a speech we are creating so we check to enable or not the save button
        connect(createSpeechMenuFrame, SIGNAL(checkSpeech(QString, QString)), parent, SLOT(checkSpeech(QString, QString)));
        /// Clicked on the save button to create the given speech
        connect(createSpeechMenuFrame, SIGNAL(createSpeech(QString, QString, QString, QString, QString)), this, SLOT(addSpeech(QString,QString,QString,QString, QString)));
    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "SpeechController::SpeechController could not find the createSpeechMenuFrame";
        Q_UNREACHABLE();
    }

    QObject *createSpeechGroupMenu = applicationWindow->findChild<QObject*>("createSpeechGroupMenu");
    if (createSpeechGroupMenu){
        /// Tell the menu where we create groups that we enable the save button
        connect(this, SIGNAL(enableGroupSaveQml(QVariant)), createSpeechGroupMenu, SLOT(enableSave(QVariant)));
        /// The group name has been modified so we check if it's taken to enable or not the save button
        connect(createSpeechGroupMenu, SIGNAL(checkGroup(QString)), this, SLOT(checkGroup(QString)));
        /// Clicked on the save button to create the given group
        connect(createSpeechGroupMenu, SIGNAL(createGroup(QString)), this, SLOT(addGroup(QString)));
        /// Clicked on the save button while editing a group
        connect(createSpeechGroupMenu, SIGNAL(renameGroup(QString, QString)), this, SLOT(renameGroup(QString, QString)));
    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "SpeechController::SpeechController could not find the createSpeechMenuFrame";
        Q_UNREACHABLE();
    }

    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";

    /// desktop
//    currentSpeechsFile = Helper::getAppPath() + QDir::separator() + "currentSpeechs.xml";

    /// android
    currentSpeechsFile = location + QDir::separator() + "currentSpeechs.xml";
    qDebug() << "SpeechController::SpeechController" << currentSpeechsFile;
    loadSpeechs(currentSpeechsFile);
}

void SpeechController::loadSpeechs(const QString fileName){
    SpeechXMLParser::readSpeechs(this, fileName);
}

void SpeechController::addGroup(QString groupName, bool saveXML){
    if(!speechs->getGroups().contains(groupName)){
        speechs->addGroup(groupName);
        emit addGroupQml(groupName);
        if(saveXML)
            SpeechXMLParser::save(this, currentSpeechsFile);
    }
}

void SpeechController::addSpeech(const QString name, const QString groupName, const QString tts, const QString oldName, const QString oldGroup,  bool saveXML)
{
    addGroup(groupName, saveXML);
    speechs->addSpeech(groupName, name, tts);

    /// We are creating a new speech
    if(oldName.isEmpty())
        emit addSpeechQml(name, groupName, tts);
    else {
        deleteSpeech(oldGroup, oldName);
        emit editSpeechQml(oldName, oldGroup, name, groupName, tts);
        qDebug() << "editSpeechQml editing speech in addSpech speechcontroller.cpp";
    }
    if(saveXML)
        SpeechXMLParser::save(this, currentSpeechsFile);
}

void SpeechController::deleteSpeech(QString groupName, QString name){
    qDebug() << "Delete speech";
    /// we remove the speech from the c++ side
    speechs->deleteSpeech(groupName, name);
    SpeechXMLParser::save(this, currentSpeechsFile);
}

void SpeechController::deleteGroup(QString groupName){
    speechs->deleteGroup(groupName);
    SpeechXMLParser::save(this, currentSpeechsFile);
}

bool SpeechController::checkSpeechName(const QString name){
    QMapIterator<QString, QPointer<SpeechGroup>> i(speechs->getGroups());
    while (i.hasNext()) {
        i.next();
        QVector<QPointer<Speech>> group = i.value()->getSpeechVector();
        for(int j = 0; j < group.size(); j++){
            if(group.at(j)->getName().compare(name) == 0)
                return true;
        }
    }
    return false;
}

void SpeechController::renameGroup(QString newName, QString oldName){
    qDebug() << "SpeechController::renameGroup from" << oldName << "to" << newName;
    speechs->renameGroup(newName, oldName);
    emit renameGroupQml(newName, oldName);
    SpeechXMLParser::save(this, currentSpeechsFile);
}

void SpeechController::moveTo(QString name, QString oldGroup, QString newGroup){
    qDebug() << "SpeechController::move" << name << "from" << oldGroup << "to" << newGroup;
    speechs->moveSpeech(name, oldGroup, newGroup);
    SpeechXMLParser::save(this, currentSpeechsFile);
}

void SpeechController::checkErrorSpeech(const QString name, const QString oldName){
    bool nameError = false;

    /// Name not empty
    nameError = name.isEmpty();

    /// Check if the name is taken by another speech
    if(!nameError && name.compare(oldName) != 0)
        nameError = checkSpeechName(name);

    /// Send the result to qml to enable or not the save button
    emit enableSpeechSaveQml(nameError);
}

void SpeechController::checkGroup(QString name){
    /// Check if the name of the group is already taken and send the result to enable or not the save button
    emit enableGroupSaveQml(!speechs->checkGroupName(name));
}

void SpeechController::clearSpeechs(){
    qDebug() << "SpeechController::clearSpeechs called";
    emit deleteAllGroupsQml();
    speechs->clearGoups();
}

