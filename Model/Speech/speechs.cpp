#include <QDataStream>
#include <QDebug>
#include <QString>
#include <QString>
#include "speechs.h"
#include "Helper/helper.h"
#include "Model/Speech/speech.h"
#include "Model/Speech/speechgroup.h"


Speechs::Speechs(QObject *parent) : QObject(parent), groups(QMap<QString, QPointer<SpeechGroup>>()) {}

void Speechs::addGroup(const QString groupName){
    groups.insert(groupName, QPointer<SpeechGroup>(new SpeechGroup(this)));
}

void Speechs::addSpeech(const QString groupName, const QString name, const QString tts){
    groups.value(groupName)->addSpeech(name, tts);
}

void Speechs::deleteSpeech(const QString groupName, const QString name){
    qDebug() << "Speechs::deleteSpeech" << groupName << name;
    groups.value(groupName)->deleteSpeech(name);
}

void Speechs::deleteGroup(const QString groupName){
    groups.remove(groupName);
}

void Speechs::renameGroup(const QString newName, const QString oldName){
    groups.insert(newName, groups.take(oldName));
}

void Speechs::moveSpeech(const QString name, const QString oldGroup, const QString newGroup){
    groups.value(newGroup)->addSpeech(groups.value(oldGroup)->takeSpeech(name));
}

bool Speechs::checkGroupName(const QString name){
    return name.isEmpty() || groups.find(name) != groups.end();
}

void Speechs::clearGoups(){
    groups.clear();
}
