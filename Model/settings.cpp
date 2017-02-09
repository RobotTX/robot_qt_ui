#include "settings.h"
#include <QDebug>
#include <QDir>
#include <fstream>
#include <assert.h>
#include <QDataStream>

Settings::Settings(const int _settingMapChoice, const int _batteryWarningThreshHold):
    settingMapChoice(_settingMapChoice), batteryWarningThreshHold(_batteryWarningThreshHold)
{
    /// TODO add all the functions for which a message is needed and call openHelpMessage where the function to enable the feature is called
    activated_messages["merge_maps"] = true;
    activated_messages["scan"] = true;
    activated_messages["edit_map"] = true;
    activated_messages["recover_robot_position"] = true;
}

int Settings::currentId = 0;

void Settings::addRobot(const QString robot_name){

    QString fileStr = QDir::currentPath() + QDir::separator() + "settings" + QDir::separator() + robot_name + "_laser.txt";
    std::ifstream fileRobot(fileStr.toStdString(), std::ios::in);

    bool laser(true);

    if(fileRobot){
        fileRobot >> laser;
        fileRobot.close();
    }

    idToNameMap.insert(++currentId, QPair<QString, bool>(robot_name, laser));
}


void Settings::removeRobot(const QString robot_name){
    QMapIterator<int, QPair<QString, bool>> it(idToNameMap);
    while(it.hasNext()){
        it.next();
        if(!it.value().first.compare(robot_name)){
            idToNameMap.remove(it.key());
            break;
        }
    }
}

/**
 * @brief Settings::setLaserStatus
 * @param id
 * @param status
 * @return bool
 * returns true if the file was updated with the new status, false otherwise
 */
bool Settings::setLaserStatus(const int id, const bool status){
    qDebug() << "Settings::setLaserStatus " << id << status;
    QMapIterator<int, QPair<QString, bool> > it(idToNameMap);
    QString name;
    while(it.hasNext()){
        it.next();
        qDebug() << "next id " << it.key();
        if(it.key() == id)
            name = it.value().first;
    }
    QString fileStr = QDir::currentPath() + QDir::separator() + "settings" + QDir::separator() + name + "_laser.txt";
    std::ofstream fileLaser(fileStr.toStdString(), std::ios::out | std::ios::trunc);
    if(fileLaser) {
        fileLaser << status;
        fileLaser.close();
        QMapIterator<int, QPair<QString, bool> > it(idToNameMap);
        while(it.hasNext()){
            it.next();
            if(it.key() == id)
                idToNameMap[id].second = status;
        }
        return true;
    }
    return false;
}

/// resets the tutorial
void Settings::resetSettings(){
    /// to modify the map on the fly
    QMutableMapIterator<QString, bool> it(activated_messages);
    while(it.hasNext()){
        it.next();
        it.setValue(true);
    }
}

QDataStream& operator>>(QDataStream& in, Settings& settings){
    qint32 mapChoice(-1);
    int batteryThresh(0);
    QMap<QString, bool> tutorialMessages;
    in >> mapChoice >> batteryThresh >> tutorialMessages;
    settings.setMapChoice(mapChoice);
    settings.setBatteryThreshold(batteryThresh);
    settings.setTutorialMap(tutorialMessages);
    return in;
}

QDataStream& operator<<(QDataStream& out, const Settings& settings){
    out << settings.getSettingMapChoice() << settings.getBatteryWarningThreshold() << settings.getTutorialMap();
    return out;
}
