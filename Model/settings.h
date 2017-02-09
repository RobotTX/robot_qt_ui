#ifndef SETTINGS_H
#define SETTINGS_H

#include <QMap>
#include <QObject>
#include <QDebug>

class Settings: public QObject
{
    Q_OBJECT

public:
    Settings(const int _settingMapChoice = 0, const int _batteryWarningThreshHold = 20);

    int getSettingMapChoice(void) const { return settingMapChoice; }
    QMap<int, QPair<QString, bool>> getIDtoNameMap(void) const { return idToNameMap; }
    int getBatteryWarningThreshold(void) const { return batteryWarningThreshHold; }
    bool getHelpNeeded(const QString feature) const { return activated_messages[feature]; }
    QMap<QString, bool> getTutorialMap(void) const { return activated_messages; }

    static int getCurrentId(void) { return currentId; }

    void setMapChoice(const int choice) { settingMapChoice = choice; }
    void setBatteryThreshold(const int thresh) { batteryWarningThreshHold = thresh; }
    bool setLaserStatus(const int id, const bool status);
    void setHelpNeeded(const bool help, const QString feature) { activated_messages[feature] = !help; qDebug() << "Settings setHelpNeeded" << feature << !help; }
    void setTutorialMap(const QMap<QString, bool>& tuto) { activated_messages = tuto; }

    void resetSettings(void);

public:
    /// to keep track of the laser feedbacks
    void addRobot(const QString robot_name);
    void removeRobot(const QString robot_name);

private:
    /// the id to give to the next robot
    static int currentId;
    /// matches ids and robot's names, the bool indicates whether or not the laser feedback is activated
    QMap<int, QPair<QString, bool>> idToNameMap;
    int settingMapChoice;
    int batteryWarningThreshHold;
    /// indicates whether or not we give help messages to the user
    QMap<QString, bool> activated_messages;
};

QDataStream& operator>>(QDataStream& in, Settings& settings);

QDataStream& operator<<(QDataStream& out, const Settings& settings);


#endif /// SETTINGS_H
