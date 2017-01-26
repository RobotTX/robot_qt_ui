#ifndef SETTINGS_H
#define SETTINGS_H

#include <QMap>
#include <QObject>

class Settings: public QObject
{
    Q_OBJECT
public:
    Settings(const int _settingMapChoice = 0, const int _batteryWarningThreshHold = 20, const bool _helpNeeded = true);

    int getSettingMapChoice(void) const { return settingMapChoice; }
    QMap<int, QPair<QString, bool>> getIDtoNameMap(void) const { return idToNameMap; }
    int getBatteryWarningThreshold(void) const { return batteryWarningThreshHold; }
    void setMapChoice(const int choice) { settingMapChoice = choice; }
    void setBatteryThreshold(const int thresh) { batteryWarningThreshHold = thresh; }
    bool setLaserStatus(const int id, const bool status);
    void setHelpNeeded(const bool help) { helpNeeded = help; }

    static int getCurrentId(void) { return currentId; }
    bool getHelpNeeded(void) const { return helpNeeded; }

public:
    /// to keep track of the laser feedbacks
    void addRobot(const QString robot_name);
    void removeRobot(const QString robot_name);

private:
    /// the id to give to the next robot
    static int currentId;
    /// matches ids and robot's names
    QMap<int, QPair<QString, bool>> idToNameMap;
    int settingMapChoice;
    int batteryWarningThreshHold;
    /// when an important function is called, display a message box explaining the user how to use it
    /// unless the boolean corresponding to this particular function is set to "false"
    bool helpNeeded;
};

#endif /// SETTINGS_H
