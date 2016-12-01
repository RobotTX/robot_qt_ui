#ifndef SETTINGSWIDGET_H
#define SETTINGSWIDGET_H

class QVBoxLayout;
class QCheckBox;
class QButtonGroup;
class QLabel;
class QComboBox;

#include "Model/robots.h"
#include <QMap>
#include <QWidget>

class SettingsWidget: public QWidget
{
    Q_OBJECT
public:

    enum SETTING_MAP_CHOICE { ALWAYS_ROBOT, ALWAYS_APPLICATION, ALWAYS_ASK , ALWAYS_NEW , ALWAYS_OLD };

    SettingsWidget(QSharedPointer<Robots> robots, int settingMapChoice, QWidget *parent = 0);

    void addRobot(const QString robotIPAddress, const QString robot_name);
    void removeRobot(const QString robotIPAddress);

    QComboBox* getChooseMapBox(void) const { return chooseMapBox; }
    int getSettingMapChoice(void) const { return settingMapChoice; }
    QMap<int, QPair<QString, bool>> getIDtoIPMap(void) const { return idToIpMap; }

signals:
    void activateLaser(QString, bool);

private slots:
    void applySlot();
    void cancelSlot();
    void saveSlot();

private:
    static int currentId;
    QVBoxLayout* robotsLaserLayout;
    QButtonGroup* robotsLaserButtonGroup;
    QLabel* feedBackLabel;
    /// to map robot IP addresses and their checkboxes ids
    QMap<int, QPair<QString, bool>> idToIpMap;
    QLabel* chooseMapLabel;
    QComboBox* chooseMapBox;
    int settingMapChoice;
};

#endif /// SETTINGSWIDGET_H
