#ifndef SETTINGSWIDGET_H
#define SETTINGSWIDGET_H

class QVBoxLayout;
class QCheckBox;
class QButtonGroup;
class QLabel;
class QComboBox;
class QCheckBox;
class CustomPushButton;

#include <QSlider>
#include "Model/Robots/robots.h"
#include "Model/Settings/settings.h"
#include <QMap>
#include <QWidget>
#include <QDebug>
#include <QObject>


class SettingsWidget: public QWidget
{
    Q_OBJECT
public:

    enum SETTING_MAP_CHOICE { ALWAYS_ROBOT, ALWAYS_APPLICATION, ALWAYS_ASK , ALWAYS_NEW , ALWAYS_OLD };

    SettingsWidget(const Settings &settings, QWidget* parent = 0);

    void addRobot(const QString robot_name, const int currentId, const bool laser);
    void removeRobot(const int robotId);

    QComboBox* getChooseMapBox(void) const { return chooseMapBox; }
    QSlider* getBatterySlider(void) const { return batteryThresholdSlider; }
    QButtonGroup* getRobotLaserButtonGroup(void) { return robotsLaserButtonGroup; }
    CustomPushButton* getHelpButton(void) const { return helpButton; }

signals:
    void updateMapChoice(int);

private slots:
    void applySlot();
    void cancelSlot();
    void saveSlot();

private:
    static int currentId;
    QVBoxLayout* robotsLaserLayout;
    QButtonGroup* robotsLaserButtonGroup;
    QLabel* feedBackLabel;
    QLabel* chooseMapLabel;
    QComboBox* chooseMapBox;
    QSlider* batteryThresholdSlider;
    QLabel* batteryThresholdLabel;
    CustomPushButton* helpButton;
};

#endif /// SETTINGSWIDGET_H
