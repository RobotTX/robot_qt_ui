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

    SettingsWidget(QSharedPointer<Robots> robots, QWidget *parent = 0);

    void addRobot(const QString robotIPAddress, const QString robot_name);
    void removeRobot(const QString robotIPAddress);

    QComboBox* getChooseMapBox(void) const { return chooseMapBox; }


private slots:
    void emitLaserSettingChange(int robotId, bool turnOnLaserFeedBack);

signals:
    void laserFeedBack(QString, bool);

private:
    static int currentId;
    QVBoxLayout* menuLayout;
    QButtonGroup* robotsLaserButtonGroup;
    QLabel* feedBackLabel;
    /// to map robot IP addresses and their checkboxes ids
    QMap<int, QString> iDtoIPMap;
    QComboBox* chooseMapBox;
};

#endif /// SETTINGSWIDGET_H
