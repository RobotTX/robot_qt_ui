#ifndef DISPLAYSELECTEDPOINTROBOTS_H
#define DISPLAYSELECTEDPOINTROBOTS_H

class PointView;
class Robots;
class QButtonGroup;
class CustomPushButton;
class QAbstractButton;

#include <QWidget>
#include <QVBoxLayout>
#include <QSharedPointer>
#include <QObject>

/**
 * @brief The DisplaySelectedPointRobots class
 * This class provides a wdiget whose purpose is to display information relative to points being homes or parts of robot paths
 */

class DisplaySelectedPointRobots: public QWidget {
    Q_OBJECT
public:
    DisplaySelectedPointRobots(QWidget* parent);
    void setRobotsWidget(QSharedPointer<PointView> pointView, QSharedPointer<Robots> robots, const QString robotName = "");
    void removeAllPathButtons();
    QWidget* getHomeWidget(void) const { return homeWidget; }

private slots:
    /// shortcuts to go to the robot page
    void robotBtnClicked();
    void pathBtnClicked(QAbstractButton* button);

signals:
    void setSelectedRobotFromPoint(QString);

private:
    QVBoxLayout* layout;
    QWidget* homeWidget;
    CustomPushButton* robotBtn;

    QWidget* pathWidget;
    QButtonGroup* pathBtnGroup;
    QVBoxLayout* pathBtnLayout;
};

#endif // DISPLAYSELECTEDPOINTROBOTS_H
