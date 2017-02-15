#ifndef ROBOTSCONTROLLER_H
#define ROBOTSCONTROLLER_H

class RobotServerWorker;
class Robots;
class RobotView;

#include <QObject>
#include <QThread>
#include "Controller/mainwindow.h"

class RobotsController : public QObject {
    Q_OBJECT
public:
    RobotsController(MainWindow* mainWindow);
    ~RobotsController();

    /// Getters
    QSharedPointer<Robots> getRobots(void) const { return robots; }
    QPointer<RobotView> getSelectedRobot(void) const { return selectedRobot; }
    RobotsLeftWidget* getRobotsLeftWidget(void) const {return robotsLeftWidget;}
    EditSelectedRobotWidget* getEditSelectedRobotWidget(void) const {return editSelectedRobotWidget;}

    /// Setters
    void setSelectedRobot(QPointer<RobotView> _selectedRobot){ selectedRobot = _selectedRobot; }
    void resetSelectedRobot(void){ selectedRobot = Q_NULLPTR; }

    void initializeMenus(MainWindow* mainWindow);
    void launchServer(MainWindow* mainWindow);
    void updateRobotsLeftWidget(void);

signals:
    void stopUpdateRobotsThread();
    void scanRobotPos(QString, double, double, double);
    /// to stop displaying all paths on the map
    void clearMapOfPaths();
    /// to display the path whose groupname and pathname are propagated by the signal
    void showPath(QString, QString);

private slots:
    void updateRobot(const QString ipAddress, const float posX, const float posY, const float ori);
    void updatePathsMenuEditSelectedRobotWidget(bool openMenu);
    void updateHomeMenuEditSelectedRobotWidget(bool openMenu);
    /// called after the main window has received ackownledgement from the robot (update path)
    void applyNewPath(const QString groupName, const QString pathName);

private:
    RobotServerWorker* robotServerWorker;
    QThread serverThread;
    QSharedPointer<Robots> robots;
    QPointer<RobotView> selectedRobot;

    RobotsLeftWidget* robotsLeftWidget;
    EditSelectedRobotWidget* editSelectedRobotWidget;
};

#endif // ROBOTSCONTROLLER_H
