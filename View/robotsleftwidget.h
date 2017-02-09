#ifndef ROBOTSLEFTWIDGET_H
#define ROBOTSLEFTWIDGET_H

class Robots;
class CustomScrollArea;
class QVBoxLayout;
class MainWindow;
class TopLeftMenu;
class QHideEvent;

#include <QWidget>
#include <QSharedPointer>
#include "View/robotbtngroup.h"
#include <QAbstractButton>

/**
 * @brief The RobotsLeftWidget class
 * The left menu which display the list of robots and
 * the edit & map button
 */
class RobotsLeftWidget: public QWidget{
public:
    RobotsLeftWidget(MainWindow *mainWindow, QSharedPointer<Robots> const &_robots);

    /// Getters
    RobotBtnGroup* getBtnGroup(void) const { return btnGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QString getSelectedRobotName(void) const { return btnGroup->getBtnGroup()->checkedButton()->text(); }
    int getLastCheckedId(void) const { return lastCheckedId; }

    ///Setters
    void setRobots(QSharedPointer<Robots> const& robots);
    void setLastCheckedId(const int id) { lastCheckedId = id; }

    /**
     * @brief updateRobots
     * @param robots
     * Update the list of robots when needed (e.g. : when a robot's name is edited)
     */
    void updateRobots(QSharedPointer<Robots> const& robots);
    void unSelectAllRobots();

protected:
    void showEvent(QShowEvent *);

private:
    QVBoxLayout* layout;
    QSharedPointer<Robots> robots;
    QVBoxLayout* robotsLayout;
    RobotBtnGroup* btnGroup;
    CustomScrollArea* scrollArea;
    TopLeftMenu* actionButtons;
    int lastCheckedId;

};

#endif // ROBOTSLEFTWIDGET_H
