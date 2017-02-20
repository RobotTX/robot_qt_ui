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
#include "View/Robots/robotbtngroup.h"
#include <QAbstractButton>

/**
 * @brief The RobotsLeftWidget class
 * The left menu which display the list of robots and
 * the edit & map button
 */
class RobotsLeftWidget: public QWidget{
    Q_OBJECT
public:
    RobotsLeftWidget(MainWindow *mainWindow);

    /// Getters
    RobotBtnGroup* getBtnGroup(void) const { return btnGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QString getSelectedRobotName(void) const { return btnGroup->getBtnGroup()->checkedButton()->text(); }
    int getLastCheckedId(void) const { return lastCheckedId; }

    ///Setters
    void setLastCheckedId(const int id) { lastCheckedId = id; }

    void unSelectAllRobots(void);

protected:
    void showEvent(QShowEvent *);

signals:
    void deselectRobots();

private:
    RobotBtnGroup* btnGroup;
    CustomScrollArea* scrollArea;
    TopLeftMenu* actionButtons;
    int lastCheckedId;

};

#endif // ROBOTSLEFTWIDGET_H
