#ifndef EDITSELECTEDROBOTWIDGET_H
#define EDITSELECTEDROBOTWIDGET_H

class Robots;
class RobotView;
class QVBoxLayout;
class QGridLayout;
class CustomPushButton;
class QLabel;
class PathWidget;
class Points;
class CustomLabel;
class CustomRobotDialog;
class MainWindow;
class SpaceWidget;

#include <QWidget>
#include <QSharedPointer>
#include <QProgressBar>
#include "Model/Points/point.h"
#include "Model/Paths/paths.h"
#include "Model/Paths/pathpoint.h"
#include "View/Points/pointview.h"
#include "View/Robots/robotview.h"

/**
 * @brief The EditSelectedRobotWidget class
 * The class which display the menu to edit a robot
 */
class EditSelectedRobotWidget: public QWidget {

    Q_OBJECT

public:
    EditSelectedRobotWidget(MainWindow* mainWindow);

    CustomLabel* getNameLabel(void) const { return nameLabel; }
    CustomPushButton* getHomeBtn(void) const { return homeBtn; }
    QSharedPointer<PointView> getHome() const { return home; }
    CustomLabel* getWifiNameLabel(void) const { return wifiNameLabel; }
    PathWidget* getPathWidget(void) const { return pathWidget; }
    CustomLabel* getHomeLabel(void) const { return homeLabel; }
    CustomPushButton* getGoHomeBtn(void) const { return goHomeBtn; }
    CustomPushButton* getDeletePathBtn(void) const { return deletePathBtn; }
    CustomRobotDialog* getRobotInfoDialog(void) const { return robotDialog; }
    QPointer<RobotView> getRobot(void) const { return robotView; }

    void setHome(const QSharedPointer<PointView> _home) { home = _home; }
    void setPathWidget(PathWidget* pw) { pathWidget = pw; }
    void setSelectedRobot(const QPointer<RobotView> robotView);
    void setBatteryLevel(const int level) { batteryLevel->setValue(level); }

    /// disables the buttons and line edits
    void disableAll(void);
    /// enables the buttons and line edits
    void setEnableAll(const bool enable);
    /// updates the path description of the robot
    void setPath(const QVector<QSharedPointer<PathPoint> > &path);
    /// hides the path description of the robot
    void clearPath(void);
    void updatePathsMenu(const MainWindow *mainWindow);
    void updateHomeMenu(const QSharedPointer<Points::Groups> groups);
    void openHomeMenu(void);
    void openPathsMenu(void);

signals:
    /// Signal emitted when a robot has been edited & saved
    void robotSaved(void);
    /// called by the show event to set the homes properly add the delete buttons if they are needed
    void showEditSelectedRobotWidget(void);
    /// called by the hide event to set the homes pointviews and reset the bottom layout properly
    void hideEditSelectedRobotWidget(void);
    /// to notify that a new home has been assigned
    void newHome(QString);
    /// the new path of the robot to be sent to it by the main window
    void sendPathSelectedRobot(QString, QString);
    /// to update the path menu to tick the path assigned to the current robot
    void updatePathsMenu(bool open);
    /// to update the home menu to tick the point corresponding to the home and prevent other robots to have the same home
    void updateHomeMenu(bool open);

protected:
    void showEvent(QShowEvent *event);
    void hideEvent(QHideEvent *event);
    void resizeEvent(QResizeEvent *event);
    void resetRobotDialog(void);

private slots:
    void updateAndOpenPathsMenu(void);
    void updateAndOpenHomeMenu(void);
    void assignPath(QAction* action);
    void assignHome(QAction* action);
    /**
     * @brief editRobot
     * Opens a page to change the name and/or the wifi information
     */
    void editRobot();
    /**
     * @brief cancelRobotModificationsSlot
     * called when a user cancels the modifications on the edition page
     */
    void cancelRobotModificationsSlot();

private:
    CustomLabel* nameLabel;
    CustomLabel* wifiNameLabel;
    CustomLabel* homeLabel;
    CustomPushButton* homeBtn;
    CustomPushButton* deletePathBtn;
    CustomPushButton* goHomeBtn;
    CustomPushButton* editRobotInfoBtn;
    CustomPushButton* assignPathButton;
    CustomRobotDialog* robotDialog;
    QLabel* ipAddressLabel;
    QMenu* pathsMenu;
    QMenu* homeMenu;
    QPointer<RobotView> robotView;
    QProgressBar* batteryLevel;
    QSharedPointer<PointView> home;
    PathWidget* pathWidget;
    SpaceWidget* pathSpaceWidget;
};

#endif // EDITSELECTEDROBOTWIDGET_H
