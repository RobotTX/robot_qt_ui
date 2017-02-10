#ifndef EDITSELECTEDROBOTWIDGET_H
#define EDITSELECTEDROBOTWIDGET_H

class Robots;
class RobotView;
class QVBoxLayout;
class QGridLayout;
class CustomPushButton;
class QLabel;
class MainWindow;
class CustomLabel;
class PathWidget;
class Points;
class CustomLabel;
class CustomRobotDialog;
class MainWindow;

#include "pathpoint.h"
#include <QWidget>
#include <QSharedPointer>
#include "Model/point.h"
#include "View/pointview.h"
#include "mainwindow.h"
#include "Model/paths.h"
#include <QProgressBar>

/**
 * @brief The EditSelectedRobotWidget class
 * The class which display the menu to edit a robot
 */
class EditSelectedRobotWidget: public QWidget{
    Q_OBJECT
public:
    EditSelectedRobotWidget(MainWindow* mainWindow);

    CustomLabel* getNameLabel(void){ return nameLabel; }
    CustomPushButton* getHomeBtn(void){ return homeBtn; }
    QSharedPointer<PointView> getHome() const { return home; }
    CustomLabel* getWifiNameLabel(void) const { return wifiNameLabel; }
    PathWidget* getPathWidget(void) const { return pathWidget; }
    CustomPushButton* getAddPathBtn(void) const { return addPathBtn; }
    bool getPathChanged() const { return pathChanged; }
    CustomLabel* getHomeLabel(void) const { return homeLabel; }
    CustomPushButton* getSaveButton(void) const { return saveBtn; }
    CustomPushButton* getCancelButton(void) const { return cancelBtn; }
    bool isEditing(void) const { return editing; }
    QString getAssignedPath(void) const { return assignedPath; }
    QString getPathName(void) const { return assignedPath; }
    QString getGroupPathName(void) const { return groupAssignedPath; }
    CustomPushButton* getGoHomeBtn(void) const { return goHomeBtn; }
    CustomPushButton* getDeletePathBtn(void) const { return deletePathBtn; }
    CustomRobotDialog* getRobotInfoDialog(void) const { return robotDialog; }
    QPointer<RobotView> getRobot(void) const { return robotView; }

    void setEditing(bool const _editing) { editing = _editing; }
    void setHome(QSharedPointer<PointView> const _home) { home = _home; }
    void setPathChanged(const bool change) { pathChanged = change; }
    void setPathWidget(PathWidget* pw) { pathWidget = pw; }
    void setAssignedPath(const QString path) { assignedPath = path; }
    void setGroupPath(const QString group) { groupAssignedPath = group; }
    void setSelectedRobot(QPointer<RobotView> const robotView);
    void setBatteryLevel(const int level) { batteryLevel->setValue(level); }

    /// disables the buttons and line edits
    void disableAll(void);
    /// enables the buttons and line edits
    void setEnableAll(const bool enable);
    /// updates the path description of the robot
    void setPath(const QVector<QSharedPointer<PathPoint> > &path);
    /// hides the path description of the robot
    void clearPath();

public slots:
    /// to update the home menu to tick the point corresponding to the home and prevent other robots to have the same home
    void updateHomeMenu();
    /// to update the path menu to tick the path assigned to the current robot
    void updatePathsMenu();
    /// called after the main window has received ackownledgement from the robot (update path)
    void applyNewPath(const QString groupName, const QString pathName);

signals:
    /// Signal emitted when a robot has been edited & saved
    void robotSaved(void);
    /// called by the show event to set the homes properly add the delete buttons if they are needed
    void showEditSelectedRobotWidget(void);
    /// called by the hide event to set the homes pointviews and reset the bottom layout properly
    void hideEditSelectedRobotWidget(void);
    /// to display the path whose groupname and pathname are propagated by the signal
    void showPath(QString, QString);
    /// to stop displaying all paths on the map
    void clearMapOfPaths();
    /// to notify that a new home has been assigned
    void newHome(QString);
    /// the new path of the robot to be sent to it by the main window
    void sendPathSelectedRobot(QString, QString);

protected:
    void showEvent(QShowEvent *event);
    void hideEvent(QHideEvent *event);
    void resizeEvent(QResizeEvent *event);

private slots:
    void openMenu();
    void openHomeMenu();
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
    QVBoxLayout* layout;
    QVBoxLayout* pathLayout;
    QPointer<RobotView> robotView;
    CustomLabel* nameLabel;
    CustomLabel* wifiNameLabel;
    QLabel* ipAddressLabel;
    CustomPushButton* saveBtn;
    CustomPushButton* homeBtn;
    QSharedPointer<PointView> home;
    CustomPushButton* addPathBtn;
    CustomPushButton* cancelBtn;
    CustomPushButton* deletePathBtn;
    PathWidget* pathWidget;
    bool pathChanged;
    CustomLabel* homeLabel;
    bool editing;
    QMenu* pathsMenu;
    QMenu* homeMenu;
    SpaceWidget* pathSpaceWidget;
    CustomPushButton* assignPathButton;
    QString assignedPath;
    QString groupAssignedPath;
    QProgressBar* batteryLevel;
    CustomPushButton* goHomeBtn;
    CustomPushButton* editRobotInfoBtn;
    CustomRobotDialog* robotDialog;
    QWidget* inWidget;
    MainWindow* mainWindow;
};

#endif // EDITSELECTEDROBOTWIDGET_H
