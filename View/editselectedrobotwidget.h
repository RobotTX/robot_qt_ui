#ifndef EDITSELECTEDROBOTWIDGET_H
#define EDITSELECTEDROBOTWIDGET_H

class Robots;
class RobotView;
class QVBoxLayout;
class QGridLayout;
class CustomPushButton;
class QLabel;
class MainWindow;
class CustomLineEdit;
class QProgressBar;
class PathWidget;
class Points;
class CustomLabel;

#include "pathpoint.h"
#include <QWidget>
#include <QSharedPointer>
#include "Model/point.h"
#include "View/pointview.h"
#include "mainwindow.h"
#include "Model/paths.h"


/**
 * @brief The EditSelectedRobotWidget class
 * The class which display the menu to edit a robot
 */
class EditSelectedRobotWidget: public QWidget{
    Q_OBJECT
public:
    EditSelectedRobotWidget(QWidget* parent, MainWindow* mainWindow, const QSharedPointer<Points>& _points, QSharedPointer<Robots> const robots, const QSharedPointer<Paths>& _paths);

    CustomLineEdit* getNameEdit(void){ return nameEdit; }
    CustomPushButton* getHomeBtn(void){ return homeBtn; }
    QSharedPointer<PointView> getHome() const { return home; }
    bool isFirstConnection() const { return firstConnection; }
    CustomLineEdit* getWifiNameEdit(void) const { return wifiNameEdit; }
    CustomLineEdit* getWifiPwdEdit(void) const { return wifiPwdEdit; }
    PathWidget* getPathWidget(void) const { return pathWidget; }
    CustomPushButton* getAddPathBtn(void) const { return addPathBtn; }
    bool getPathChanged() const { return pathChanged; }
    CustomLabel* getHomeLabel(void) const { return homeLabel; }
    CustomPushButton* getSaveButton(void) const { return saveBtn; }
    CustomPushButton* getCancelButton(void) const { return cancelBtn; }
    bool isEditing(void) const { return editing; }
    QString getAssignedPath(void) const { return assignedPath; }
    void setAssignedPath(const QString path) { assignedPath = path; }
    void setGroupPath(const QString group) { groupAssignedPath = group; }
    QString getPathName(void) { return assignedPath; }
    QString getGroupPathName(void) { return groupAssignedPath; }
    CustomPushButton* getDeleteHomeBtn(void) { return deleteHomeBtn; }
    CustomPushButton* getDeletePathBtn(void) { return deletePathBtn; }

    void setRobots(QSharedPointer<Robots> const _robots) { robots = _robots; }
    void setEditing(bool const _editing) { editing = _editing; }
    void setHome(QSharedPointer<PointView> const _home) { home = _home; }
    void setPathChanged(const bool change) { pathChanged = change; }
    void setPathWidget(PathWidget* pw) { pathWidget = pw; }

    /// updates the name and wifi or the robot
    void editName(void);
    /// disables the buttons and line edits
    void disableAll(void);
    /// enables the buttons and line edits
    void setEnableAll(const bool enable);
    void setSelectedRobot(RobotView * const robotView, bool firstConnection = false);
    void setPath(const QVector<QSharedPointer<PathPoint> > &path);
    void clearPath();

public slots:
    void updateHomeMenu();
    void updatePathsMenu();

signals:
    /// Signal emitted when a robot has been edited & saved
    void robotSaved(void);
    void showEditSelectedRobotWidget(void);
    void hideEditSelectedRobotWidget(void);
    void showPath(QString, QString);
    void clearMapOfPaths();
    /// to notify that a new home has been assigned
    void newHome(QString);

protected:
    void showEvent(QShowEvent *event);
    void hideEvent(QHideEvent *event);

private slots:
    /**
     * @brief saveEditSelecRobotBtnEvent
     * Called when the save button if clicked
     */
    void saveEditSelecRobotBtnEvent(void);

    /**
    * @brief checkRobotName
    * Check if the robot name is already taken
    */
    void checkRobotName(void);
    void checkWifiName(void);
    void openMenu();
    void openHomeMenu();
    void deletePwd(void);
    void assignPath(QAction* action);
    void assignHome(QAction* action);

private:
    QVBoxLayout* layout;
    QVBoxLayout* pathLayout;
    RobotView* robotView;
    CustomLineEdit* nameEdit;
    CustomLineEdit* wifiNameEdit;
    CustomLineEdit* wifiPwdEdit;
    QLabel* ipAddressLabel;
    QSharedPointer<Points> points;
    QSharedPointer<Robots> robots;
    QSharedPointer<Paths> paths;
    CustomPushButton* saveBtn;
    CustomPushButton* homeBtn;
    QSharedPointer<PointView> home;
    CustomPushButton* addPathBtn;
    bool firstConnection;
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
    CustomPushButton* scanBtn;
    QProgressBar* batteryLevel;
    CustomPushButton* deleteHomeBtn;

};

#endif // EDITSELECTEDROBOTWIDGET_H
