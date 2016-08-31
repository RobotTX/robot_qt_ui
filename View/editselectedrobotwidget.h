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

#include "pathpoint.h"
#include <QWidget>
#include <QSharedPointer>
#include "Model/point.h"
#include "View/pointview.h"
#include "mainwindow.h"


/**
 * @brief The EditSelectedRobotWidget class
 * The class which display the menu to edit a robot
 */
class EditSelectedRobotWidget: public QWidget{
    Q_OBJECT
public:
    EditSelectedRobotWidget(QWidget* parent, MainWindow* mainWindow, QSharedPointer<Robots> const robots);

    CustomLineEdit* getNameEdit(void){ return nameEdit; }
    CustomPushButton* getHomeBtn(void){ return homeBtn; }
    QSharedPointer<PointView> getHome() const { return home; }
    bool isFirstConnection() const { return firstConnection; }
    CustomLineEdit* getWifiNameEdit(void) const { return wifiNameEdit; }
    CustomLineEdit* getWifiPwdEdit(void) const { return wifiPwdEdit; }
    PathWidget* getPathWidget(void) const { return pathWidget; }
    CustomPushButton* getAddPathBtn(void) const { return addPathBtn; }
    bool getPathChanged() const { return pathChanged; }
    QSharedPointer<PointView> getOldHome() const { return oldHome; }
    QLabel* getHomeLabel(void) const { return homeLabel; }
    CustomPushButton* getSaveButton(void) const { return saveBtn; }
    CustomPushButton* getCancelButton(void) const { return cancelBtn; }
    bool isEditing(void) const { return editing; }


    void setRobots(QSharedPointer<Robots> const _robots) { robots = _robots; }
    void setEditing(bool const _editing) { editing = _editing; }
    void setHome(QSharedPointer<PointView> const _home) { home = _home; }
    void setOldHome(QSharedPointer<PointView> const _oldHome) { oldHome = _oldHome;}
    void setPathChanged(const bool change) { pathChanged = change; }
    void setPathWidget(PathWidget* pw) { pathWidget = pw; }

    /// updates the name and wifi or the robot
    void editName(void);
    /// disables the buttons and line edits
    void disableAll(void);
    /// enables the buttons and line edits
    void setEnableAll(bool enable);
    void setSelectedRobot(RobotView * const robotView, bool firstConnection = false);
    void setPath(const QVector<QSharedPointer<PathPoint> > &path);
    void clearPath();

signals:
    /// Signal emitted when a robot has been edited & saved
    void robotSaved(void);
    void showEditSelectedRobotWidget(void);
    void hideEditSelectedRobotWidget(void);

private:
    QVBoxLayout* layout;
    QGridLayout* wifiLayout;
    RobotView* robotView;
    QProgressBar* batteryLevel;
    CustomLineEdit* nameEdit;
    QLabel* wifiTitle;
    QLabel* wifiName;
    CustomLineEdit* wifiNameEdit;
    QLabel* wifiPwd;
    CustomLineEdit* wifiPwdEdit;
    QLabel* ipAddressLabel;
    QSharedPointer<Robots> robots;
    CustomPushButton* saveBtn;
    CustomPushButton* homeBtn;
    QSharedPointer<PointView> home;
    QSharedPointer<PointView> oldHome;
    CustomPushButton* addPathBtn;
    bool firstConnection;
    CustomPushButton* cancelBtn;
    CustomPushButton* deletePathBtn;
    PathWidget* pathWidget;
    bool pathChanged;
    QLabel* homeLabel;
    bool editing;

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

    void deletePwd(void);

};

#endif // EDITSELECTEDROBOTWIDGET_H
