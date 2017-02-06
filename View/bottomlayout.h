#ifndef BOTTOMLAYOUT_H
#define BOTTOMLAYOUT_H

class Robots;
class RobotView;
class QMainWindow;
class PathPoint;
class QButtonGroup;
class QLabel;
class QHBoxLayout;
class QAbstractButton;
class QScrollArea;
class QVBoxLayout;

#include <QSharedPointer>
#include <QWidget>
#include <QVector>
#include <QList>

/**
 * @brief The BottomLayout class
 * The widget displayed at the bottom of the app
 * Showing the path of each robot and allowing the user
 * to play/pause the path of the robot or delete the path
 */

class BottomLayout: public QWidget
{
public:
    BottomLayout(QMainWindow* parent, QSharedPointer<Robots> const& robots);

    QButtonGroup* getPlayRobotBtnGroup(void) const { return playRobotBtnGroup; }
    QButtonGroup* getStopRobotBtnGroup(void) const { return stopRobotBtnGroup; }
    QButtonGroup* getRobotBtnGroup(void) const { return robotBtnGroup; }
    QButtonGroup* getViewPathRobotBtnGroup(void) const { return viewPathRobotBtnGroup; }
    QButtonGroup* getDeletePathBtnGroup(void) const { return deletePathBtnGroup; }
    QVector<QLabel*> getVectorPathLabel(void) const { return vectorPathLabel; }
    int getLastCheckedId(void) const { return lastCheckedId; }

    void setLastCheckedId(const int id) { lastCheckedId = id; }

    /// unchecks the buttons with the robots names
    void uncheckRobots(void);

    /// updates the robot's information of the robot with Id <id> and robotView <robotView>
    /// which consists in updating name and path, disabling some buttons if path is empty
    void updateRobot(const int id, QPointer<RobotView> const robotView);

    /// enables or disables the button
    void setEnable(const bool enable);

    /// adds a robot using its robotview
    void addRobot(QPointer<RobotView> const robotView);

    /// removes the robot with Id <id>
    void removeRobot(const int id);

    /// to stop displaying the path of the robot whose id is given as a parameter
    void uncheckViewPathSelectedRobot(const int robotNb = -1);

    /// uncheck all view path buttons
    void uncheckAll();

    /// to create a QString to display from the path
    QString pathToStr(const QVector<QSharedPointer<PathPoint> > &path, const int stage = 0);

    /// to display at which stage of the path the robot is in the label itself (with a different color)
    void updateStageRobot(const int id, QPointer<RobotView>robotView, const int stage);

private:
    /**
     * @brief layout
     * The main layout of the widget
     */
    QHBoxLayout* layout;

    QButtonGroup* homeBtnGroup;

    /**
     * @brief playRobotBtnGroup
     * Group of the play buttons to play/pause the robot in its path
     * QButtonGroup is not a displayable group, but only allow us
     * to only connect the signal when the user click on a button of the group,
     * instead of connecting each button to a slot
     */
    QButtonGroup* playRobotBtnGroup;

    /**
     * @brief stopRobotBtnGroup
     * Group of the stop buttons to delete a path
     */
    QButtonGroup* stopRobotBtnGroup;

    /**
     * @brief robotBtnGroup
     * Group of buttons with the name of the robots
     */
    QButtonGroup* robotBtnGroup;

    /**
     * @brief pathRobotBtnGroup
     * Group of buttons to display paths
     */
    QButtonGroup* viewPathRobotBtnGroup;

    /**
     * @brief deletePathBtnGroup
     * Group of buttons to delete paths
     */
    QButtonGroup* deletePathBtnGroup;

    /**
     * @brief vectorPathLabel
     * vector containing the different labels displaying the path of each robot,
     * in attribut of the class, so that we can change/update the labels when a path is
     * changed/delete
     */
    QVector<QLabel*> vectorPathLabel;
    QList<QAbstractButton*> listEnabled;
    QScrollArea* pathScroll;
    QVBoxLayout* columnName;
    QVBoxLayout* columnPath;
    QVBoxLayout* columnHome;
    QVBoxLayout* columnPlay;
    QVBoxLayout* columnViewPath;
    QVBoxLayout* columnStop;
    QVBoxLayout* columnDelete;
    QWidget* widgetName;
    QWidget* widgetPath;
    QWidget* actionWidget;

    int lastCheckedId;
};

#endif /// BOTTOMLAYOUT_H
