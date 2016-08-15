#ifndef BOTTOMLAYOUT_H
#define BOTTOMLAYOUT_H

class Robots;
class RobotView;
class QHBoxLayout;
class QPushButton;
class QLabel;
class QButtonGroup;
class QMainWindow;
class QScrollArea;
class QVBoxLayout;
class PathPoint;


#include <memory>
#include <QWidget>
#include <QVector>
#include <QList>
#include <QAbstractButton>

/**
 * @brief The BottomLayout class
 * The widget displayed on the bottom of the app
 * Showing the path of each robot and allowing the user
 * to play/pause the path of the robot or delete the path
 */

class BottomLayout: public QWidget
{
public:
    BottomLayout(QMainWindow* parent, std::shared_ptr<Robots> const& robots);

    /// Getters
    QButtonGroup* getPlayRobotBtnGroup(void) const { return playRobotBtnGroup; }
    QButtonGroup* getStopRobotBtnGroup(void) const { return stopRobotBtnGroup; }
    QButtonGroup* getRobotBtnGroup(void) const { return robotBtnGroup; }
    QButtonGroup* getViewPathRobotBtnGroup(void) const { return viewPathRobotBtnGroup; }
    QButtonGroup* getDeletePathBtnGroup(void) const { return deletePathBtnGroup; }
    QVector<QLabel*> getVectorPathLabel(void) const { return vectorPathLabel; }

    /**
     * @brief deletePath
     * @param index
     * Function to delete the path from the robot in the given index
     */
    void deletePath(const int index);
    void updateRobot(const int id, RobotView* const robotView);
    void setEnable(const bool enable);
    void addRobot(RobotView * const robotView);
    void removeRobot(const int id);
    /// to stop displaying the path of the robot whose id is given as a parameter
    void uncheckViewPathSelectedRobot(const int robotNb = -1);
    void uncheckAll();
    QString pathToStr(QVector<std::shared_ptr<PathPoint>> const path);

private:
    /**
     * @brief layout
     * The main layout of the widget
     */
    QHBoxLayout* layout;

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
     * Group of buttons with to view the path
     */
    QButtonGroup* viewPathRobotBtnGroup;

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
    QVBoxLayout* columnPlay;
    QVBoxLayout* columnViewPath;
    QVBoxLayout* columnStop;
    QVBoxLayout* columnDelete;
    QWidget* widgetName;
    QWidget* widgetPath;
    QWidget* actionWidget;
};

#endif // BOTTOMLAYOUT_H
