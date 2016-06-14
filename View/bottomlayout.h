#ifndef BOTTOMLAYOUT_H
#define BOTTOMLAYOUT_H

class Robots;
class RobotView;
class QHBoxLayout;
class QPushButton;
class QLabel;
class QButtonGroup;
class QMainWindow;

#include <QWidget>
#include <QVector>

/**
 * @brief The BottomLayout class
 * The widget displayed on the bottom of the app
 * Showing the path of each robot and allowing the user
 * to play/pause the path of the robot or delete the path
 */

class BottomLayout: public QWidget
{
public:
    BottomLayout(QMainWindow* parent, Robots* const& robots);
    ~BottomLayout();

    /// Getters
    QButtonGroup* getPlayRobotBtnGroup(void) const{return playRobotBtnGroup;}
    QButtonGroup* getStopRobotBtnGroup(void) const{return stopRobotBtnGroup;}
    QVector<QLabel*> getVectorPathLabel(void) const{return vectorPathLabel;}

    /**
     * @brief deletePath
     * @param index
     * Function to delete the path from the robot in the given index
     */
    void deletePath(const int index);
    void updateRobot(const int id, RobotView* const robotView);

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
    QButtonGroup* robotBtnGroup;

    /**
     * @brief vectorPathLabel
     * vector containing the different labels displaying the path of each robot,
     * in attribut of the class, so that we can change/update the labels when a path is
     * changed/delete
     */
    QVector<QLabel*> vectorPathLabel;
};

#endif // BOTTOMLAYOUT_H
