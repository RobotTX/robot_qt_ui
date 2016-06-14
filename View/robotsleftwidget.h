#ifndef ROBOTSLEFTWIDGET_H
#define ROBOTSLEFTWIDGET_H

class Robots;
class VerticalScrollArea;
class RobotBtnGroup;
class QVBoxLayout;
class QPushButton;
class QMainWindow;

#include <QPushButton>
#include <QWidget>

/**
 * @brief The RobotsLeftWidget class
 * The left menu which display the list of robots and
 * the edit & map button
 */
class RobotsLeftWidget: public QWidget{
public:
    RobotsLeftWidget(QMainWindow* parent);
    ~RobotsLeftWidget();

    /// Getters
    bool getEditBtnStatus(void) const { return editBtn->isChecked(); }
    bool getCheckBtnStatus(void) const { return checkBtn->isChecked(); }
    RobotBtnGroup* getBtnGroup(void) const { return btnGroup; }
    RobotBtnGroup* getBtnCheckGroup(void) const { return btnCheckGroup; }

    ///Setters
    void setRobots(Robots* const& robots);
    void setCheckBtnStatus(const bool status) { checkBtn->setChecked(status); }
    void setEditBtnStatus(const bool status) { editBtn->setChecked(status); }

    /**
     * @brief updateRobots
     * @param robots
     * Update the list of robots when needed (e.g. : when a robot's name is edited)
     */
    void updateRobots(Robots* const& robots);


private:
    QMainWindow* parent;
    QVBoxLayout* layout;
    Robots* robots;
    QVBoxLayout* robotsLayout;
    RobotBtnGroup* btnGroup;
    RobotBtnGroup* btnCheckGroup;
    QPushButton* checkBtn;
    QPushButton* editBtn;
    QVBoxLayout* scrollLayout;
    VerticalScrollArea* scrollArea;
};

#endif // ROBOTSLEFTWIDGET_H
