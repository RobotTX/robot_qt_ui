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
#include <memory>

/**
 * @brief The RobotsLeftWidget class
 * The left menu which display the list of robots and
 * the edit & map button
 */
class RobotsLeftWidget: public QWidget{
public:
    RobotsLeftWidget(QMainWindow* parent);

    /// Getters
    bool getEditBtnStatus(void) const { return editBtn->isChecked(); }
    bool getCheckBtnStatus(void) const { return checkBtn->isChecked(); }
    RobotBtnGroup* getBtnGroup(void) const { return btnGroup; }
    RobotBtnGroup* getBtnCheckGroup(void) const { return btnCheckGroup; }

    ///Setters
    void setRobots(std::shared_ptr<Robots> const& robots);
    void setCheckBtnStatus(const bool status) { checkBtn->setChecked(status); }
    void setEditBtnStatus(const bool status) { editBtn->setChecked(status); }

    /**
     * @brief updateRobots
     * @param robots
     * Update the list of robots when needed (e.g. : when a robot's name is edited)
     */
    void updateRobots(std::shared_ptr<Robots> const& robots);


private:
    QMainWindow* parent;
    QVBoxLayout* layout;
    std::shared_ptr<Robots> robots;
    QVBoxLayout* robotsLayout;
    RobotBtnGroup* btnGroup;
    RobotBtnGroup* btnCheckGroup;
    QPushButton* checkBtn;
    QPushButton* editBtn;
    QVBoxLayout* scrollLayout;
    VerticalScrollArea* scrollArea;
};

#endif // ROBOTSLEFTWIDGET_H
