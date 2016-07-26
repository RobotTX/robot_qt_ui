#ifndef ROBOTSLEFTWIDGET_H
#define ROBOTSLEFTWIDGET_H

class Robots;
class CustomScrollArea;
class RobotBtnGroup;
class QVBoxLayout;
class QPushButton;
class QMainWindow;
class TopLeftMenu;

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
    RobotBtnGroup* getBtnGroup(void) const { return btnGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QString getSelectedRobotName(void);


    ///Setters
    void setRobots(std::shared_ptr<Robots> const& robots);

    /**
     * @brief updateRobots
     * @param robots
     * Update the list of robots when needed (e.g. : when a robot's name is edited)
     */
    void updateRobots(std::shared_ptr<Robots> const& robots);
    void unSelectAllRobots();
   // void hideEvent(QHideEvent *event);
    void showEvent(QShowEvent *);


private:
    QMainWindow* parent;
    QVBoxLayout* layout;
    std::shared_ptr<Robots> robots;
    QVBoxLayout* robotsLayout;
    RobotBtnGroup* btnGroup;
    QVBoxLayout* scrollLayout;
    CustomScrollArea* scrollArea;
    TopLeftMenu* actionButtons;

};

#endif // ROBOTSLEFTWIDGET_H
