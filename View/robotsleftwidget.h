#ifndef ROBOTSLEFTWIDGET_H
#define ROBOTSLEFTWIDGET_H

class Robots;
class CustomScrollArea;
class RobotBtnGroup;
class QVBoxLayout;
class MainWindow;
class TopLeftMenu;

#include <QWidget>
#include <QSharedPointer>

/**
 * @brief The RobotsLeftWidget class
 * The left menu which display the list of robots and
 * the edit & map button
 */
class RobotsLeftWidget: public QWidget{
public:
    RobotsLeftWidget(QWidget *parent, MainWindow *mainWindow, QSharedPointer<Robots> const &_robots);

    /// Getters
    RobotBtnGroup* getBtnGroup(void) const { return btnGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QString getSelectedRobotName(void);


    ///Setters
    void setRobots(QSharedPointer<Robots> const& robots);

    /**
     * @brief updateRobots
     * @param robots
     * Update the list of robots when needed (e.g. : when a robot's name is edited)
     */
    void updateRobots(QSharedPointer<Robots> const& robots);
    void unSelectAllRobots();
   // void hideEvent(QHideEvent *event);
    void showEvent(QShowEvent *);


private:
    MainWindow* mainWindow;
    QVBoxLayout* layout;
    QSharedPointer<Robots> robots;
    QVBoxLayout* robotsLayout;
    RobotBtnGroup* btnGroup;
    CustomScrollArea* scrollArea;
    TopLeftMenu* actionButtons;

protected:
    void resizeEvent(QResizeEvent *event);

};

#endif // ROBOTSLEFTWIDGET_H
