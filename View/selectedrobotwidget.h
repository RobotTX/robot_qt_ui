#ifndef SELECTEDROBOTWIDGET_H
#define SELECTEDROBOTWIDGET_H

class PathWidget;
class CustomScrollArea;
class RobotView;
class QVBoxLayout;
class QPushButton;
class QLabel;
class QMainWindow;
class QProgressBar;
class TopLeftMenu;

#include <QWidget>
#include "mainwindow.h"
/**
 * @brief The SelectedRobotWidget class
 * Widget of the left menu which is displayed when a robot is selected
 * (from the map or the list of robots)
 */
class SelectedRobotWidget: public QWidget{
    Q_OBJECT
public:
    SelectedRobotWidget(QMainWindow* parent);

    /**
     * @brief setSelectedRobot
     * @param _robotView
     * Update the widget with the selected robot
     */
    void setSelectedRobot(RobotView* const& _robotView);
    QPushButton* getScanBtn(void) const {return scanBtn;}
   // QPushButton* getHomeBtn(void) const {return homeBtn;}
    void disable();
    void enable();
    TopLeftMenu* getActionButtons(void) const {return actionButtons;}
    QString getName(void) ;
    PathWidget* getPathWidget(void) const {return pathWidget;}
    QLabel* getNoPath(void) const { return noPath; }
    QLabel* getNameLabel(void) const { return name; }
    QLabel* getWifiLabel(void) const { wifiNameLabel; }

private:
    QVBoxLayout* layout;
    RobotView* robotView;
    QProgressBar* batteryLevel;
    QLabel* wifiNameLabel;
    QPushButton* goHome;
    QLabel* ipAddressLabel;
    PathWidget* pathWidget;
    QPushButton* scanBtn;
    QLabel* name;
    TopLeftMenu* actionButtons ;
    QLabel* noPath ;
    QLabel* homeLabel2;

protected:
    void showEvent(QShowEvent *event);
    void hideEvent(QHideEvent *event);

signals:
    void showSelectedRobotWidget();
    void hideSelectedRobotWidget();
};

#endif // SELECTEDROBOTWIDGET_H
