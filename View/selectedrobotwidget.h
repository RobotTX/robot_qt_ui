#ifndef SELECTEDROBOTWIDGET_H
#define SELECTEDROBOTWIDGET_H

class PathWidget;
class CustomScrollArea;
class RobotView;
class QVBoxLayout;
class CustomPushButton;
class QMainWindow;
class QProgressBar;
class TopLeftMenu;

#include <QWidget>
#include "mainwindow.h"
#include <QLabel>
#include "View/customlabel.h"

/**
 * @brief The SelectedRobotWidget class
 * Widget of the left menu which is displayed when a robot is selected
 * (from the map or the list of robots)
 */
class SelectedRobotWidget: public QWidget{
    Q_OBJECT
public:
    SelectedRobotWidget(QWidget* parent, MainWindow* mainWindow);

    /**
     * @brief setSelectedRobot
     * @param _robotView
     * Update the widget with the selected robot
     */

    CustomPushButton* getScanBtn(void) const {return scanBtn;}
    QString getName(void) const { return name->text(); }
    PathWidget* getPathWidget(void) const {return pathWidget;}
    QLabel* getNoPath(void) const { return noPath; }
    CustomLabel* getNameLabel(void) const { return name; }
    CustomLabel* getWifiLabel(void) const { return wifiNameLabel; }
    TopLeftMenu* getActionButtons(void) const {return actionButtons;}

    void disable();
    void enable();

    void setSelectedRobot(RobotView* const& _robotView);

private:
    QVBoxLayout* layout;
    RobotView* robotView;
    QProgressBar* batteryLevel;
    CustomLabel* name;
    CustomLabel* wifiNameLabel;
    CustomLabel* ipAddressLabel;
    CustomPushButton* goHome;
    PathWidget* pathWidget;
    CustomPushButton* scanBtn;
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
