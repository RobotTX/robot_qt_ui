#ifndef SELECTEDROBOTWIDGET_H
#define SELECTEDROBOTWIDGET_H

class PathWidget;
class VerticalScrollArea;
class RobotView;
class QVBoxLayout;
class QPushButton;
class QLabel;
class QMainWindow;
class QProgressBar;

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
    QPushButton* getHomeBtn(void) const {return homeBtn;}
    void disable();
    void enable();

private:
    QVBoxLayout* layout;
    RobotView* robotView;
    QProgressBar* batteryLevel;
    QPushButton* backBtn;
    QLabel* wifiNameLabel;
    QPushButton* addPathBtn;
    QPushButton* goHome;
    QLabel* ipAddressLabel;
    QPushButton* homeBtn;
    PathWidget* pathWidget;
    QPushButton* scanBtn;
    QPushButton* editBtn;
protected:
    void showEvent(QShowEvent *event);
    void hideEvent(QHideEvent *event);

signals:
    /**
     * @brief selectHome
     * Signal emitted when the user click on the button of the home point, but there is
     * no home so he can choose one
     */
    void selectHome(RobotView*);
    void showSelectedRobotWidget();
    void hideSelectedRobotWidget();

private slots:
    void homeBtnEvent();
};

#endif // SELECTEDROBOTWIDGET_H
