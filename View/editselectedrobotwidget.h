#ifndef EDITSELECTEDROBOTWIDGET_H
#define EDITSELECTEDROBOTWIDGET_H

class Robots;
class RobotView;
class QVBoxLayout;
class QPushButton;
class QLabel;
class QMainWindow;
class QLineEdit;
class QProgressBar;

#include <QWidget>

/**
 * @brief The EditSelectedRobotWidget class
 * The class which display the menu to edit a robot
 */
class EditSelectedRobotWidget: public QWidget{
    Q_OBJECT
public:
    EditSelectedRobotWidget(QMainWindow* parent, Robots * const robots);
    ~EditSelectedRobotWidget();

    void setSelectedRobot(RobotView * const robotView);
    void setRobots(Robots* const _robots){robots = _robots;}
    void editName(void);
    QLineEdit* getNameEdit(void){ return nameEdit; }

signals:
    /// Signal emitted when a robot has been edited & saved
    void robotSaved(void);

private:
    QVBoxLayout* layout;
    RobotView* robotView;
    QProgressBar* batteryLevel;
    QLineEdit* nameEdit;
    QLabel* wifiNameLabel;
    QPushButton* addPathBtn;
    QLabel* ipAddressLabel;
    Robots* robots;
    QPushButton* saveBtn;

private slots:
    /**
     * @brief saveEditSelecRobotBtnEvent
     * Called when the save button if clicked
     */
    void saveEditSelecRobotBtnEvent(void);

    /**
    * @brief checkRobotName
    * Check if the robot name is already taken
    */
    void checkRobotName(void);
};

#endif // EDITSELECTEDROBOTWIDGET_H
