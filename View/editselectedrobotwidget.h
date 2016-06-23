#ifndef EDITSELECTEDROBOTWIDGET_H
#define EDITSELECTEDROBOTWIDGET_H

class Robots;
class RobotView;
class QVBoxLayout;
class QGridLayout;
class QPushButton;
class QLabel;
class QMainWindow;
class QLineEdit;
class QProgressBar;

#include <QWidget>
#include <memory>
/**
 * @brief The EditSelectedRobotWidget class
 * The class which display the menu to edit a robot
 */
class EditSelectedRobotWidget: public QWidget{
    Q_OBJECT
public:
    EditSelectedRobotWidget(QMainWindow* parent, std::shared_ptr<Robots> const robots);
    ~EditSelectedRobotWidget();

    void setSelectedRobot(RobotView * const robotView);
    void setRobots(std::shared_ptr<Robots> const _robots){robots = _robots;}
    void editName(void);
    QLineEdit* getNameEdit(void){ return nameEdit; }
    QLineEdit* getWifiNameEdit(void){ return wifiNameEdit; }
    QLineEdit* getWifiPwdEdit(void){ return wifiPwdEdit; }

signals:
    /// Signal emitted when a robot has been edited & saved
    void robotSaved(void);

private:
    QVBoxLayout* layout;
    QGridLayout* wifiLayout;
    RobotView* robotView;
    QProgressBar* batteryLevel;
    QLineEdit* nameEdit;
    QLabel* wifiTitle;
    QLabel* wifiName;
    QLineEdit* wifiNameEdit;
    QLabel* wifiPwd;
    QLineEdit* wifiPwdEdit;
    QPushButton* addPathBtn;
    QLabel* ipAddressLabel;
    std::shared_ptr<Robots> robots;
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
