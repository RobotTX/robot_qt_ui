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
#include <memory>
#include "Model/point.h"
#include "View/pointview.h"

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
    QPushButton* getHomeBtn(void){ return homeBtn; }
    void disableAll(void);
    void enableAll(void);
    void setHome(PointView* const _home, bool const _temporary){home = _home; temporary = _temporary;}
    PointView* getHome() const {return home;}
    bool isTemporaryHome()const {return temporary;}
    void setOldHome(std::shared_ptr<Point> const _oldHome){oldHome = _oldHome;}
    std::shared_ptr<Point> getOldHome() const {return oldHome;}

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
    std::shared_ptr<Robots> robots;
    QPushButton* saveBtn;
    QPushButton* homeBtn;
    PointView* home;
    bool temporary;
    std::shared_ptr<Point> oldHome;

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
