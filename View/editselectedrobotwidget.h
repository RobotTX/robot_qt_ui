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
class PathWidget;
#include "pathpoint.h"
#include <QWidget>
#include <memory>
#include "Model/point.h"
#include "View/pointview.h"
#include "mainwindow.h"
/**
 * @brief The EditSelectedRobotWidget class
 * The class which display the menu to edit a robot
 */
class EditSelectedRobotWidget: public QWidget{
    Q_OBJECT
public:
    EditSelectedRobotWidget(QMainWindow* const parent, std::shared_ptr<Robots> const robots);

    void setSelectedRobot(RobotView * const robotView, bool firstConnection = false);
    void setRobots(std::shared_ptr<Robots> const _robots){robots = _robots;}
    void editName(void);

    QLineEdit* getNameEdit(void){ return nameEdit; }
    QPushButton* getHomeBtn(void){ return homeBtn; }
    void disableAll(void);
    void enableAll(void);
    void setHome(std::shared_ptr<PointView> const _home){home = _home;}
    std::shared_ptr<PointView> getHome() const {return home;}
    bool isFirstConnection()const {return firstConnection;}
    void setOldHome(std::shared_ptr<PointView> const _oldHome){oldHome = _oldHome;}
    std::shared_ptr<PointView> getOldHome() const {return oldHome;}
    QLineEdit* getWifiNameEdit(void){ return wifiNameEdit; }
    QLineEdit* getWifiPwdEdit(void){ return wifiPwdEdit; }
    PathWidget* getPathWidget(void){ return pathWidget; }
    QPushButton* getAddPathBtn(void){return addPathBtn;}
    void setPathChanged(bool change){pathChanged = change;}
    bool getPathChanged(){return pathChanged ;}
    void setPathWidget(PathWidget* pw){pathWidget = pw;}
    void setOldPath(std::vector<std::shared_ptr<PathPoint>> pw){oldPath = pw;}
    std::vector<std::shared_ptr<PathPoint>> getOldPath( ){return oldPath;}

signals:
    /// Signal emitted when a robot has been edited & saved
    void robotSaved(void);
    void showEditSelectedRobotWidget(void);
    void hideEditSelectedRobotWidget(void);
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
    QLabel* ipAddressLabel;
    std::shared_ptr<Robots> robots;
    QPushButton* saveBtn;
    QPushButton* homeBtn;
    std::shared_ptr<PointView> home;
    std::shared_ptr<PointView> oldHome;
    QPushButton* addPathBtn;
    bool firstConnection;
    QPushButton* cancelBtn;
    PathWidget* pathWidget;
    std::vector<std::shared_ptr<PathPoint>> oldPath;
    bool pathChanged;

protected:
    void showEvent(QShowEvent *event);
    void hideEvent(QHideEvent *event);

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
    void checkWifiName(void);

    void deletePwd(void);

};

#endif // EDITSELECTEDROBOTWIDGET_H
