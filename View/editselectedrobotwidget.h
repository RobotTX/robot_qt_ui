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
    void setHome(PointView* const _home){home = _home;}
    PointView* getHome() const {return home;}
    bool isFirstConnection()const {return firstConnection;}
    void setOldHome(PointView* const _oldHome){oldHome = _oldHome;}
    PointView* getOldHome() const {return oldHome;}
    QLineEdit* getWifiNameEdit(void) const { return wifiNameEdit; }
    QLineEdit* getWifiPwdEdit(void) const { return wifiPwdEdit; }
    PathWidget* getPathWidget(void) const { return pathWidget; }
    QPushButton* getAddPathBtn(void) const { return addPathBtn;}
    void setPathChanged(const bool change) { pathChanged = change; }
    bool getPathChanged() const { return pathChanged; }
    void setPathWidget(PathWidget* pw) { pathWidget = pw; }
    void setPath(QVector<std::shared_ptr<PathPoint>> const path);
    void clearPath();
    QLabel* getHomeLabel(void) const {return homeLabel;}
    QPushButton* getSaveButton(void) const { return saveBtn; }

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
    PointView* home;
    PointView* oldHome;
    QPushButton* addPathBtn;
    bool firstConnection;
    QPushButton* cancelBtn;
    PathWidget* pathWidget;
    bool pathChanged;
    QLabel* homeLabel;
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
