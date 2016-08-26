

#ifndef LEFTMENU_H
#define LEFTMENU_H

class Points;
class Robots;
class LeftMenuWidget;
class PointsLeftWidget;
class SelectedRobotWidget;
class RobotsLeftWidget;
class MapLeftWidget;
class EditSelectedRobotWidget;
class SelectedPointWidget;
class CreatePointWidget;
class DisplaySelectedPoint;
class DisplaySelectedGroup;
class PathCreationWidget;
class DisplaySelectedPath;
class QVBoxLayout;
class MainWindow;
class Map;
class PathPainter;
class GroupsPathsWidget;
class DisplayPathGroup;
class CustomPushButton;
class Paths;

#include <QWidget>
#include <QSharedPointer>

/**
 * @brief The LeftMenu class
 * The main menu class that initialize all the other menu
 */
class LeftMenu: public QWidget{
    Q_OBJECT
public:
    LeftMenu(MainWindow* _mainWindow, const QSharedPointer<Points> &_points, QSharedPointer<Paths> const& _paths, QSharedPointer<Robots> const& robots, QSharedPointer<Points> const &pointViews, QSharedPointer<Map> const& _map, const PathPainter* pathPainter);

    /// Getters
    LeftMenuWidget* getLeftMenuWidget(void) const {return leftMenuWidget;}
    PointsLeftWidget* getPointsLeftWidget(void) const {return pointsLeftWidget;}
    SelectedRobotWidget* getSelectedRobotWidget(void) const {return selectedRobotWidget;}
    RobotsLeftWidget* getRobotsLeftWidget(void) const {return robotsLeftWidget;}
    MapLeftWidget* getMapLeftWidget(void) const {return mapLeftWidget;}
    EditSelectedRobotWidget* getEditSelectedRobotWidget(void) const {return editSelectedRobotWidget;}
    SelectedPointWidget* getSelectedPointWidget(void) const {return selectedPointWidget;}
    CreatePointWidget* getEditSelectedPointWidget(void) const {return createPointWidget;}
    DisplaySelectedPoint* getDisplaySelectedPoint(void) const { return displaySelectedPoint; }
    DisplaySelectedGroup* getDisplaySelectedGroup(void) const { return displaySelectedGroup; }
    PathCreationWidget* getRobotPathCreationWidget(void) const { return robotPathCreationWidget; }
    CustomPushButton* getReturnButton(void) const { return returnButton; }
    CustomPushButton* getCloseButton(void) const { return closeBtn; }
    QWidget* getLastWidget() const {return lastWidget; }
    DisplaySelectedPath* getDisplaySelectedPath(void) const { return displaySelectedPath; }
    GroupsPathsWidget* getGroupsPathsWidget(void) const { return groupsPathsWidget; }
    DisplayPathGroup* getPathGroupDisplayed(void) const { return pathGroup; }
    PathCreationWidget* getNoRobotPathCreationWidget(void) const { return noRobotPathCreationWidget; }

    void showBackButton(QString name);
    void hideBackButton();

    void updateGroupDisplayed(const QString groupIndex);
    void disableButtons();
    void setEnableReturnCloseButtons(bool enable);

private:
    CustomPushButton* closeBtn;
    QVBoxLayout* leftLayout;
    QWidget* lastWidget;
    LeftMenuWidget* leftMenuWidget;
    PointsLeftWidget* pointsLeftWidget;
    SelectedRobotWidget* selectedRobotWidget;
    RobotsLeftWidget* robotsLeftWidget;
    MapLeftWidget* mapLeftWidget;
    GroupsPathsWidget* groupsPathsWidget;
    EditSelectedRobotWidget* editSelectedRobotWidget;
    SelectedPointWidget* selectedPointWidget;
    CreatePointWidget* createPointWidget;
    DisplaySelectedPoint* displaySelectedPoint;
    DisplaySelectedGroup* displaySelectedGroup;
    PathCreationWidget* robotPathCreationWidget;
    CustomPushButton * returnButton;
    MainWindow* mainWindow;
    QSharedPointer<Points> points;
    QSharedPointer<Paths> paths;
    DisplaySelectedPath* displaySelectedPath;
    DisplayPathGroup* pathGroup;
    QString lastCheckedId;
    PathCreationWidget* noRobotPathCreationWidget;

};

#endif // LEFTMENU_H

