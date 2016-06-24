#ifndef LEFTMENU_H
#define LEFTMENU_H

class Points;
class Robots;
class PointsView;
class LeftMenuWidget;
class PointsLeftWidget;
class SelectedRobotWidget;
class RobotsLeftWidget;
class MapLeftWidget;
class EditSelectedRobotWidget;
class SelectedPointWidget;
class EditSelectedPointWidget;
class DisplaySelectedPoint;
class DisplaySelectedGroup;
class PointsViews;
class PointsViews;
class PathCreationWidget;
class QVBoxLayout;
class QMainWindow;

#include <QWidget>
#include <memory>

/**
 * @brief The LeftMenu class
 * The main menu class that initialize all the other menu
 */
class LeftMenu: public QWidget{
    Q_OBJECT
public:
    LeftMenu(QMainWindow* parent, Points const& points, std::shared_ptr<Robots> const& robots, PointsView* const& pointViews);
    ~LeftMenu();
    QWidget* getLastWidget() const {return lastWidget;}

    /// Getters
    LeftMenuWidget* getLeftMenuWidget(void) const {return leftMenuWidget;}
    PointsLeftWidget* getPointsLeftWidget(void) const {return pointsLeftWidget;}
    SelectedRobotWidget* getSelectedRobotWidget(void) const {return selectedRobotWidget;}
    RobotsLeftWidget* getRobotsLeftWidget(void) const {return robotsLeftWidget;}
    MapLeftWidget* getMapLeftWidget(void) const {return mapLeftWidget;}
    EditSelectedRobotWidget* getEditSelectedRobotWidget(void) const {return editSelectedRobotWidget;}
    SelectedPointWidget* getSelectedPointWidget(void) const {return selectedPointWidget;}
    EditSelectedPointWidget* getEditSelectedPointWidget(void) const {return editSelectedPointWidget;}
    DisplaySelectedPoint* getDisplaySelectedPoint(void) const { return displaySelectedPoint; }
    DisplaySelectedGroup* getDisplaySelectedGroup(void) const { return displaySelectedGroup; }
    PathCreationWidget* getPathCreationWidget(void) const { return pathCreationWidget; }

public:
    void updateGroupDisplayed(const Points& _points, const int groupIndex);

private:
    QVBoxLayout* leftLayout;
    QWidget* lastWidget;
    LeftMenuWidget* leftMenuWidget;
    PointsLeftWidget* pointsLeftWidget;
    SelectedRobotWidget* selectedRobotWidget;
    RobotsLeftWidget* robotsLeftWidget;
    MapLeftWidget* mapLeftWidget;
    EditSelectedRobotWidget* editSelectedRobotWidget;
    SelectedPointWidget* selectedPointWidget;
    EditSelectedPointWidget* editSelectedPointWidget;
    DisplaySelectedPoint* displaySelectedPoint;
    DisplaySelectedGroup* displaySelectedGroup;
    PathCreationWidget* pathCreationWidget;

};

#endif // LEFTMENU_H
