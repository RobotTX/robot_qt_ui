

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
class CreatePointWidget;
class DisplaySelectedPoint;
class DisplaySelectedGroup;
class PointsViews;
class PointsViews;
class PathCreationWidget;
class QVBoxLayout;
class MainWindow;
class Map;

#include <QWidget>
#include <memory>
#include <QPushButton>
/**
 * @brief The LeftMenu class
 * The main menu class that initialize all the other menu
 */
class LeftMenu: public QWidget{
    Q_OBJECT
public:
    LeftMenu(MainWindow* _parent, const std::shared_ptr<Points> &_points, std::shared_ptr<Robots> const& robots, PointsView* const& pointViews, std::shared_ptr<Map> const& _map);

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
    PathCreationWidget* getPathCreationWidget(void) const { return pathCreationWidget; }
    void showBackButton(QString name);
    void hideBackButton();
    QPushButton* getReturnButton(void) const { return returnButton; }
    QPushButton* getCloseButton(void) const { return closeBtn; }
    QWidget* getLastWidget() const {return lastWidget;}

    void updateGroupDisplayed(const std::shared_ptr<Points> &_points, const int groupIndex);
    void disableButtons();
    void setEnableReturnCloseButtons(bool enable);

private slots:
    void enableButtons(int index);
    void removePoint();

private:
    QPushButton* closeBtn;
    QVBoxLayout* leftLayout;
    QWidget* lastWidget;
    LeftMenuWidget* leftMenuWidget;
    PointsLeftWidget* pointsLeftWidget;
    SelectedRobotWidget* selectedRobotWidget;
    RobotsLeftWidget* robotsLeftWidget;
    MapLeftWidget* mapLeftWidget;
    EditSelectedRobotWidget* editSelectedRobotWidget;
    SelectedPointWidget* selectedPointWidget;
    CreatePointWidget* createPointWidget;
    DisplaySelectedPoint* displaySelectedPoint;
    DisplaySelectedGroup* displaySelectedGroup;
    PathCreationWidget* pathCreationWidget;
    QPushButton * returnButton;
    MainWindow* parent;
    std::shared_ptr<Points> points;

    int lastCheckedId;

};

#endif // LEFTMENU_H

