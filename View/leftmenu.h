

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
class QVBoxLayout;
class MainWindow;
class Map;
class PathPainter;

#include <QWidget>
#include <QSharedPointer>
#include <QPushButton>
/**
 * @brief The LeftMenu class
 * The main menu class that initialize all the other menu
 */
class LeftMenu: public QWidget{
    Q_OBJECT
public:
    LeftMenu(MainWindow* _parent, const QSharedPointer<Points> &_points, QSharedPointer<Robots> const& robots, QSharedPointer<Points> const &pointViews, QSharedPointer<Map> const& _map, const PathPainter* pathPainter);

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
    QPushButton* getReturnButton(void) const { return returnButton; }
    QPushButton* getCloseButton(void) const { return closeBtn; }
    QWidget* getLastWidget() const {return lastWidget; }

    void showBackButton(QString name);
    void hideBackButton();

    void updateGroupDisplayed(const QSharedPointer<Points> &_points, const QString groupIndex);
    void disableButtons();
    void setEnableReturnCloseButtons(bool enable);

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
    QSharedPointer<Points> points;

    QString lastCheckedId;

};

#endif // LEFTMENU_H

