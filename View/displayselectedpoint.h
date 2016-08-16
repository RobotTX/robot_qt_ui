#ifndef DISPLAYSELECTEDPOINT_H
#define DISPLAYSELECTEDPOINT_H

class Point;
class QMainWindow;
class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class QEvent;
class QKeyEvent;
class QLabel;
class Map;
class Robots;
class DisplaySelectedPointRobots;

#include "View/createpointwidget.h"
#include "Model/points.h"
#include "View/pointview.h"
#include <QLineEdit>
#include <QSharedPointer>
#include <QWidget>
#include <QObject>
#include "Model/graphicitemstate.h"
#include "topleftmenu.h"
#include "Model/point.h"


class DisplaySelectedPoint: public QWidget
{
        Q_OBJECT
public:
    /// used to determine which menu or object (could be the map) cause the information of this point to be displayed
    enum Origin { MAP, GROUP_MENU, POINTS_MENU };

    DisplaySelectedPoint(QMainWindow* const _parent, QSharedPointer<Robots> const robots,const QSharedPointer<Points> &_points, QSharedPointer<Map> const& _map, QSharedPointer<PointView> _point = QSharedPointer<PointView>(), const Origin _origin = MAP);

    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QPushButton* getSaveButton(void) const { return saveButton; }
    QPushButton* getCancelButton(void) const { return cancelButton; }
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    QString getPointName(void) const { return pointView->getPoint()->getName(); }
    void setPointView(QSharedPointer<PointView> _pointView, const QString robotName);
    Origin getOrigin(void) const { return origin; }
    QLabel* getXLabel(void) const { return posXLabel; }
    QLabel* getYLabel(void) const { return posYLabel; }
    QString formatName(const QString name) const;
    QSharedPointer<PointView> getPointView(void) const { return pointView; }
    DisplaySelectedPointRobots* getDisplaySelectedPointRobots(void) { return robotsWidget; }

public:
    void displayPointInfo(void);
    void setOrigin(const Origin _origin);
    void resetWidget(void);
    void setRobotsLabel(void);

protected:
    void mousePressEvent(QEvent*);
    void keyPressEvent(QKeyEvent* event);
    void hideEvent(QHideEvent *event);

signals:
    /// to notify the mapview that one of its points have been updated (in order to update the name that's displayed when the mouse is hovering over a point)
    void nameChanged(QString, QString);
    /// to reset the state of the map if a user clicks a random button while he was editting a point
    void resetState(GraphicItemState, bool);
    void invalidName(QString, CreatePointWidget::Error);
    void setSelectedRobotFromPoint(QString);

private slots:
    /// to check that a name is available before we proceed to the update
    int checkPointName(QString name);

private:
    QSharedPointer<Map> map;
    QLineEdit* nameEdit;
    QVBoxLayout* layout;
    QHBoxLayout* editLayout;

    QLabel* posXLabel;
    QLabel* posYLabel;
    DisplaySelectedPointRobots* robotsWidget;

    QPushButton* saveButton;
    QPushButton* cancelButton;
    QSharedPointer<PointView> pointView;
    QMainWindow* parent;
    QSharedPointer<Points> points;
    TopLeftMenu* actionButtons;
    QSharedPointer<Robots> robots;

    /// to determine whether we come from the group menu and have to go back to it if we click on the back button
    /// or if we got here by clicking on the map
    Origin origin;
};

#endif // DISPLAYSELECTEDPOINT_H
