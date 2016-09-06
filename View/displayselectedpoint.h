#ifndef DISPLAYSELECTEDPOINT_H
#define DISPLAYSELECTEDPOINT_H

class Point;
class QMainWindow;
class QVBoxLayout;
class QHBoxLayout;
class CustomPushButton;
class QEvent;
class QKeyEvent;
class CustomLabel;
class Map;
class Robots;
class DisplaySelectedPointRobots;
class CustomLineEdit;

#include "View/createpointwidget.h"
#include "Model/points.h"
#include "View/pointview.h"
#include <QSharedPointer>
#include <QWidget>
#include <QObject>
#include "Model/graphicitemstate.h"
#include "topleftmenu.h"
#include "Model/point.h"

/**
 * @brief The DisplaySelectedPoint class
 * provides a widget to display the information of a given point as well as the possibility to edit it
 */
class DisplaySelectedPoint: public QWidget
{
        Q_OBJECT
public:

    DisplaySelectedPoint(QWidget* _parent, QSharedPointer<Robots> const robots, const QSharedPointer<Points> &_points, QSharedPointer<Map> const& _map, QSharedPointer<PointView> _point = QSharedPointer<PointView>());

    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    CustomPushButton* getSaveButton(void) const { return saveButton; }
    CustomPushButton* getCancelButton(void) const { return cancelButton; }
    CustomLineEdit* getNameEdit(void) const { return nameEdit; }
    QString getPointName(void) const { return pointView->getPoint()->getName(); }
    void setPointView(QSharedPointer<PointView> _pointView, const QString robotName);
    CustomLabel* getXLabel(void) const { return posXLabel; }
    CustomLabel* getYLabel(void) const { return posYLabel; }
    QSharedPointer<PointView> getPointView(void) const { return pointView; }
    DisplaySelectedPointRobots* getDisplaySelectedPointRobots(void) { return robotsWidget; }

public:
    /**
     * @brief displayPointInfo
     * displays the name and coordinates of the point as well as checks / unchecks the map button
     * to reflect the visibility of the point on the map
     */
    void displayPointInfo(void);

    /**
     * @brief resetWidget
     * hides the cancel and save buttons, resets tooltips and state of the edit button
     * emits a signal to reset the state of the map
     */
    void resetWidget(void);
    /**
     * @brief formatName
     * @param name
     * @return QString
     * formats the name given as a parameter so that duplicated and useless spaces are removed ex: " a     name" -> "a name"
     */
    QString formatName(const QString name) const;

protected:
    void mousePressEvent(QEvent*);
    void keyPressEvent(QKeyEvent* event);
    void hideEvent(QHideEvent *event);
    void resizeEvent(QResizeEvent *event);

signals:
    /// to notify the mapview that one of its points have been updated (in order to update the name that's displayed when the mouse is hovering over a point)
    void nameChanged(QString, QString);
    /// to reset the state of the map if a user clicks a random button while he was editing a point
    void resetState(GraphicItemState);
    /// emitted when the field to type the name of the point is changed to allow or not the user to save its point
    void invalidName(QString, CreatePointWidget::Error);
    /// in the event where this point is a special point for a robot (home or part of a path), allows those pieces of information to
    /// be displayed
    void setSelectedRobotFromPoint(QString);
    /// to remove the point when pressing the delete key
    void removePoint();

private slots:
    /// to check that a name is available before we proceed to the update
    int checkPointName(QString name);

private:
    QSharedPointer<Map> map;
    CustomLineEdit* nameEdit;
    QVBoxLayout* layout;
    QHBoxLayout* editLayout;

    CustomLabel* posXLabel;
    CustomLabel* posYLabel;
    DisplaySelectedPointRobots* robotsWidget;

    CustomPushButton* saveButton;
    CustomPushButton* cancelButton;
    QSharedPointer<PointView> pointView;
    QSharedPointer<Points> points;
    TopLeftMenu* actionButtons;
    QSharedPointer<Robots> robots;
};

#endif // DISPLAYSELECTEDPOINT_H
