#ifndef DISPLAYSELECTEDPOINT_H
#define DISPLAYSELECTEDPOINT_H

class Point;
class QMainWindow;
class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class Point;
class QEvent;
class QKeyEvent;
class QLabel;
class Map;

#include "Model/points.h"
#include "View/pointview.h"
#include <QLineEdit>
#include <memory>
#include <QWidget>
#include <QObject>
#include "Model/graphicitemstate.h"
#include "topleftmenu.h"
class DisplaySelectedPoint: public QWidget
{
        Q_OBJECT
public:
    /// used to determine which menu or object (could be the map) cause the information of this point to be displayed
    enum Origin { MAP, GROUP_MENU, POINTS_MENU };

    DisplaySelectedPoint(QMainWindow* const _parent, const std::shared_ptr<Points> &_points, std::shared_ptr<Map> const& _map, PointView* _pointView = 0, const Origin _origin = MAP);

    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QPushButton* getSaveButton(void) const { return saveButton; }
    QPushButton* getCancelButton(void) const { return cancelButton; }
    QString getPointName(void) const { return nameEdit->text(); }
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    std::shared_ptr<Point> getPoint(void) const { return pointView->getPoint(); }
    void setPoint(std::shared_ptr<Point> const& _point) { pointView->setPoint(_point); }
    PointView* getPointView(void) const { return pointView; }
    void setPointView(PointView* const& _pointView, QString robotName);
    Origin getOrigin(void) const { return origin; }
    QLabel* getXLabel(void) const { return posXLabel; }
    QLabel* getYLabel(void) const { return posYLabel; }
    QWidget* getHomeWidget(void) const { return homeWidget; }
    QPushButton* getRobotButton(void) const { return robotBtn; }

public:
    void displayPointInfo(void);
    void setOrigin(const Origin _origin);
    void resetWidget(void);

protected:
    void mousePressEvent(QEvent*);
    void keyPressEvent(QKeyEvent* event);
    void hideEvent(QHideEvent *event);

signals:
    /// to notify the mapview that one of its points have been updated (in order to update the name that's displayed when the mouse is hovering over a point)
    void nameChanged(QString, QString);
    /// to reset the state of the map if a user clicks a random button while he was editting a point
    void resetState(GraphicItemState, bool);

private slots:
    /// to check that a name is available before we proceed to the update
    void checkPointName(const QString name) const;

private:
    std::shared_ptr<Map> map;
    QLineEdit* nameEdit;
    QHBoxLayout* nameLayout;
    QVBoxLayout* layout;
    QHBoxLayout* editLayout;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* grid;

    QLabel* posXLabel;
    QLabel* posYLabel;

    QPushButton* plusButton;
    QPushButton* minusButton;
    QPushButton* mapButton;
    QPushButton* eyeButton;
    QPushButton* editButton;
    QPushButton* saveButton;
    QPushButton* cancelButton;
    PointView* pointView;
    QMainWindow* parent;
    std::shared_ptr<Points> points;
    QWidget* homeWidget;
    QPushButton* robotBtn;
    TopLeftMenu* actionButtons;

    /// to determine whether we come from the group menu and have to go back to it if we click on the back button
    /// or if we got here by clicking on the map
    Origin origin;
};

#endif // DISPLAYSELECTEDPOINT_H
