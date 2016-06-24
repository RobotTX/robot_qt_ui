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

#include "Model/points.h"
#include "View/pointview.h"
#include <QLineEdit>
#include <memory>
#include <QWidget>
#include <QObject>
#include "Model/graphicitemstate.h"

class DisplaySelectedPoint: public QWidget
{
        Q_OBJECT
public:
    /// used to determine which menu or object (could be the map) cause the information of this point to be displayed
    enum Origin { MAP, GROUP_MENU, POINTS_MENU };

    DisplaySelectedPoint(QMainWindow* _parent, Points const& _points, PointView* _pointView = 0, const Origin _origin = MAP);

    QPushButton* getBackButton(void) const { return backButton; }
    QPushButton* getMinusButton(void) const { return minusButton; }
    QPushButton* getMapButton(void) const { return mapButton; }
    QPushButton* getEditButton(void) const { return editButton; }
    QPushButton* getSaveButton(void) const { return saveButton; }
    QPushButton* getCancelButton(void) const { return cancelButton; }
    QString getPointName(void) const { return nameEdit->text(); }
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    std::shared_ptr<Point> getPoint(void) const { return pointView->getPoint(); }
    void setPoint(std::shared_ptr<Point> const& _point) { pointView->setPoint(_point); }
    PointView* getPointView(void) const { return pointView; }
    void setPointView(PointView* const& _pointView) { pointView = _pointView; }
    Origin getOrigin(void) const { return origin; }
    QLabel* getXLabel(void) const { return posXLabel; }
    QLabel* getYLabel(void) const { return posYLabel; }

public:
    void displayPointInfo(void);
    void mousePressEvent(QEvent* event);
    void keyPressEvent(QKeyEvent* event);
    void hideEvent(QHideEvent *event);
    void setOrigin(const Origin _origin);
    void resetWidget(void);

signals:
    /// to notify the mapview that one of its points have been updated (in order to update the name that's displayed when the mouse is hovering over a point)
    void nameChanged(QString, QString);
    /// to reset the state of the map if a user clicks a random button while he was editting a point
    void resetState(GraphicItemState, bool);


private slots:
    /// to check that a name is available before we proceed to the update
    void checkPointName() const;

private:
    QLineEdit* nameEdit;
    QHBoxLayout* nameLayout;
    QVBoxLayout* layout;
    QHBoxLayout* editLayout;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* grid;

    QLabel* posXLabel;
    QLabel* posYLabel;

    QPushButton* backButton;
    QPushButton* plusButton;
    QPushButton* minusButton;
    QPushButton* mapButton;
    QPushButton* eyeButton;
    QPushButton* editButton;
    QPushButton* saveButton;
    QPushButton* cancelButton;
    PointView* pointView;
    QMainWindow* parent;
    Points points;

    /// to determine whether we come from the group menu and have to go back to it if we click on the back button
    /// or if we got here by clicking on the map
    Origin origin;
};

#endif // DISPLAYSELECTEDPOINT_H
