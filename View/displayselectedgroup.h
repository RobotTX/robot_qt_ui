#ifndef DISPLAYSELECTEDGROUP_H
#define DISPLAYSELECTEDGROUP_H

class CustomScrollArea;
class QVBoxLayout;
class QHBoxLayout;
class QMainWindow;
class QLabel;
class Points;

#include "Model/points.h"
#include <QWidget>
#include "View/pointbuttongroup.h"
#include <QSharedPointer>
#include "topleftmenu.h"
/**
 * @brief The DisplaySelectedGroup class
 * The purpose of this class is to provide a QWidget that displays the name and the points
 * contained by the group that was just clicked
 */

class DisplaySelectedGroup: public QWidget {
    Q_OBJECT
public:
    DisplaySelectedGroup(QWidget *_parent, const QSharedPointer<Points> &_points);

    PointButtonGroup* getPointButtonGroup(void) const { return pointButtonGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QLabel* getNameLabel(void) const { return name; }
    QSharedPointer<Points> getPoints(void) const { return points; }

    void setName(const QString _name);

    void uncheck(void) { pointButtonGroup->uncheck(); }
    void disableButtons();

private slots:
    void buttonClickedSlot(QAbstractButton*);

protected:
    void showEvent(QShowEvent* event);

signals:
    /// emitted when a button is clicked to reset the colors of the path points
    void resetPathPointViews();

private:

    PointButtonGroup* pointButtonGroup;

    /// this is the graphical object that allows a user to scroll the points if they can't fit in the screen
    CustomScrollArea* scrollArea;

    QVBoxLayout* layout;
    TopLeftMenu* actionButtons;

    QLabel* name;
    QSharedPointer<Points> points;
    QString lastCheckedButton;
};

#endif // DISPLAYSELECTEDGROUP_H
