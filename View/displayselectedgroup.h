#ifndef DISPLAYSELECTEDGROUP_H
#define DISPLAYSELECTEDGROUP_H

class CustomScrollArea;
class QVBoxLayout;
class QPushButton;
class QHBoxLayout;
class QMainWindow;
class QLabel;
class Points;
class DoubleClickableButton;
#include "Model/points.h"
#include <QWidget>
#include "View/pointbuttongroup.h"
#include <memory>
#include "topleftmenu.h"
/**
 * @brief The DisplaySelectedGroup class
 * The purpose of this class is to provide a QWidget that displays the name and the points
 * contained by the group that was just clicked
 */

class DisplaySelectedGroup: public QWidget{
    Q_OBJECT
public:
    DisplaySelectedGroup(QMainWindow *_parent, const std::shared_ptr<Points> &_points);

    PointButtonGroup* getPointButtonGroup(void) const { return pointButtonGroup; }

    TopLeftMenu* getActionButtons(void) const { return actionButtons; }

    QLabel* getNameLabel(void) const { return name; }
    std::shared_ptr<Points> getPoints(void) const { return points; }

    void setName(const QString _name);

    void uncheck(void) { pointButtonGroup->uncheck(); }
    void disableButtons();

private slots:
    void buttonClickedSlot(QAbstractButton*);

protected:
    void showEvent(QShowEvent* event);

signals:
    void resetPathPointViews();

private:

    PointButtonGroup* pointButtonGroup;

    /// this is the graphical object that allows a user to scroll the points if they can't fit in the screen
    CustomScrollArea* scrollArea;

    QVBoxLayout* layout;
    TopLeftMenu* actionButtons;

    QLabel* name;
    std::shared_ptr<Points> points;
    QString lastCheckedButton;
};

#endif // DISPLAYSELECTEDGROUP_H
