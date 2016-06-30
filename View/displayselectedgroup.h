#ifndef DISPLAYSELECTEDGROUP_H
#define DISPLAYSELECTEDGROUP_H

class VerticalScrollArea;
class QVBoxLayout;
class QPushButton;
class QHBoxLayout;
class QMainWindow;
class QLabel;
class Points;

#include "Model/points.h"
#include <QWidget>
#include "View/pointbuttongroup.h"
#include <memory>

/**
 * @brief The DisplaySelectedGroup class
 * The purpose of this class is to provide a QWidget that displays the name and the points
 * contained by the group that was just clicked
 */

class DisplaySelectedGroup: public QWidget
{
public:
    DisplaySelectedGroup(QMainWindow *_parent, const std::shared_ptr<Points> &_points);

    PointButtonGroup* getPointButtonGroup(void) const { return pointButtonGroup; }
    QPushButton* getMapButton(void) const { return mapButton; }
    QPushButton* getEyeButton(void) const { return eyeButton; }
    QPushButton* getEditButton(void) const { return editButton; }
    QPushButton* getMinusButton(void) const { return minusButton; }
    QLabel* getNameLabel(void) const { return name; }
    std::shared_ptr<Points> getPoints(void) const { return points; }

    void setName(const QString _name);

public:
    void uncheck(void) { pointButtonGroup->uncheck(); }

private:

    PointButtonGroup* pointButtonGroup;
    /// this is the graphical object that allows a user to scroll the points if they can't fit in the screen
    VerticalScrollArea* scrollArea;

    QVBoxLayout* layout;
   // QPushButton* backButton;
    QPushButton* plusButton;
    QPushButton* minusButton;
    QPushButton* mapButton;
    QPushButton* eyeButton;
    QPushButton* editButton;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* grid;

    QLabel* name;
    std::shared_ptr<Points> points;
};

#endif // DISPLAYSELECTEDGROUP_H
