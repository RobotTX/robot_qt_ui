#ifndef DISPLAYSELECTEDGROUP_H
#define DISPLAYSELECTEDGROUP_H

class VerticalScrollArea;
class PointButtonGroup;
class QVBoxLayout;
class QPushButton;
class QHBoxLayout;
class QMainWindow;
class QLabel;

#include "Model/points.h"
#include <QWidget>

/**
 * @brief The DisplaySelectedGroup class
 * The purpose of this class is to provide a QWidget that displays the name and the points
 * contained by the group that was just clicked
 */

class DisplaySelectedGroup: public QWidget
{
public:
    DisplaySelectedGroup(QMainWindow *_parent, const Points& _points);
    ~DisplaySelectedGroup();

    PointButtonGroup* getPointButtonGroup(void) const { return pointButtonGroup; }
    QPushButton* getBackButton(void) const { return backButton; }
    QPushButton* getMapButton(void) const { return mapButton; }
    QPushButton* getEyeButton(void) const { return eyeButton; }
    QPushButton* getEditButton(void) const { return editButton; }
    QLabel* getNameLabel(void) const { return name; }

    void setName(const QString _name);

private:

    PointButtonGroup* pointButtonGroup;
    VerticalScrollArea* scrollArea;

    QVBoxLayout* layout;
    QPushButton* backButton;
    QPushButton* plusButton;
    QPushButton* minusButton;
    QPushButton* mapButton;
    QPushButton* eyeButton;
    QPushButton* editButton;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* grid;

    QLabel* name;
};

#endif // DISPLAYSELECTEDGROUP_H
