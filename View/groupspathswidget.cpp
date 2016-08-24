#include "groupspathswidget.h"
#include "Controller/mainwindow.h"
#include "View/topleftmenu.h"
#include <QDebug>
#include <QLabel>
#include <QVBoxLayout>
#include "Model/point.h"
#include "View/groupspathsbuttongroup.h"
#include "View/pathbuttongroup.h"
#include "View/custompushbutton.h"

GroupsPathsWidget::GroupsPathsWidget(MainWindow* _parent, const QSharedPointer<Points> &_points): QWidget(_parent), points(_points)
{
    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->getPlusButton()->setToolTip("Click to create a new group of paths");
    actionButtons->getMinusButton()->setToolTip("Click to remove a group of paths");
    layout->addWidget(actionButtons);

    paths = QSharedPointer<Paths>(new Paths(_parent));

    paths->createGroup("monday");
    paths->createGroup("tuesday");
    paths->createGroup("monday");
    paths->createPath("monday", "room1");
    paths->createPath("tuesday", "room2");
    paths->createPath("tuesday", "room3");
    paths->createPath("tuesday", "room2");
    paths->createPath("wednesday", "room3");
    Point point("First point", 4.2, 3.1);
    paths->addPathPoint("monday", "room1", QSharedPointer<PathPoint>(new PathPoint(point, PathPoint::Action::HUMAN_ACTION, 2)));
    paths->addPathPoint("wednesday", "room1", QSharedPointer<PathPoint>(new PathPoint(point, PathPoint::Action::HUMAN_ACTION, 2)));
    paths->addPathPoint("monday", "room1", QSharedPointer<PathPoint>(new PathPoint(point, PathPoint::Action::HUMAN_ACTION, 2)));
    paths->addPathPoint("tuesday", "room4", QSharedPointer<PathPoint>(new PathPoint(point, PathPoint::Action::HUMAN_ACTION, 2)));
    paths->addPathPoint("tuesday", "room2", QSharedPointer<PathPoint>(new PathPoint(point, PathPoint::Action::HUMAN_ACTION, 2)));
    paths->displayGroups();

    buttonGroup = new GroupsPathsButtonGroup(_parent, paths);
    layout->addWidget(buttonGroup);

    pathButtonGroup = new PathButtonGroup(this, paths);
/*
    foreach(QAbstractButton *button, buttonGroup->getButtonGroup()->buttons())
        connect(buttonGroup->getButtonGroup(), SIGNAL(doubleClick), )
        */
/*
    /// to handle double clicks
    foreach(QAbstractButton *button, pathButtonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), _parent, SLOT(double(QString)));
        */

    layout->setAlignment(Qt::AlignTop);
    hide();
}
