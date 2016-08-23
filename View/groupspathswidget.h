#ifndef GROUPSPATHSWIDGET_H
#define GROUPSPATHSWIDGET_H

#include <QWidget>
#include "Model/points.h"
#include "Model/paths.h"

class GroupsPathsButtonGroup;
class PathButtonGroup;
class TopLeftMenu;
class MainWindow;
class QLabel;
class QVBoxLayout;

class GroupsPathsWidget: public QWidget
{
    Q_OBJECT
public:
    GroupsPathsWidget(MainWindow* _parent, const QSharedPointer<Points> &_points);

    PathButtonGroup* getPathButtonGroup(void) const { return pathButtonGroup; }

private:
    QSharedPointer<Points> points;
    QSharedPointer<Paths> paths;
    QVBoxLayout* layout;
    GroupsPathsButtonGroup* buttonGroup;
    PathButtonGroup* pathButtonGroup;
    TopLeftMenu* actionButtons;
};

#endif // GROUPSPATHSWIDGET_H
