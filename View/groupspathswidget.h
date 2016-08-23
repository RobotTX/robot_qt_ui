#ifndef GROUPSPATHSWIDGET_H
#define GROUPSPATHSWIDGET_H

#include <QWidget>
#include "Model/points.h"

class TopLeftMenu;
class MainWindow;
class QLabel;
class QVBoxLayout;

class GroupsPathsWidget: public QWidget
{
    Q_OBJECT
public:
    GroupsPathsWidget(MainWindow* _parent, const QSharedPointer<Points> &_points);

private:
    QSharedPointer<Points> points;
    QVBoxLayout* layout;

    TopLeftMenu* actionButtons;
};

#endif // GROUPSPATHSWIDGET_H
