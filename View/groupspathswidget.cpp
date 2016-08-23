#include "groupspathswidget.h"
#include "Controller/mainwindow.h"
#include "View/topleftmenu.h"
#include <QDebug>
#include <QLabel>
#include <QVBoxLayout>

GroupsPathsWidget::GroupsPathsWidget(MainWindow* _parent, const QSharedPointer<Points> &_points): QWidget(_parent), points(_points)
{
    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->getPlusButton()->setToolTip("Click to create a new group of paths");
    actionButtons->getMinusButton()->setToolTip("Click to remove a group of paths");
    layout->addWidget(actionButtons);


    layout->setAlignment(Qt::AlignTop);
    hide();
}
