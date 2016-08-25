#include "robotsleftwidget.h"
#include "Model/robots.h"
#include "View/robotbtngroup.h"
#include "View/customscrollarea.h"
#include "View/spacewidget.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include "topleftmenu.h"
#include <QDebug>
#include "View/custompushbutton.h"

RobotsLeftWidget::RobotsLeftWidget(QMainWindow* _parent):QWidget(_parent){
    parent = _parent;
    layout = new QVBoxLayout(this);
    scrollLayout = new QVBoxLayout();
    scrollArea = new CustomScrollArea(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->disableAll();

    connect(actionButtons->getGoButton(), SIGNAL(clicked()), parent, SLOT(selectViewRobot()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked()), parent, SLOT(editRobotBtnEvent()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), parent, SLOT(checkRobotBtnEventMenu()));

    actionButtons->getMapButton()->setCheckable(true);

    layout->addWidget(actionButtons);
    layout->addWidget(scrollArea);

    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    //layout->setContentsMargins(0,0,0,0);

}

QString RobotsLeftWidget::getSelectedRobotName(){
    qDebug() << "check for name";

    return btnGroup->getBtnGroup()->checkedButton()->text();
}

void RobotsLeftWidget::setRobots(QSharedPointer<Robots> const &_robots){
    robots = _robots;

    /// Clickable buttons group to select/edit a robot
    btnGroup = new RobotBtnGroup(robots->getRobotsVector(), parent);

    /// Checkable buttons group to show/hide a robot
    btnGroup->show();

    connect(btnGroup->getBtnGroup(), SIGNAL(buttonClicked(QAbstractButton*)), parent, SLOT(setSelectedRobot(QAbstractButton*)));

    scrollLayout->addWidget(btnGroup);

    QWidget* widget = new QWidget(this);

    widget->setLayout(scrollLayout);
    scrollArea->setWidget(widget);

    update();
}


void RobotsLeftWidget::updateRobots(QSharedPointer<Robots> const& _robots){
    scrollLayout->removeWidget(btnGroup);
    delete btnGroup;

    setRobots(_robots);
}

void RobotsLeftWidget::unSelectAllRobots()
{
    btnGroup->getBtnGroup()->setExclusive(false);

    for (int i=0;i<  btnGroup->getBtnGroup()->buttons().size() ;i++)
    {
            btnGroup->getBtnGroup()->buttons()[i]->setChecked(false);
    }
    btnGroup->getBtnGroup()->setExclusive(true);
}

void RobotsLeftWidget::showEvent(QShowEvent *)
{

actionButtons->disableAll();
actionButtons->uncheckAll();
unSelectAllRobots();
}
