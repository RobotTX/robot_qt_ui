#include "robotsleftwidget.h"
#include "Model/robots.h"
#include "View/customscrollarea.h"
#include "View/spacewidget.h"
#include <QVBoxLayout>
#include "Controller/mainwindow.h"
#include "Controller/robotscontroller.h"
#include "topleftmenu.h"
#include <QDebug>
#include "View/custompushbutton.h"
#include <QHideEvent>

RobotsLeftWidget::RobotsLeftWidget(MainWindow* mainWindow)
    : QWidget(mainWindow), lastCheckedId(-1){
    robots = mainWindow->getRobotsController()->getRobots();

    layout = new QVBoxLayout(this);
    scrollArea = new CustomScrollArea(this, true);

    actionButtons = new TopLeftMenu(this);
    actionButtons->enableAll(false);

    connect(actionButtons->getEditButton(), SIGNAL(clicked()), mainWindow, SLOT(editRobotBtnEvent()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), mainWindow, SLOT(checkRobotBtnEventMenu()));

    actionButtons->getMapButton()->setCheckable(true);

    layout->addWidget(actionButtons);

    setRobots(mainWindow);

    layout->addWidget(scrollArea);
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 10, 0);
}

void RobotsLeftWidget::setRobots(MainWindow* mainWindow){

    /// Clickable buttons group to select/edit a robot
    btnGroup = new RobotBtnGroup(robots->getRobotsVector(), mainWindow, this);

    /// Checkable buttons group to show/hide a robot
    btnGroup->show();

    connect(btnGroup->getBtnGroup(), SIGNAL(buttonClicked(QAbstractButton*)), mainWindow, SLOT(setSelectedRobot(QAbstractButton*)));

    scrollArea->setWidget(btnGroup);
}

void RobotsLeftWidget::updateRobots(MainWindow* mainWindow){
    scrollArea->takeWidget();
    delete btnGroup;
    setRobots(mainWindow);
}

void RobotsLeftWidget::unSelectAllRobots(){
    btnGroup->getBtnGroup()->setExclusive(false);

    for (int i = 0; i < btnGroup->getBtnGroup()->buttons().size() ;i++)
            btnGroup->getBtnGroup()->buttons().at(i)->setChecked(false);

    btnGroup->getBtnGroup()->setExclusive(true);
}

void RobotsLeftWidget::showEvent(QShowEvent *){
    lastCheckedId = -1;
    actionButtons->enableAll(false);
    actionButtons->checkAll(false);
    robots->deselect();
    unSelectAllRobots();
}

