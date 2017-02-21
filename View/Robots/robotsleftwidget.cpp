#include "robotsleftwidget.h"
#include "Model/Robots/robots.h"
#include "View/Other/customscrollarea.h"
#include "View/Other/spacewidget.h"
#include <QVBoxLayout>
#include "Controller/mainwindow.h"
#include "Controller/Robots/robotscontroller.h"
#include "View/LeftMenu/topleftmenu.h"
#include <QDebug>
#include "View/Other/custompushbutton.h"
#include <QHideEvent>

RobotsLeftWidget::RobotsLeftWidget(MainWindow* mainWindow)
    : QWidget(mainWindow), lastCheckedId(-1){

    QVBoxLayout* layout = new QVBoxLayout(this);
    scrollArea = new CustomScrollArea(this, true);

    actionButtons = new TopLeftMenu(this);
    actionButtons->enableAll(false);

    connect(actionButtons->getEditButton(), SIGNAL(clicked()), mainWindow, SLOT(editRobotBtnEvent()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), mainWindow, SLOT(checkRobotBtnEventMenu()));

    actionButtons->getMapButton()->setCheckable(true);

    layout->addWidget(actionButtons);

    /// Clickable buttons group to select/edit a robot
    btnGroup = new RobotBtnGroup(this);
    /// Checkable buttons group to show/hide a robot
    btnGroup->show();
    connect(btnGroup->getBtnGroup(), SIGNAL(buttonClicked(QAbstractButton*)), mainWindow, SLOT(setSelectedRobot(QAbstractButton*)));
    connect(btnGroup, SIGNAL(doubleClickOnRobot(QString)), mainWindow, SLOT(doubleClickOnRobotSlot(QString)));

    scrollArea->setWidget(btnGroup);

    layout->addWidget(scrollArea);
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 10, 0);
}

void RobotsLeftWidget::unSelectAllRobots(void){
    btnGroup->getBtnGroup()->setExclusive(false);

    for (int i = 0; i < btnGroup->getBtnGroup()->buttons().size() ;i++)
            btnGroup->getBtnGroup()->buttons().at(i)->setChecked(false);

    btnGroup->getBtnGroup()->setExclusive(true);
}

void RobotsLeftWidget::showEvent(QShowEvent *){
    lastCheckedId = -1;
    actionButtons->enableAll(false);
    actionButtons->checkAll(false);
    emit deselectRobots();
    unSelectAllRobots();
}

