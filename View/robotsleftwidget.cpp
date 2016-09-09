#include "robotsleftwidget.h"
#include "Model/robots.h"
#include "View/robotbtngroup.h"
#include "View/customscrollarea.h"
#include "View/spacewidget.h"
#include <QVBoxLayout>
#include "Controller/mainwindow.h"
#include "topleftmenu.h"
#include <QDebug>
#include "View/custompushbutton.h"

RobotsLeftWidget::RobotsLeftWidget(QWidget* parent, MainWindow* _mainWindow, QSharedPointer<Robots> const &_robots):QWidget(parent), mainWindow(_mainWindow){

    layout = new QVBoxLayout(this);
    scrollArea = new CustomScrollArea(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->disableAll();

    //connect(actionButtons->getGoButton(), SIGNAL(clicked()), mainWindow, SLOT(selectViewRobot()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked()), mainWindow, SLOT(editRobotBtnEvent()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), mainWindow, SLOT(checkRobotBtnEventMenu()));

    actionButtons->getMapButton()->setCheckable(true);

    layout->addWidget(actionButtons);

    setRobots(_robots);

    layout->addWidget(scrollArea);
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0,0,0,0);
}

QString RobotsLeftWidget::getSelectedRobotName(){
    qDebug() << "check for name";

    return btnGroup->getBtnGroup()->checkedButton()->text();
}

void RobotsLeftWidget::setRobots(QSharedPointer<Robots> const &_robots){
    robots = _robots;

    /// Clickable buttons group to select/edit a robot
    btnGroup = new RobotBtnGroup(robots->getRobotsVector(), mainWindow);
    //btnGroup->setMaximumWidth(width());

    /// Checkable buttons group to show/hide a robot
    btnGroup->show();

    connect(btnGroup->getBtnGroup(), SIGNAL(buttonClicked(QAbstractButton*)), mainWindow, SLOT(setSelectedRobot(QAbstractButton*)));

    scrollArea->setWidget(btnGroup);
}

void RobotsLeftWidget::updateRobots(QSharedPointer<Robots> const& _robots){
    scrollArea->takeWidget();
    delete btnGroup;
    setRobots(_robots);
}

void RobotsLeftWidget::unSelectAllRobots(){
    btnGroup->getBtnGroup()->setExclusive(false);

    for (int i=0;i<  btnGroup->getBtnGroup()->buttons().size() ;i++){
            btnGroup->getBtnGroup()->buttons().at(i)->setChecked(false);
    }
    btnGroup->getBtnGroup()->setExclusive(true);
}

void RobotsLeftWidget::showEvent(QShowEvent *){
    actionButtons->disableAll();
    actionButtons->uncheckAll();
    unSelectAllRobots();
}
