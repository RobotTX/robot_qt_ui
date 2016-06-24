#include "robotsleftwidget.h"
#include "Model/robots.h"
#include "View/robotbtngroup.h"
#include "View/verticalscrollarea.h"
#include "View/spacewidget.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QMainWindow>

RobotsLeftWidget::RobotsLeftWidget(QMainWindow* _parent){
    parent = _parent;
    layout = new QVBoxLayout();
    scrollLayout = new QVBoxLayout();
    scrollArea = new VerticalScrollArea(this);

    /// Button to go back in the previous menu
    QPushButton* backBtn = new QPushButton(QIcon(":/icons/arrowLeft.png"),"Robots");
    backBtn->setStyleSheet ("text-align: left");
    backBtn->setIconSize(_parent->size()/10);
    layout->addWidget(backBtn);

    /// Buttons to edit or show/hide a robot
    QHBoxLayout* grid = new QHBoxLayout();
    editBtn = new QPushButton(QIcon(":/icons/edit.png"),"");
    checkBtn = new QPushButton(QIcon(":/icons/map.png"),"");

    editBtn->setCheckable(true);
    checkBtn->setCheckable(true);
    editBtn->setIconSize(_parent->size()/10);
    checkBtn->setIconSize(_parent->size()/10);

    grid->addWidget(editBtn);
    grid->addWidget(checkBtn);

    layout->addLayout(grid);

    connect(backBtn, SIGNAL(clicked()), parent, SLOT(backRobotBtnEvent()));
    connect(editBtn, SIGNAL(clicked()), parent, SLOT(editRobotBtnEvent()));
    connect(checkBtn, SIGNAL(clicked()), parent, SLOT(checkRobotBtnEvent()));

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    layout->addWidget(spaceWidget);

    layout->addWidget(scrollArea);

    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    setLayout(layout);
}

RobotsLeftWidget::~RobotsLeftWidget(){
    delete layout;
    delete robotsLayout;
    delete btnGroup;
    delete btnCheckGroup;
    delete checkBtn;
    delete editBtn;
    delete scrollLayout;
    delete scrollArea;
}


void RobotsLeftWidget::setRobots(std::shared_ptr<Robots> const &_robots){
    robots = _robots;

    /// Clickable buttons group to select/edit a robot
    btnGroup = new RobotBtnGroup(robots->getRobotsVector(), false);

    /// Checkable buttons group to show/hide a robot
    btnCheckGroup = new RobotBtnGroup(robots->getRobotsVector(), true);
    btnGroup->show();

    connect(btnGroup->getBtnGroup(), SIGNAL(buttonClicked(QAbstractButton*)), parent, SLOT(setSelectedRobot(QAbstractButton*)));
    connect(btnCheckGroup->getBtnGroup(), SIGNAL(buttonToggled(QAbstractButton*, bool)), parent, SLOT(setCheckedRobot(QAbstractButton*, bool)));

    scrollLayout->addWidget(btnGroup);
    scrollLayout->addWidget(btnCheckGroup);

    QWidget* widget = new QWidget();

    widget->setLayout(scrollLayout);
    scrollArea->setWidget(widget);

    update();
}


void RobotsLeftWidget::updateRobots(std::shared_ptr<Robots> const& _robots){
    scrollLayout->removeWidget(btnGroup);
    delete btnGroup;
    scrollLayout->removeWidget(btnCheckGroup);
    delete btnCheckGroup;
    setRobots(_robots);
}
