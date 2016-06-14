#include "pointsleftwidget.h"
#include "groupmenu.h"
#include "pointlist.h"
#include "pointbuttongroup.h"
#include "verticalscrollarea.h"
#include "groupbuttongroup.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QHBoxLayout>
#include "groupeditwindow.h"

PointsLeftWidget::PointsLeftWidget(QMainWindow* _parent, Points const& points, bool _groupDisplayed): groupDisplayed(_groupDisplayed)
{
    scrollArea = new VerticalScrollArea();

    groupWindow = new GroupEditWindow(this);
    groupWindow->getEdit()->move(200, 200);
    groupWindow->getLabel()->move(300, 300);
    groupWindow->hide();

    indexLastGroupClicked = 0;
    parent = _parent;

    layout = new QVBoxLayout();

    backButton = new QPushButton(QIcon(":/icons/arrowLeft.png"),"Points");
    backButton->setIconSize(_parent->size()/10);
    backButton->setStyleSheet ("text-align: left");
    layout->addWidget(backButton);

    backToGroupsButton = new QPushButton(QIcon(":/icons/arrowLeft.png"),"Groups");
    layout->addWidget(backToGroupsButton);
    backToGroupsButton->hide();



    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"");
    plusButton->setIconSize(_parent->size()/10);
    plusButton->setCheckable(true);

    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"");
    minusButton->setIconSize(_parent->size()/10);
    minusButton->setCheckable(true);

    editButton = new QPushButton(QIcon(":/icons/edit.png"),"");
    editButton->setIconSize(_parent->size()/10);
    editButton->setCheckable(true);

    grid = new QHBoxLayout();
    grid->addWidget(plusButton);
    grid->addWidget(minusButton);
    grid->addWidget(editButton);


    mapButton = new QPushButton(QIcon(":/icons/map.png"),"");
    mapButton->setCheckable(true);
    mapButton->setIconSize(_parent->size()/10);

    eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "");
    eyeButton->setIconSize(_parent->size()/10);
    eyeButton->setCheckable(true);

    eyeMapLayout = new QHBoxLayout();
    eyeMapLayout->addWidget(eyeButton);
    eyeMapLayout->addWidget(mapButton);

    layout->addLayout(grid);
    layout->addLayout(eyeMapLayout);

    groupNameLabel = new QLabel("New group's name : ");
    groupNameLabel->hide();
    groupNameEdit = new QLineEdit();
    groupNameEdit->hide();

    layout->addWidget(groupNameLabel);
    layout->addWidget(groupNameEdit);

    groupButtonGroup = new GroupButtonGroup(points);
    scrollArea->setWidget(groupButtonGroup);

    layout->addWidget(scrollArea);
    /// for the minus button
    connect(groupButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(int)), parent, SLOT(removeGroupEvent(int)));
    /// for the edit button
    connect(groupButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(int)), parent, SLOT(modifyGroupEvent(int)));
    /// for the map button
    connect(groupButtonGroup->getButtonGroup(), SIGNAL(buttonToggled(int, bool)), parent, SLOT(displayGroupEvent(int, bool)));
    /// for the eye button
    connect(groupButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(int)), parent, SLOT(pointInfoEvent()));

    connect(backButton, SIGNAL(clicked()), parent, SLOT(backGroupBtnEvent()));
    connect(plusButton, SIGNAL(clicked()), parent, SLOT(plusGroupBtnEvent()));
    connect(minusButton, SIGNAL(clicked()), parent, SLOT(minusGroupBtnEvent()));

    //connect(editButton, SIGNAL(clicked()), parent, SLOT(editGroupBtnEvent()));

    connect(editButton, SIGNAL(clicked()), parent, SLOT(editGroupBtnEvent()));

    connect(mapButton, SIGNAL(clicked()), parent, SLOT(displayGroupMapEvent()));
    connect(backToGroupsButton, SIGNAL(clicked()), parent, SLOT(backToGroupsButtonEvent()));
    connect(eyeButton, SIGNAL(clicked()), parent, SLOT(displayPointsInGroup()));

    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignBottom);
    setLayout(layout);
}

PointsLeftWidget::~PointsLeftWidget(){
    delete layout;
    delete parent;
    delete eyeMapLayout;
    delete grid;

    delete backButton;
    delete backToGroupsButton;
    delete minusButton;
    delete mapButton;
    delete plusButton;
    delete editButton;
    delete eyeButton;

    delete groupButtonGroup;

    delete groupNameEdit;
    delete groupNameLabel;

    delete eyeButton;
    delete groupWindow;
    delete scrollArea;
}


