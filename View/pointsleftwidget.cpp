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
#include "View/spacewidget.h"
#include "Model/points.h"


PointsLeftWidget::PointsLeftWidget(QMainWindow* _parent, std::shared_ptr<Points> const& _points, bool _groupDisplayed)
    : QWidget(_parent), groupDisplayed(_groupDisplayed)
{
    points = _points;
    parent = _parent;
    scrollArea = new VerticalScrollArea(this);

    groupWindow = new GroupEditWindow(this);
    groupWindow->getEdit()->move(200, 200);
    groupWindow->getLabel()->move(300, 300);
    groupWindow->hide();

    indexLastGroupClicked = 0;

    layout = new QVBoxLayout(this);



    backToGroupsButton = new QPushButton(QIcon(":/icons/arrowLeft.png"),"Groups", this);
    layout->addWidget(backToGroupsButton);
    backToGroupsButton->hide();

    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"", this);
    plusButton->setIconSize(_parent->size()/10);
    plusButton->setCheckable(true);

    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"", this);
    minusButton->setIconSize(_parent->size()/10);
    minusButton->setCheckable(true);
    /// to force the user to choose a group or point first
    minusButton->setEnabled(false);
    minusButton->setToolTip("Select a group or a point and click here to remove it");

    editButton = new QPushButton(QIcon(":/icons/edit.png"),"", this);
    editButton->setIconSize(_parent->size()/10);
    editButton->setCheckable(true);

    grid = new QHBoxLayout();
    grid->addWidget(plusButton);
    grid->addWidget(minusButton);
    grid->addWidget(editButton);


    mapButton = new QPushButton(QIcon(":/icons/map.png"),"", this);
    mapButton->setIconSize(_parent->size()/10);

    eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "", this);
    eyeButton->setIconSize(_parent->size()/10);
    eyeButton->setCheckable(true);

    eyeMapLayout = new QHBoxLayout();
    eyeMapLayout->addWidget(eyeButton);
    eyeMapLayout->addWidget(mapButton);

    layout->addLayout(grid);
    layout->addLayout(eyeMapLayout);


    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    layout->addWidget(spaceWidget);

    groupNameLabel = new QLabel("New group's name : ", this);
    groupNameLabel->hide();
    groupNameEdit = new QLineEdit(this);
    groupNameEdit->hide();

    layout->addWidget(groupNameLabel);
    layout->addWidget(groupNameEdit);

    groupButtonGroup = new GroupButtonGroup(*_points, this);

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
    /// for the back button
    connect(backToGroupsButton, SIGNAL(clicked()), parent, SLOT(backToGroupsButtonEvent()));

    connect(plusButton, SIGNAL(clicked(bool)), parent, SLOT(plusGroupBtnEvent()));
    connect(minusButton, SIGNAL(clicked(bool)), parent, SLOT(minusGroupBtnEvent()));
    connect(editButton, SIGNAL(clicked(bool)), parent, SLOT(editGroupBtnEvent(bool)));
    connect(eyeButton, SIGNAL(clicked()), parent, SLOT(displayPointsInGroup()));
    connect(mapButton, SIGNAL(clicked()), parent, SLOT(displayGroupMapEvent()));

    /// to handle double clicks
    foreach(QAbstractButton *button, groupButtonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(int)), parent, SLOT(doubleClickOnGroup(int)));

    /// to enable the buttons
    connect(groupButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(int)), this, SLOT(enableButtons(int)));

    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignBottom);
}

void PointsLeftWidget::updateGroupButtonGroup(Points const& points){
    groupButtonGroup->update(points);
}

void PointsLeftWidget::enableButtons(int index){
    /// enables the minus button
    minusButton->setEnabled(true);
    qDebug() << points->count();
    if(index < points->count()-1)
        minusButton->setToolTip("Click to remove the selected group");
    else
        minusButton->setToolTip("Click to remove the selected point");
}

void PointsLeftWidget::disableButtons(void){
    /// resets the minus button
    minusButton->setEnabled(false);
    minusButton->setToolTip("Select a group or a point and click here to remove it");
}
