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
#include "Model/group.h"
#include "Model/point.h"
#include "Controller/mainwindow.h"
#include "View/leftmenu.h"


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

    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"", this);
    plusButton->setIconSize(_parent->size()/10);
    plusButton->setCheckable(true);
    plusButton->setToolTip("Click here to add a new group");

    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"", this);
    minusButton->setIconSize(_parent->size()/10);
    minusButton->setCheckable(true);
    /// to force the user to choose a group or point first
    minusButton->setEnabled(false);
    minusButton->setToolTip("Select a group or a point and click here to remove it");

    editButton = new QPushButton(QIcon(":/icons/edit.png"),"", this);
    editButton->setIconSize(_parent->size()/10);
    editButton->setCheckable(true);
    /// to force the user to choose a group or point first
    editButton->setEnabled(false);
    editButton->setToolTip("Select a group or a point and click here to modify it");

    grid = new QHBoxLayout();
    grid->addWidget(plusButton);
    grid->addWidget(minusButton);
    grid->addWidget(editButton);

    mapButton = new QPushButton(QIcon(":/icons/map.png"),"", this);
    mapButton->setIconSize(_parent->size()/10);
    /// to force the user to choose first
    mapButton->setEnabled(false);
    mapButton->setCheckable(true);
    mapButton->setToolTip("Select a group or a point and click here to display or hide it on the map");

    eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "", this);
    eyeButton->setIconSize(_parent->size()/10);
    eyeButton->setCheckable(true);
    /// to force the user to choose first
    eyeButton->setEnabled(false);
    eyeButton->setToolTip("Select a group or a point and click here to display its information");

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

    creationLayout = new QHBoxLayout();
    saveButton = new QPushButton("Save", this);
    saveButton->hide();
    saveButton->setEnabled(false);
    cancelButton = new QPushButton("Cancel", this);
    cancelButton->hide();
    creationLayout->addWidget(cancelButton);
    creationLayout->addWidget(saveButton);

    layout->addLayout(creationLayout);

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

    /// to make sure the name chosen for a new group is valid
    connect(groupNameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkGroupName(QString)));

    connect(cancelButton, SIGNAL(clicked(bool)), this, SLOT(cancelCreationGroup()));

    connect(saveButton, SIGNAL(clicked(bool)), this, SLOT(emitNewGroupSignal()));

    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignBottom);
}

void PointsLeftWidget::updateGroupButtonGroup(Points const& points){
    groupButtonGroup->update(points);
}

void PointsLeftWidget::enableButtons(int index){
    disableButtons();
    /// enables the minus button
    minusButton->setEnabled(true);
    if(index < points->count()-1)
        minusButton->setToolTip("Click to remove the selected group");
    else
        minusButton->setToolTip("Click to remove the selected point");
    /// enables the eye button
    eyeButton->setEnabled(true);
    if(index < points->count()-1)
        eyeButton->setToolTip("Click to display the information of the selected group");
    else
        eyeButton->setToolTip("Click to display the information of the selected point");
    /// enables the map button
    mapButton->setCheckable(true);
    mapButton->setEnabled(true);
    if(index < points->count()-1){
        if(points->getGroups().at(index)->isDisplayed()){
            mapButton->setChecked(true);
            mapButton->setToolTip("Click to hide the selected group on the map");
        } else {
            mapButton->setChecked(false);
            mapButton->setToolTip("Click to display the selected group on the map");
        }
    } else {
        if(points->getDefaultGroup()->getPoints().at(index-points->count()+1)->isDisplayed()){
            mapButton->setChecked(true);
            mapButton->setToolTip("Click to hide the selected point on the map");
        } else {
            mapButton->setChecked(false);
            mapButton->setToolTip("Click to display the selected point on the map");
        }
    }
    /// enables the edit button
    editButton->setEnabled(true);
    if(index < points->count()-1)
        editButton->setToolTip("Click to modify the selected group");
    else
        editButton->setToolTip("click to modify the selected point");
}

void PointsLeftWidget::disableButtons(void){
    /// resets the minus button
    minusButton->setEnabled(false);
    minusButton->setToolTip("Select a group or a point and click here to remove it");

    /// resets the eye button
    eyeButton->setEnabled(false);
    eyeButton->setToolTip("Select a group or a point and click here to display its information");

    /// resets the map button
    mapButton->setEnabled(false);
    mapButton->setToolTip("Select a group or a point and click here to display or hide it on the map");
    /// resets when we go back to the previous menu and come back to this one
    mapButton->setCheckable(false);

    /// resets the edit button
    editButton->setEnabled(false);
    editButton->setToolTip("Select a group or a point and click here to modify it");
    editButton->setChecked(false);
}

void PointsLeftWidget::checkGroupName(QString name){
    qDebug() << name;
    if(!name.compare("")){
        saveButton->setToolTip("The name of your group cannot be empty");
        saveButton->setEnabled(false);
        return;
    }
    for(int i = 0; i < points->count(); i++){
        if(!name.compare(points->getGroups().at(i)->getName())){
            saveButton->setToolTip("A group with the same name already exists, please choose another name for your group");
            saveButton->setEnabled(false);
            return;
        }
    }
    saveButton->setToolTip("");
    saveButton->setEnabled(true);
}

void PointsLeftWidget::cancelCreationGroup(){
    /// hides everything that's related to the creation of a group
    groupNameEdit->hide();
    groupNameLabel->hide();
    saveButton->hide();
    cancelButton->hide();

    // emit un signal a la main window to do it

    //((LeftMenu*) parentWidget())->getReturnButton()->setEnabled(true);
    /// resets the buttons so we can click them
    groupButtonGroup->setEnabled(true);
}

void PointsLeftWidget::emitNewGroupSignal(){
    emit newGroup(groupNameEdit->text());
}
