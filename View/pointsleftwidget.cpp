#include "pointsleftwidget.h"
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
#include "View/spacewidget.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"
#include "Controller/mainwindow.h"
#include "View/leftmenu.h"
#include <QKeyEvent>
#include <QDebug>

PointsLeftWidget::PointsLeftWidget(QMainWindow* _parent, std::shared_ptr<Points> const& _points, bool _groupDisplayed)
    : QWidget(_parent), groupDisplayed(_groupDisplayed)
{
    points = _points;
    parent = _parent;

    scrollArea = new VerticalScrollArea(this);

    indexLastGroupClicked = 0;

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->setAllCheckable();

    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getEyeButton()->setEnabled(false);

    actionButtons->getPlusButton()->setToolTip("Click here to add a new group");
    actionButtons->getMinusButton()->setToolTip("Select a group or a point and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a group or a point and click here to modify it");
    actionButtons->getMapButton()->setToolTip("Select a group or a point and click here to display or hide it on the map");
    actionButtons->getEyeButton()->setToolTip("Select a group or a point and click here to display its information");

    layout->addWidget(actionButtons);

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

    connect(actionButtons->getPlusButton(), SIGNAL(clicked(bool)), parent, SLOT(plusGroupBtnEvent()));
    connect(actionButtons->getMinusButton(), SIGNAL(clicked(bool)), parent, SLOT(minusGroupBtnEvent()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked(bool)), parent, SLOT(editGroupBtnEvent()));
    connect(actionButtons->getEyeButton(), SIGNAL(clicked()), parent, SLOT(displayPointsInGroup()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), parent, SLOT(displayGroupMapEvent()));

    /// to handle double clicks
    foreach(QAbstractButton *button, groupButtonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(int)), parent, SLOT(doubleClickOnGroup(int)));

    /// to enable the buttons
    connect(groupButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(int)), this, SLOT(enableButtons(int)));

    /// to make sure the name chosen for a new group is valid
    connect(groupNameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkGroupName(QString)));

    /// to make sure the new name chosen for a group is valid
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(textEdited(QString)), this, SLOT(checkGroupName(QString)));

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
    actionButtons->getMinusButton()->setEnabled(true);
    if(index < points->count()-1)
        actionButtons->getMinusButton()->setToolTip("Click to remove the selected group");
    else
        actionButtons->getMinusButton()->setToolTip("Click to remove the selected point");
    /// enables the eye button
    actionButtons->getEyeButton()->setEnabled(true);
    if(index < points->count()-1)
        actionButtons->getEyeButton()->setToolTip("Click to display the information of the selected group");
    else
        actionButtons->getEyeButton()->setToolTip("Click to display the information of the selected point");
    /// enables the map button
    actionButtons->getMapButton()->setCheckable(true);
    actionButtons->getMapButton()->setEnabled(true);
    if(index < points->count()-1){
        if(points->getGroups().at(index)->isDisplayed()){
            actionButtons->getMapButton()->setChecked(true);
            actionButtons->getMapButton()->setToolTip("Click to hide the selected group on the map");
        } else {
            actionButtons->getMapButton()->setChecked(false);
            actionButtons->getMapButton()->setToolTip("Click to display the selected group on the map");
        }
    } else {
        if(points->getDefaultGroup()->getPoints().at(index-points->count()+1)->isDisplayed()){
            actionButtons->getMapButton()->setChecked(true);
            actionButtons->getMapButton()->setToolTip("Click to hide the selected point on the map");
        } else {
            actionButtons->getMapButton()->setChecked(false);
            actionButtons->getMapButton()->setToolTip("Click to display the selected point on the map");
        }
    }
    /// enables the edit button
    actionButtons->getEditButton()->setEnabled(true);
    if(index < points->count()-1)
        actionButtons->getEditButton()->setToolTip("Click to modify the selected group");
    else
        actionButtons->getEditButton()->setToolTip("click to modify the selected point");

}

void PointsLeftWidget::disableButtons(void){

    /// resets the minus button
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getMinusButton()->setToolTip("Select a group or a point and click here to remove it");

    /// resets the eye button
    actionButtons->getEyeButton()->setEnabled(false);
    actionButtons->getEyeButton()->setToolTip("Select a group or a point and click here to display its information");

    /// resets the map button
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getMapButton()->setToolTip("Select a group or a point and click here to display or hide it on the map");
    /// resets when we go back to the previous menu and come back to this one
    actionButtons->getMapButton()->setCheckable(false);

    /// resets the edit button
    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getEditButton()->setToolTip("Select a group or a point and click here to modify it");
    actionButtons->getEditButton()->setChecked(false);

}

int PointsLeftWidget::checkGroupName(QString name){

    if(!creatingGroup && !name.compare(points->getGroups().at(groupButtonGroup->getIndexModifyEdit())->getName(), Qt::CaseInsensitive)){
        saveButton->setToolTip("");
        qDebug() << "same name";
        return 0;
    }
    if(!name.compare("")){
        saveButton->setToolTip("The name of your group cannot be empty");
        saveButton->setEnabled(false);
        return 1;
    }
    for(int i = 0; i < points->count(); i++){
        if(!name.compare(points->getGroups().at(i)->getName(), Qt::CaseInsensitive)){
            qDebug() << points->getGroups().at(i)->getName();
            saveButton->setToolTip("A group with the same name already exists, please choose another name for your group");
            saveButton->setEnabled(false);
            return 2;
        }
    }
    saveButton->setToolTip("");
    saveButton->setEnabled(true);
    return 0;
}

void PointsLeftWidget::cancelCreationGroup(){
    /// hides everything that's related to the creation of a group
    groupNameEdit->hide();
    groupNameLabel->hide();
    saveButton->hide();
    cancelButton->hide();

    // emit un signal a la main window or to the left menu to do it

    //((LeftMenu*) parentWidget())->getReturnButton()->setEnabled(true);
    /// resets the buttons so we can click them
    groupButtonGroup->setEnabled(true);
}

void PointsLeftWidget::emitNewGroupSignal(){
    qDebug() << "emitnewgrupsignal called" << groupNameEdit->text();
    emit newGroup(groupNameEdit->text());
}

void PointsLeftWidget::keyPressEvent(QKeyEvent* event){
    /// this is the enter key
    if(!event->text().compare("\r")){
        if(creatingGroup)
            emit newGroup(groupNameEdit->text());
        else
            emit modifiedGroup(groupButtonGroup->getModifyEdit()->text());
    }
}

void PointsLeftWidget::hideEvent(QHideEvent *event){
    resetWidget();
    QWidget::hideEvent(event);
}
