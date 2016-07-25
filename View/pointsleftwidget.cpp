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
#include "View/customizedlineedit.h"
#include "View/toplayout.h"
#include "View/buttonmenu.h"


PointsLeftWidget::PointsLeftWidget(QMainWindow* _parent, std::shared_ptr<Points> const& _points, bool _groupDisplayed)
    : QWidget(_parent), groupDisplayed(_groupDisplayed), points(_points), creatingGroup(true), lastCheckedId(-1)
{
    scrollArea = new VerticalScrollArea(this);

    indexLastGroupClicked = 0;

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->setAllCheckable();

    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getGoButton()->setEnabled(false);

    actionButtons->getPlusButton()->setToolTip("Click here to add a new group");
    actionButtons->getMinusButton()->setToolTip("Select a group or a point and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a group or a point and click here to modify it");
    actionButtons->getMapButton()->setToolTip("Select a group or a point and click here to display or hide it on the map");
    actionButtons->getGoButton()->setToolTip("Select a group or a point and click here to display its information");

    layout->addWidget(actionButtons);

    groupNameLabel = new QLabel("New group's name : ", this);
    groupNameLabel->hide();
    groupNameEdit = new CustomizedLineEdit(this);
    groupNameEdit->hide();

    layout->addWidget(groupNameLabel);
    layout->addWidget(groupNameEdit);

    groupButtonGroup = new GroupButtonGroup(_points, this);

    scrollArea->setWidget(groupButtonGroup);
    layout->addWidget(scrollArea);

    creationLayout = new QHBoxLayout();
    saveButton = new QPushButton("Save", this);
    saveButton->setAutoDefault(true);
    saveButton->hide();
    saveButton->setEnabled(false);
    cancelButton = new QPushButton("Cancel", this);
    cancelButton->setAutoDefault(true);
    cancelButton->hide();
    creationLayout->addWidget(cancelButton);
    creationLayout->addWidget(saveButton);

    layout->addLayout(creationLayout);

    connect(actionButtons->getPlusButton(), SIGNAL(clicked(bool)), _parent, SLOT(plusGroupBtnEvent()));
    connect(actionButtons->getMinusButton(), SIGNAL(clicked(bool)), _parent, SLOT(minusGroupBtnEvent()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked(bool)), _parent, SLOT(editGroupBtnEvent()));
    connect(actionButtons->getGoButton(), SIGNAL(clicked()), _parent, SLOT(displayPointsInGroup()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), _parent, SLOT(displayGroupMapEvent()));

    /// to handle double clicks
    foreach(QAbstractButton *button, groupButtonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(int)), _parent, SLOT(doubleClickOnGroup(int)));

    /// to enable the buttons
    connect(groupButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(int)), this, SLOT(enableButtons(int)));

    /// to make sure the name chosen for a new group is valid
    connect(groupNameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkGroupName(QString)));

    /// to make sure the new name chosen for a group is valid
    //connect(groupButtonGroup->getModifyEdit(), SIGNAL(textEdited(QString)), this, SLOT(checkEditGroupName(QString)));

    connect(cancelButton, SIGNAL(clicked(bool)), this, SLOT(cancelCreationGroup()));

    connect(saveButton, SIGNAL(clicked(bool)), this, SLOT(emitNewGroupSignal()));

    /// to capture the moment a user stops editing whether it is to modify or create a group
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(clickSomewhere(QString)), this, SLOT(modifyGroupAfterClick(QString)));

    connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));  

    connect(this, SIGNAL(enableReturn()), _parent, SLOT(enableReturnAndCloseButtons()));

    /// to reconnect the modifyEdit field in the case where a user creates a new group
    connect(groupButtonGroup, SIGNAL(modifyEditReconnection()), this, SLOT(reconnectModifyEdit()));

    /// relay the signal to the mainWindow so it displays the appropriate messages to the user (e.g, you cannot change the name of the group for this one because it's empty)
    connect(groupButtonGroup, SIGNAL(codeEditGroup(int)), this, SLOT(sendMessageEditGroup(int)));

    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignBottom);
    layout->setContentsMargins(0,0,0,0);
    //setTabOrder(cancelButton, saveButton);

}

void PointsLeftWidget::updateGroupButtonGroup(Points const& points){
    groupButtonGroup->update(points);
}

void PointsLeftWidget::enableButtons(int index){
    qDebug()   << "enable button";
    if(index == lastCheckedId){

        groupButtonGroup->uncheck();
        lastCheckedId = -1;
        disableButtons();
    } else {

        lastCheckedId = index;
        groupButtonGroup->setIndexModifyEdit(index);
        groupButtonGroup->getLayout()->removeWidget(groupButtonGroup->getModifyEdit());
        groupButtonGroup->getLayout()->insertWidget(index, groupButtonGroup->getModifyEdit());
        disableButtons();
        /// enables the minus button
        actionButtons->getMinusButton()->setEnabled(true);
        if(index < points->count()-1)
            actionButtons->getMinusButton()->setToolTip("Click to remove the selected group");
        else
            actionButtons->getMinusButton()->setToolTip("Click to remove the selected point");
        /// enables the eye button
        actionButtons->getGoButton()->setEnabled(true);
        if(index < points->count()-1)
            actionButtons->getGoButton()->setToolTip("Click to display the information of the selected group");
        else
            actionButtons->getGoButton()->setToolTip("Click to display the information of the selected point");
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
}

void PointsLeftWidget::disableButtons(void){

    /// resets the minus button
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getMinusButton()->setToolTip("Select a group or a point and click here to remove it");

    /// resets the eye button
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getGoButton()->setToolTip("Select a group or a point and click here to display its information");

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
    qDebug() << "checking while creating" << name ;
    groupNameEdit->setText(formatName(groupNameEdit->text()));
    name = groupNameEdit->text().simplified();
    if(!creatingGroup && !name.compare(points->getGroups().at(groupButtonGroup->getIndexModifyEdit())->getName(), Qt::CaseInsensitive)){
        saveButton->setToolTip("");
        qDebug() << "same name";
        connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));
        return 0;
    }
    if(!name.compare("")){
        saveButton->setToolTip("The name of your group cannot be empty");
        saveButton->setEnabled(false);
        connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));
        emit messageCreationGroup(TEXT_COLOR_WARNING, "The name of your group cannot be empty");
        return 1;
    }
    for(int i = 0; i < points->count(); i++){
        if(!name.compare(points->getGroups().at(i)->getName(), Qt::CaseInsensitive)){
            qDebug() << points->getGroups().at(i)->getName();
            saveButton->setToolTip("A group with the same name already exists, please choose another name for your group");
            saveButton->setEnabled(false);
            connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));
            emit messageCreationGroup(TEXT_COLOR_WARNING, "A group with the same name already exists, please choose another name for your group");
            return 2;
        }
    }
    saveButton->setToolTip("");
    saveButton->setEnabled(true);
    disconnect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));
    emit messageCreationGroup(TEXT_COLOR_INFO, "To save this group press Enter or click the \"Save button\"");
    return 0;
}

void PointsLeftWidget::cancelCreationGroup(){
    qDebug() << "cancelcreationgroup called";
    setLastCheckedId(-1);
    /// hides everything that's related to the creation of a group
    groupNameEdit->hide();
    groupNameLabel->hide();
    saveButton->hide();
    cancelButton->hide();

    /// emits un signal to the left menu to enable the return button
    emit enableReturn();
    emit messageCreationGroup(TEXT_COLOR_NORMAL, "");
    /// resets the buttons so we can click them
    groupButtonGroup->setEnabled(true);
}

void PointsLeftWidget::emitNewGroupSignal(){
    qDebug() << "emitnewgrupsignal called" << groupNameEdit->text();
    emit newGroup(groupNameEdit->text().simplified());
}

void PointsLeftWidget::keyPressEvent(QKeyEvent* event){
    qDebug() << "pressed enter key";
    /// this is the enter key
    if(!event->text().compare("\r")){
        if(creatingGroup)
            emit newGroup(groupNameEdit->text());
        else{
            switch(groupButtonGroup->checkEditGroupName(groupButtonGroup->getModifyEdit()->text())){
            case 0:
                emit modifiedGroup(groupButtonGroup->getModifyEdit()->text());
                break;
            case 1:
                emit messageCreationGroup(TEXT_COLOR_DANGER, "The name of your group cannot be empty");
                break;
            case 2:
                emit messageCreationGroup(TEXT_COLOR_DANGER, "A group with the same name already exists, please choose another name for your group");
                break;
            default:
                qDebug() << "if you get here you probably forgot to implement the behavior for one or more error codes";
                break;
            }
        }
    }
}

void PointsLeftWidget::showEvent(QShowEvent *event){
    resetWidget();
    QWidget::showEvent(event);
}

void PointsLeftWidget::resetWidget(void){
    groupButtonGroup->uncheck();
    lastCheckedId=-1;
    disableButtons();
}

void PointsLeftWidget::modifyGroupAfterClick(QString name){
    qDebug() << "modify group after click called";
    emit modifiedGroupAfterClick(name);
}

void PointsLeftWidget::reconnectModifyEdit(){
    qDebug() << "reconnectModifyEdit";
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(clickSomewhere(QString)), this, SLOT(modifyGroupAfterClick(QString)));
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(textEdited(QString)), groupButtonGroup, SLOT(checkEditGroupName(QString)));
}

QString PointsLeftWidget::formatName(const QString name) const {

    QString ret("");
    bool containsSpace(false);
    bool containsNonSpace(false);
    for(int i = 0; i < name.length(); i++){
        if(!name.at(i).isSpace() || (!containsSpace && containsNonSpace)){
            if(name.at(i).isSpace())
                containsSpace = true;
            else {
                containsNonSpace = true;
                containsSpace = false;
            }
            ret += name.at(i);
        }
    }
    return ret;
}

void PointsLeftWidget::sendMessageEditGroup(int code){
    qDebug() << "send message edit group called from pointsleftwidget";
    switch(code){
    case 0:
        emit messageCreationGroup(TEXT_COLOR_INFO, "To save this group press Enter or click the \"Save button\"");
        break;
    case 1:
        emit messageCreationGroup(TEXT_COLOR_WARNING, "The name of your group cannot be empty");
        break;
    case 2:
        emit messageCreationGroup(TEXT_COLOR_WARNING, "A group with the same name already exists, please choose another name for your group");
        break;
    default:
        qDebug() << "if you get here you probably forgot to implement the behavior for one or more error codes";
    }
}
