#include "groupspathswidget.h"
#include "Controller/mainwindow.h"
#include "View/topleftmenu.h"
#include <QDebug>
#include <QLabel>
#include <QVBoxLayout>
#include "Model/point.h"
#include "View/groupspathsbuttongroup.h"
#include "View/pathbuttongroup.h"
#include "View/customizedlineedit.h"
#include <QAbstractButton>
#include "View/customscrollarea.h"
#include <QHBoxLayout>
#include "View/custompushbutton.h"

GroupsPathsWidget::GroupsPathsWidget(MainWindow* _parent, const QSharedPointer<Paths> &_paths): QWidget(_parent), paths(_paths), lastCheckedButton("")
{
    scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    /// initializes the appropriate tooltips
    actionButtons->getEditButton()->setToolTip("Select a group of paths and click here to modify it");
    actionButtons->getMinusButton()->setToolTip("Select a group of paths and click here to delete it");
    actionButtons->getGoButton()->setToolTip("Select a group of paths and click here to display its paths");
    actionButtons->getPlusButton()->setToolTip("Click here to add a new group of paths");

    /// disables buttons until one of the group of paths is clicked
    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getGoButton()->setEnabled(false);

    layout->addWidget(actionButtons);

    QVBoxLayout* downLayout = new QVBoxLayout();

    groupNameLabel = new QLabel("New group's name : ", this);
    groupNameLabel->hide();
    groupNameEdit = new CustomizedLineEdit(this);
    groupNameEdit->hide();

    downLayout->addWidget(groupNameLabel);
    downLayout->addWidget(groupNameEdit);

    /// to modify the names of groups of paths

    modifyEdit = new CustomizedLineEdit(this);
    modifyEdit->setFixedWidth(1.29*modifyEdit->width());
    modifyEdit->hide();

    buttonGroup = new GroupsPathsButtonGroup(_parent, paths);
    scrollArea->setWidget(buttonGroup);
    downLayout->addWidget(scrollArea);

    /// to enable the buttons
    connect(buttonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(enableButtons(QAbstractButton*)));    

    creationLayout = new QHBoxLayout();
    saveButton = new CustomPushButton("Save", this, false, false);
    saveButton->hide();

    cancelButton = new CustomPushButton("Cancel", this);
    cancelButton->hide();
    creationLayout->addWidget(cancelButton);
    creationLayout->addWidget(saveButton);

    downLayout->addLayout(creationLayout);

    downLayout->setAlignment(Qt::AlignBottom);
    layout->setAlignment(Qt::AlignTop);
    layout->addLayout(downLayout);

    /// to make sure the name chosen for a new group is valid
    connect(groupNameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkGroupName(QString)));

    /// to save a group of paths if the name is valid
    connect(saveButton, SIGNAL(clicked()), this, SLOT(newGroupPaths()));

    /// to cancel the creation of a group of paths
    connect(cancelButton, SIGNAL(clicked(bool)), this, SLOT(cancelCreationGroup()));

    hide();
}

void GroupsPathsWidget::enableButtons(QAbstractButton* button){

    if(!button->text().compare(lastCheckedButton)){
        buttonGroup->uncheck();
        lastCheckedButton = "";
        disableButtons();

    } else {
        qDebug() << "GroupsPathsWidget::enableButtons enabling buttons";
        lastCheckedButton = button->text();
        /*
        groupButtonGroup->setEditedGroupName(button);
        groupButtonGroup->getLayout()->removeWidget(groupButtonGroup->getModifyEdit());
        groupButtonGroup->getLayout()->addWidget(groupButtonGroup->getModifyEdit());
        */
        disableButtons();
        /// enables the minus button
        actionButtons->getMinusButton()->setEnabled(true);

        actionButtons->getMinusButton()->setToolTip("Click to remove the selected group");


        /// enables the eye button
        actionButtons->getGoButton()->setEnabled(true);
        actionButtons->getGoButton()->setToolTip("Click to display the paths of this group");

        /// enables the edit button
        actionButtons->getEditButton()->setEnabled(true);
        actionButtons->getEditButton()->setToolTip("Click to modify the selected group");
    }
}

void GroupsPathsWidget::disableButtons(){
    qDebug() << "GroupsPathsWidget disableButtons called";
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getMinusButton()->setToolTip("Select a group of paths and click here to delete it");

    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getGoButton()->setToolTip("Select a group of paths and click here to display its paths");

    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getEditButton()->setToolTip("Select a group of paths and click here to modify it");
}

void GroupsPathsWidget::keyPressEvent(QKeyEvent* event){
    qDebug() << "PointsLeftWidget::keyPressEvent called, creating a group ?" << creatingGroup;
    /// this is the enter key
    if(!event->text().compare("\r")){
        if(creatingGroup)
            emit newPathGroup(groupNameEdit->text());
        else{
            /*
            switch(groupButtonGroup->checkEditGroupName(groupButtonGroup->getModifyEdit()->text())){
            case 0:
                emit modifiedGroup(groupButtonGroup->getModifyEdit()->text());
                setLastCheckedId("");
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
            */
        }
    }
}

int GroupsPathsWidget::checkGroupName(QString name){
    qDebug() << "GroupsPathsWidget::checkGroupName checking name" << name;
    groupNameEdit->setText(formatName(name));
    name = name.simplified();/*
    if(!creatingGroup && !name.compare(groupButtonGroup->getEditedGroupName(), Qt::CaseInsensitive)){
        saveButton->setToolTip("");
        qDebug() << "same name";
        connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));
        return 0;
    }*/
    if(!name.compare("")){
        saveButton->setToolTip("The name of your group cannot be empty");
        saveButton->setEnabled(false);
        //connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));
        emit messageCreationGroup(TEXT_COLOR_WARNING, "The name of your group cannot be empty");
        return 1;
    }
    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> it(*(paths->getGroups()));
    while(it.hasNext()){
        it.next();
        if(!name.compare(it.key(), Qt::CaseInsensitive)){
            saveButton->setToolTip("A group with the same name already exists, please choose another name");
            saveButton->setEnabled(false);
            emit messageCreationGroup(TEXT_COLOR_WARNING, "A group with the same name already exists, please choose another name");
            return 2;
        }
    }
    saveButton->setToolTip("");
    saveButton->setEnabled(true);
    //disconnect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));
    emit messageCreationGroup(TEXT_COLOR_INFO, "To save this group press Enter or click the \"Save button\"");
    return 0;
}

QString GroupsPathsWidget::formatName(const QString name) const {
    qDebug() << "PointsLeftWidget::formatName called" << name;

    QString ret("");
    QStringList nameStrList = name.split(" ", QString::SkipEmptyParts);
    for(int i = 0; i < nameStrList.size(); i++){
        if(i > 0)
            ret += " ";
        ret += nameStrList.at(i);
    }
    if(name.size() > 0 && name.at(name.size()-1) == ' ')
        ret += " ";
    return ret;
}

void GroupsPathsWidget::updateGroupsPaths(void){
    qDebug() << "GroupsPathsWidget::updateGroupsPaths called";
    buttonGroup->deleteButtons();
    buttonGroup->createButtons();
}

void GroupsPathsWidget::uncheck(){

    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->getButtonGroup()->setExclusive(false);
    if(buttonGroup->getButtonGroup()->checkedButton())
        buttonGroup->getButtonGroup()->checkedButton()->setChecked(false);
    buttonGroup->getButtonGroup()->setExclusive(true);
}

void GroupsPathsWidget::newGroupPaths(){
    emit newPathGroup(groupNameEdit->text());
}

void GroupsPathsWidget::cancelCreationGroup(){
    qDebug() << "PointsLeftWidget::cancelCreationGroup called";
    setLastCheckedButton("");

    hideCreationWidgets();

    enableActionButtons();

    /// emits un signal to the left menu to enable the return button
    //emit enableReturn();
    emit messageCreationGroup(TEXT_COLOR_NORMAL, "");
    /// resets the buttons so we can click them
    buttonGroup->setEnabled(true);
}

void GroupsPathsWidget::enableActionButtons(){
    actionButtons->getEditButton()->setEnabled(true);
    actionButtons->getGoButton()->setEnabled(true);
    actionButtons->getMinusButton()->setEnabled(true);
    actionButtons->getPlusButton()->setEnabled(true);
}

void GroupsPathsWidget::hideEvent(QHideEvent *event){
    actionButtons->getPlusButton()->setEnabled(true);
    hideCreationWidgets();
    setLastCheckedButton("");
    /// resets the buttons so we can click them
    buttonGroup->setEnabled(true);
    QWidget::hideEvent(event);
}

void GroupsPathsWidget::hideCreationWidgets(){
    /// hides everything that's related to the creation of a group
    groupNameEdit->hide();
    groupNameLabel->hide();
    saveButton->hide();
    cancelButton->hide();
}

void GroupsPathsWidget::resetWidget(){
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getEditButton()->setEnabled(false);
    setLastCheckedButton("");
}

int GroupsPathsWidget::checkEditGroupName(QString name){
    qDebug() << "GroupButtonGroup::checkEditGroupName called";
    modifyEdit->setText(formatName(modifyEdit->text()));
    name = modifyEdit->text().simplified();
    if(!name.compare(buttonGroup->getButtonGroup()->checkedButton()->text(), Qt::CaseInsensitive)){
        qDebug() << "same name";
        emit codeEditGroup(0);
        return 0;
    }
    if(!name.compare("")){
        emit codeEditGroup(1);
        return 1;
    }

    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> i(*(paths->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(!name.compare(i.key(), Qt::CaseInsensitive)){
            qDebug() << "GroupButtonGroup::checkEditGroupName" << i.key();
            emit codeEditGroup(2);
            return 2;
        }
    }
    emit codeEditGroup(0);
    return 0;
}
