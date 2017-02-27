#include "groupspathswidget.h"
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QAbstractButton>
#include "Helper/helper.h"
#include "Controller/mainwindow.h"
#include "Controller/Paths/pathscontroller.h"
#include "Model/Points/point.h"
#include "View/LeftMenu/topleftmenu.h"
#include "View/Other/customlabel.h"
#include "View/Paths/groupspathsbuttongroup.h"
#include "View/Paths/pathbuttongroup.h"
#include "View/Other/customlineedit.h"
#include "View/Other/customscrollarea.h"
#include "View/Other/custompushbutton.h"
#include "View/Other/stylesettings.h"


GroupsPathsWidget::GroupsPathsWidget(PathsController* pathsController): QWidget(), lastCheckedButton("")
{
    /// to scroll the QButtonGroup if there is a lot of groups of paths
    CustomScrollArea* scrollArea = new CustomScrollArea(this, true);

    layout = new QVBoxLayout(this);
    QVBoxLayout* topLayout = new QVBoxLayout();

    actionButtons = new TopLeftMenu(this);

    initializeActionButtons();

    topLayout->addWidget(actionButtons);

    groupNameLabel = new CustomLabel("New group's name : ", this, false);
    groupNameLabel->hide();
    groupNameEdit = new CustomLineEdit(this);
    groupNameEdit->hide();

    topLayout->addWidget(groupNameLabel);
    topLayout->addWidget(groupNameEdit);
    layout->addLayout(topLayout);

    buttonGroup = new GroupsPathsButtonGroup(this);
    connect(buttonGroup, SIGNAL(updateConnectionsRequest()), pathsController, SLOT(updateConnectionsRequestSlot()));
    /// to enable the action buttons when a group is selected
    connect(buttonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(enableButtons(QAbstractButton*)));
    scrollArea->setWidget(buttonGroup);
    layout->addWidget(scrollArea);

    /// this layout contains the widgets necessary to the creation of a group
    QHBoxLayout* creationLayout = new QHBoxLayout();
    saveButton = new CustomPushButton("Save", this, CustomPushButton::ButtonType::LEFT_MENU, "center", false, false);
    saveButton->hide();

    cancelButton = new CustomPushButton("Cancel", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelButton->hide();
    creationLayout->addWidget(cancelButton);
    creationLayout->addWidget(saveButton);

    layout->addLayout(creationLayout);
    layout->setAlignment(Qt::AlignTop);
    creationLayout->setAlignment(Qt::AlignBottom);

    /// to make sure the name chosen for a new group is valid
    connect(groupNameEdit, SIGNAL(textEdited(QString)), pathsController, SLOT(checkCreateGroupName(QString)));

    /// to notify the user whether the name he chose is valid or not
    connect(buttonGroup->getModifyEdit(), SIGNAL(textEdited(QString)), pathsController, SLOT(checkEditGroupName(QString)));

    /// to save a group of paths if the name is valid
    connect(saveButton, SIGNAL(clicked()), this, SLOT(newGroupPaths()));

    /// to cancel the creation of a group of paths
    connect(cancelButton, SIGNAL(clicked(bool)), this, SLOT(cancelCreationGroup()));

    connect(this, SIGNAL(codeEditGroup(int)), pathsController, SLOT(setMessageModifGroupPaths(int)));
    connect(this, SIGNAL(updatePathGroupButtons()), pathsController, SLOT(updateGroupsPaths()));

    hide();
    creationLayout->setContentsMargins(0, 0, 10, 0);
    topLayout->setContentsMargins(0, 0, 10, 0);
    layout->setContentsMargins(0, 0, 0, 0);
}

void GroupsPathsWidget::enableButtons(QAbstractButton* button){

    /// if the button clicked was already checked we deselect it and disable the action buttons accordingly
    /// we also reset the last checked button to "" (no button)
    if(!button->text().compare(lastCheckedButton)){
        buttonGroup->uncheck();
        lastCheckedButton = "";
        disableButtons();

    } else {
    /// if the button was not selected before we enable the appropriate action buttons and update the tooltips
    /// we also update the last checked button to the button just clicked
        lastCheckedButton = button->text();

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

/// disables the buttons and resets the tooltips
void GroupsPathsWidget::disableButtons(){
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getMinusButton()->setToolTip("Select a group of paths and click here to delete it");

    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getGoButton()->setToolTip("Select a group of paths and click here to display its paths");

    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getEditButton()->setToolTip("Select a group of paths and click here to modify it");
}

/// called when the Enter key is pressed
void GroupsPathsWidget::keyPressEvent(QKeyEvent* event){
    /// this is the enter key
    if(!event->text().compare("\r")){
        /// if a group is being created
        if(creatingGroup)
            emit newPathGroup(groupNameEdit->text());
        else {
        /// if a group is being edited
            qDebug() << "editing group";
            switch(nameError){
            case 0:
                emit modifiedGroup(buttonGroup->getModifyEdit()->text());
                //setLastCheckedButton("");
                break;
            case 1:
                emit modifiedGroup(lastCheckedButton);
                //setLastCheckedButton("");
                break;
            case 2:
                emit messageCreationGroup(TEXT_COLOR_DANGER, "A group with the same name already exists, please choose another name for your group");
                break;
            default:
                Q_UNREACHABLE();
                qDebug() << "if you get here you probably forgot to implement the behavior for one or more error codes";
                break;
            }
        }
    }
    /// this is the escape key, if the name given is empty while creating or editing a group
    /// the widget is just reset like nothing happened
    else if(!event->text().compare("\u001B")){
        if(creatingGroup)
            emit newPathGroup("");
        else {
            if(nameError == 1){
                emit modifiedGroup(lastCheckedButton);
                setLastCheckedButton("");
            }
        }
    }
    else if(event->key() == Qt::Key_Delete){
        if(buttonGroup->getButtonGroup()->checkedButton())
            emit deleteGroup();
    }
}

void GroupsPathsWidget::uncheck(){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->getButtonGroup()->setExclusive(false);
    if(buttonGroup->getButtonGroup()->checkedButton())
        buttonGroup->getButtonGroup()->checkedButton()->setChecked(false);
    buttonGroup->getButtonGroup()->setExclusive(true);
}

/// to signify the mainwindow that a new group has been created
void GroupsPathsWidget::newGroupPaths(){
    emit newPathGroup(groupNameEdit->text());
}

void GroupsPathsWidget::cancelCreationGroup(){
    qDebug() << "GroupsPathsWidget::cancelCreationGroup called";
    /// resets the last checked button to no button
    setLastCheckedButton("");

    /// the widgets to create a group are no longer necessary so we hide them
    hideCreationWidgets();

    actionButtons->getPlusButton()->setEnabled(true);

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
    buttonGroup->setEnabledGroup(true);
    emit updatePathGroupButtons();
    QWidget::hideEvent(event);
}

void GroupsPathsWidget::showEvent(QShowEvent *event){
    hideCreationWidgets();
    emit updatePathGroupButtons();
    buttonGroup->getModifyEdit()->hide();
    QWidget::showEvent(event);
}

void GroupsPathsWidget::hideCreationWidgets(){
    /// hides everything that's related to the creation of a group
    groupNameEdit->hide();
    groupNameLabel->hide();
    saveButton->hide();
    cancelButton->hide();
    /// disables buttons except the plus button
    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getMinusButton()->setEnabled(false);
}

void GroupsPathsWidget::resetWidget(){
    actionButtons->getPlusButton()->setEnabled(true);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getEditButton()->setEnabled(false);
    setLastCheckedButton("");
}

void GroupsPathsWidget::initializeActionButtons(void){
    /// initializes the appropriate tooltips
    actionButtons->getEditButton()->setToolTip("Select a group of paths and click here to modify it");
    actionButtons->getMinusButton()->setToolTip("Select a group of paths and click here to delete it");
    actionButtons->getGoButton()->setToolTip("Select a group of paths and click here to display its paths");
    actionButtons->getPlusButton()->setToolTip("Click to add a new group of paths");

    /// disables buttons until one of the group of paths is clicked
    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getMinusButton()->setEnabled(false);
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getGoButton()->setEnabled(false);
}

void GroupsPathsWidget::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 10;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}

