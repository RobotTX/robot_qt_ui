#include "groupspathswidget.h"
#include "Controller/mainwindow.h"
#include "View/topleftmenu.h"
#include <QDebug>
#include "View/customlabel.h"
#include <QVBoxLayout>
#include "Model/point.h"
#include "View/groupspathsbuttongroup.h"
#include "View/pathbuttongroup.h"
#include "View/customlineedit.h"
#include <QAbstractButton>
#include "View/customscrollarea.h"
#include <QHBoxLayout>
#include "View/custompushbutton.h"


GroupsPathsWidget::GroupsPathsWidget(QWidget* parent, MainWindow* _mainWindow, const QSharedPointer<Paths> &_paths): QWidget(parent), mainWindow(_mainWindow), paths(_paths), lastCheckedButton("")
{
    /// to scroll the QButtonGroup if there is a lot of groups of paths
    scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);

    initializeActionButtons();

    layout->addWidget(actionButtons);

    QVBoxLayout* downLayout = new QVBoxLayout();

    groupNameLabel = new CustomLabel("New group's name : ", this, false);
    groupNameLabel->hide();
    groupNameEdit = new CustomLineEdit(this);
    groupNameEdit->hide();

    downLayout->addWidget(groupNameLabel);
    downLayout->addWidget(groupNameEdit);

    buttonGroup = new GroupsPathsButtonGroup(this, paths);
    scrollArea->setWidget(buttonGroup);
    downLayout->addWidget(scrollArea);

    /// to enable the action buttons when a group is selected
    connect(buttonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(enableButtons(QAbstractButton*)));    

    /// this layout contains the widgets necessary to the creation of a group
    creationLayout = new QHBoxLayout();
    saveButton = new CustomPushButton("Save", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center", false, false);
    saveButton->hide();

    cancelButton = new CustomPushButton("Cancel", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelButton->hide();
    creationLayout->addWidget(cancelButton);
    creationLayout->addWidget(saveButton);

    downLayout->addLayout(creationLayout);

    downLayout->setAlignment(Qt::AlignBottom);
    layout->setAlignment(Qt::AlignTop);
    layout->addLayout(downLayout);

    /// to make sure the name chosen for a new group is valid
    connect(groupNameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkGroupName(QString)));

    /// to notify the user whether the name he chose is valid or not
    connect(buttonGroup->getModifyEdit(), SIGNAL(textEdited(QString)), this, SLOT(checkEditGroupName(QString)));
    connect(this, SIGNAL(codeEditGroup(int)), _mainWindow, SLOT(setMessageModifGroupPaths(int)));

    /// to save a group of paths if the name is valid
    connect(saveButton, SIGNAL(clicked()), this, SLOT(newGroupPaths()));

    /// to cancel the creation of a group of paths
    connect(cancelButton, SIGNAL(clicked(bool)), this, SLOT(cancelCreationGroup()));

    /// to handle double clicks
    foreach(QAbstractButton *button, buttonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), mainWindow, SLOT(doubleClickOnPathsGroup(QString)));

    hide();
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
            switch(checkEditGroupName(buttonGroup->getModifyEdit()->text())){
            case 0:
                emit modifiedGroup(buttonGroup->getModifyEdit()->text());
                setLastCheckedButton("");
                break;
            case 1:
                emit modifiedGroup(lastCheckedButton);
                setLastCheckedButton("");
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
    /// this is the escape key, if the name given is empty while creating or editing a group
    /// the widget is just reset like nothing happened
    else if(!event->text().compare("\u001B")){
        if(creatingGroup)
            emit newPathGroup("");
        else {
            if(checkEditGroupName(buttonGroup->getModifyEdit()->text()) == 1){
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

int GroupsPathsWidget::checkGroupName(QString name){
    qDebug() << "GroupsPathsWidget::checkGroupName checking name" << name;
    groupNameEdit->setText(formatName(name));
    /// gets rid of the extra spaces
    name = name.simplified();

    if(!name.compare("")){
        saveButton->setToolTip("The name of your group cannot be empty");
        saveButton->setEnabled(false);
        emit messageCreationGroup(TEXT_COLOR_INFO, "");
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
    emit messageCreationGroup(TEXT_COLOR_INFO, "To save this group press Enter or click the \"Save button\"");
    return 0;
}

/// to remove extra spaces like "A          word  " becomes "A word"
QString GroupsPathsWidget::formatName(const QString name) const {
    qDebug() << "GroupsPathsWidget::formatName called" << name;

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

/// update the QButtonGroup by reconstructing it from scratch
void GroupsPathsWidget::updateGroupsPaths(void){
    qDebug() << "GroupsPathsWidget::updateGroupsPaths called";
    buttonGroup->deleteButtons();
    buttonGroup->createButtons();
    /// to handle double clicks
    foreach(QAbstractButton *button, buttonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), mainWindow, SLOT(doubleClickOnPathsGroup(QString)));
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
    buttonGroup->deleteButtons();
    buttonGroup->createButtons();
    /// to reestablish the connections so we can doubleclick the buttons
    /// to handle double clicks
    foreach(QAbstractButton *button, buttonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), mainWindow, SLOT(doubleClickOnPathsGroup(QString)));
    QWidget::hideEvent(event);
}

void GroupsPathsWidget::showEvent(QShowEvent *event){
    hideCreationWidgets();
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

/// to check that the name edited for a group is valid
int GroupsPathsWidget::checkEditGroupName(QString name){
    qDebug() << "GroupPathsWidget::checkEditGroupName called";
    buttonGroup->getModifyEdit()->setText(formatName(buttonGroup->getModifyEdit()->text()));
    name =  buttonGroup->getModifyEdit()->text().simplified();
    if(!name.compare(buttonGroup->getButtonGroup()->checkedButton()->text(), Qt::CaseInsensitive)){
        qDebug() << "same name";
        /// if the name has not been changed we return 0 so that the user can save the same name
        emit codeEditGroup(0);
        return 0;
    }
    if(!name.compare("")){
        /// if the name is empty we return 1 to prevent this name from being saved
        emit codeEditGroup(1);
        return 1;
    }

    QMapIterator<QString, QSharedPointer<Paths::CollectionPaths>> i(*(paths->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(!name.compare(i.key(), Qt::CaseInsensitive)){
            qDebug() << "GroupPathsGroup::checkEditGroupName" << i.key();
            /// if the name is already the name of another group of paths we return 2, saving this name is also forbidden
            emit codeEditGroup(2);
            return 2;
        }
    }
    /// if there is nothing to report we return 0 and the name can be saved
    emit codeEditGroup(0);
    return 0;
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
    int maxWidth = widget->width() - 18;
    setFixedWidth(maxWidth);

    QWidget::resizeEvent(event);
}

