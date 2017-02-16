#include "pointsleftwidget.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QDebug>
#include <QAbstractButton>
#include "Controller/mainwindow.h"
#include "Controller/Points/pointscontroller.h"
#include "Controller/TopLayout/toplayoutcontroller.h"
#include "Model/Points/points.h"
#include "Model/Points/point.h"
#include "View/Other/spacewidget.h"
#include "View/LeftMenu/leftmenu.h"
#include "View/Other/customlineedit.h"
#include "View/TopLayout/toplayoutwidget.h"
#include "View/Points/pointview.h"
#include "View/Other/custompushbutton.h"
#include "View/Other/stylesettings.h"
#include "View/Points/pointbuttongroup.h"
#include "View/Other/customscrollarea.h"
#include "View/Points/groupbuttongroup.h"

PointsLeftWidget::PointsLeftWidget(MainWindow* const mainWindow, QSharedPointer<Points> const& points)
    : QWidget(mainWindow), creatingGroup(true), lastCheckedId(""), nameError(0)
{
    CustomScrollArea* scrollArea = new CustomScrollArea(this, true);

    QVBoxLayout* layout = new QVBoxLayout(this);
    QVBoxLayout* topLayout = new QVBoxLayout();

    actionButtons = new TopLeftMenu(this);
    actionButtons->setCheckable(false);

    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getEditButton()->setEnabled(false);
    actionButtons->getMapButton()->setEnabled(false);
    actionButtons->getGoButton()->setEnabled(false);

    actionButtons->getPlusButton()->setToolTip("Click here to add a new group");
    actionButtons->getMinusButton()->setToolTip("Select a group or a point and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a group or a point and click here to modify it");
    actionButtons->getMapButton()->setToolTip("Select a group or a point and click here to display or hide it on the map");
    actionButtons->getGoButton()->setToolTip("Select a group or a point and click here to display its information");

    topLayout->addWidget(actionButtons);

    groupNameLabel = new QLabel("New group's name : ", this);
    groupNameLabel->hide();
    groupNameEdit = new CustomLineEdit(this);
    groupNameEdit->hide();

    topLayout->addWidget(groupNameLabel);
    topLayout->addWidget(groupNameEdit);
    layout->addLayout(topLayout);

    groupButtonGroup = new GroupButtonGroup(points, this);
    scrollArea->setWidget(groupButtonGroup);
    layout->addWidget(scrollArea);

    QHBoxLayout* creationLayout = new QHBoxLayout();
    saveButton = new CustomPushButton("Save", this, CustomPushButton::ButtonType::LEFT_MENU, "center", false, false);
    saveButton->hide();

    cancelButton = new CustomPushButton("Cancel", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelButton->hide();
    creationLayout->addWidget(cancelButton);
    creationLayout->addWidget(saveButton);

    layout->addLayout(creationLayout);

    connect(actionButtons->getPlusButton(), SIGNAL(clicked(bool)), mainWindow->getPointsController(), SLOT(plusGroupBtnEvent()));
    connect(actionButtons->getMinusButton(), SIGNAL(clicked(bool)), mainWindow->getPointsController(), SLOT(minusGroupBtnEvent()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked(bool)), mainWindow->getPointsController(), SLOT(editGroupBtnEvent()));
    connect(actionButtons->getGoButton(), SIGNAL(clicked()), mainWindow->getPointsController(), SLOT(displayPointsInGroup()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), mainWindow->getPointsController(), SLOT(displayGroupMapEvent()));

    /// to handle double clicks
    foreach(QAbstractButton *button, groupButtonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), mainWindow->getPointsController(), SLOT(doubleClickOnGroup(QString)));

    /// to enable the buttons
    connect(groupButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), mainWindow->getPointsController(), SLOT(enableButtonsPointsLeftWidget(QAbstractButton*)));

    /// to make sure the name chosen for a new group is valid
    connect(groupNameEdit, SIGNAL(textEdited(QString)), mainWindow->getPointsController(), SLOT(checkGroupName(QString)));

    connect(cancelButton, SIGNAL(clicked(bool)), this, SLOT(cancelCreationGroup()));

    connect(saveButton, SIGNAL(clicked(bool)), this, SLOT(emitNewGroupSignal()));

    /// to capture the moment a user stops editing whether it is to modify or create a group
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(clickSomewhere(QString)), this, SLOT(modifyGroupAfterClick(QString)));

    connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));  

    connect(this, SIGNAL(enableReturn()), mainWindow, SLOT(enableReturnAndCloseButtons()));

    /// to reset the path points point views after a path point is deselected
    connect(this, SIGNAL(resetPathPointViews()), mainWindow, SLOT(resetPathPointViewsSlot()));

    connect(groupNameEdit, SIGNAL(enableGroupEdit(bool)), mainWindow, SLOT(setEnableAll(bool)));
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(enableGroupEdit(bool)), mainWindow, SLOT(setEnableAll(bool)));

    connect(this, SIGNAL(deleteGroup()), mainWindow->getPointsController(), SLOT(minusGroupBtnEvent()));

    /// to connect the buttons in the left menu so they can be double clicked after they were updated
    connect(groupButtonGroup, SIGNAL(updateConnectionsRequest()), mainWindow->getPointsController(), SLOT(reestablishConnectionsGroups()));
    /// to create a new group, the signal is sent by pointsLeftWidget
    connect(this, SIGNAL(newGroup(QString)), mainWindow->getPointsController(), SLOT(createGroup(QString)));
    /// to modify the name of a group, the signal is sent by pointsLeftWidget
    connect(this, SIGNAL(modifiedGroup(QString)), mainWindow->getPointsController(), SLOT(modifyGroupWithEnter(QString)));
    /// same but mainWindow happens when the user clicks on a random point of the window
    connect(this, SIGNAL(modifiedGroupAfterClick(QString)), mainWindow->getPointsController(), SLOT(modifyGroupAfterClick(QString)));
    /// to know what message to display when a user is creating a group
    connect(this, SIGNAL(messageCreationGroup(QString, QString)), mainWindow->getTopLayoutController(), SLOT(setLabel(QString,QString)));
    connect(this, SIGNAL(resetPointViews()), mainWindow->getPointsController(), SLOT(resetPointViewsSlot()));

    creationLayout->setContentsMargins(0, 0, 10, 0);
    topLayout->setContentsMargins(0, 0, 10, 0);
    layout->setContentsMargins(0, 0, 0, 0);
    setAutoFillBackground(true);
}

void PointsLeftWidget::disableButtons(void){
    qDebug() << "PointsLeftWidget::disableButtons called";

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

void PointsLeftWidget::cancelCreationGroup(){
    setLastCheckedId("");
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
    emit newGroup(groupNameEdit->text().simplified());
}

void PointsLeftWidget::keyPressEvent(QKeyEvent* event){
    /// this is the enter key
    if(!event->text().compare("\r")){
        if(creatingGroup)
            emit newGroup(groupNameEdit->text());
        else{
            switch(nameError){
            case 0:
                emit modifiedGroup(groupButtonGroup->getModifyEdit()->text());
                //setLastCheckedId("");
                break;
            case 1:
                emit modifiedGroup("");
                //setLastCheckedId("");
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
    /// in this case we simply cancel the creation / edition
    else if(event->key() == Qt::Key_Escape){
        if(creatingGroup)
            emit newGroup("");
        else {
            emit modifiedGroup("");
            setLastCheckedId("");
        }
    }
    else if(event->key() == Qt::Key_Delete){
        if(groupButtonGroup->getButtonGroup()->checkedButton())
            emit deleteGroup();
    }
}

void PointsLeftWidget::showEvent(QShowEvent *event){
    resetWidget();
    emit resetPointViews();
    emit resetPathPointViews();
    groupButtonGroup->getModifyEdit()->hide();
    groupNameEdit->hide();
    groupNameLabel->hide();
    actionButtons->getPlusButton()->setEnabled(true);
    saveButton->hide();
    cancelButton->hide();
    emit updateGroupButtonGroup();
    QWidget::showEvent(event);
}

void PointsLeftWidget::resetWidget(void){
    //qDebug() << "PointsLeftWidget::resetWidget called";
    emit updateGroupButtonGroup();
    groupButtonGroup->uncheck();
    lastCheckedId=-1;
    disableButtons();
}

void PointsLeftWidget::modifyGroupAfterClick(QString name){
    //qDebug() << "PointsLeftWidget::modifyGroupAfterClick called";
    emit modifiedGroupAfterClick(name);
}

void PointsLeftWidget::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 10;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}

