#include "pointsleftwidget.h"
#include "pointbuttongroup.h"
#include "customscrollarea.h"
#include "groupbuttongroup.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QHBoxLayout>
#include "View/spacewidget.h"
#include "Model/points.h"
#include "Model/point.h"
#include "Controller/mainwindow.h"
#include "View/leftmenu.h"
#include <QKeyEvent>
#include <QDebug>
#include "View/customlineedit.h"
#include "View/toplayout.h"
#include "View/pointview.h"
#include <QAbstractButton>
#include "View/custompushbutton.h"

PointsLeftWidget::PointsLeftWidget(QWidget* _parent, MainWindow* const mainWindow, QSharedPointer<Points> const& _points, bool _groupDisplayed)
    : QWidget(_parent), groupDisplayed(_groupDisplayed), points(_points), creatingGroup(true), lastCheckedId("")
{
    scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    actionButtons = new TopLeftMenu(this);
    actionButtons->setCheckable(true);

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
    groupNameEdit = new CustomLineEdit(this);
    groupNameEdit->hide();

    layout->addWidget(groupNameLabel);
    layout->addWidget(groupNameEdit);

    groupButtonGroup = new GroupButtonGroup(points, this);
    scrollArea->setWidget(groupButtonGroup);
    layout->addWidget(scrollArea);

    creationLayout = new QHBoxLayout();
    saveButton = new CustomPushButton("Save", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center", false, false);
    saveButton->hide();

    cancelButton = new CustomPushButton("Cancel", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelButton->hide();
    creationLayout->addWidget(cancelButton);
    creationLayout->addWidget(saveButton);

    layout->addLayout(creationLayout);

    connect(actionButtons->getPlusButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(plusGroupBtnEvent()));
    connect(actionButtons->getMinusButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(minusGroupBtnEvent()));
    connect(actionButtons->getEditButton(), SIGNAL(clicked(bool)), mainWindow, SLOT(editGroupBtnEvent()));
    connect(actionButtons->getGoButton(), SIGNAL(clicked()), mainWindow, SLOT(displayPointsInGroup()));
    connect(actionButtons->getMapButton(), SIGNAL(clicked()), mainWindow, SLOT(displayGroupMapEvent()));

    /// to handle double clicks
    foreach(QAbstractButton *button, groupButtonGroup->getButtonGroup()->buttons())
        connect(button, SIGNAL(doubleClick(QString)), mainWindow, SLOT(doubleClickOnGroup(QString)));

    /// to enable the buttons
    connect(groupButtonGroup->getButtonGroup(), SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(enableButtons(QAbstractButton*)));

    /// to make sure the name chosen for a new group is valid
    connect(groupNameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkGroupName(QString)));

    /// to make sure the new name chosen for a group is valid
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(textEdited(QString)), groupButtonGroup, SLOT(checkEditGroupName(QString)));

    connect(cancelButton, SIGNAL(clicked(bool)), this, SLOT(cancelCreationGroup()));

    connect(saveButton, SIGNAL(clicked(bool)), this, SLOT(emitNewGroupSignal()));

    /// to capture the moment a user stops editing whether it is to modify or create a group
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(clickSomewhere(QString)), this, SLOT(modifyGroupAfterClick(QString)));

    connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));  

    connect(this, SIGNAL(enableReturn()), mainWindow, SLOT(enableReturnAndCloseButtons()));

    /// to reconnect the modifyEdit field in the case where a user creates a new group
    connect(groupButtonGroup, SIGNAL(modifyEditReconnection()), this, SLOT(reconnectModifyEdit()));

    /// relay the signal to the mainWindow so it displays the appropriate messages to the user (e.g, you cannot change the name of the group for this one because it's empty)
    connect(groupButtonGroup, SIGNAL(codeEditGroup(int)), this, SLOT(sendMessageEditGroup(int)));

    /// to reset the path points point views after a path point is deselected
    connect(this, SIGNAL(resetPathPointViews()), mainWindow, SLOT(resetPathPointViewsSlot()));

    connect(groupNameEdit, SIGNAL(enableGroupEdit(bool)), mainWindow, SLOT(setEnableAll(bool)));
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(enableGroupEdit(bool)), mainWindow, SLOT(setEnableAll(bool)));

    connect(this, SIGNAL(deleteGroup()), mainWindow, SLOT(minusGroupBtnEvent()));

    layout->setContentsMargins(0,0,0,0);
    setAutoFillBackground(true);
}

void PointsLeftWidget::updateGroupButtonGroup(){
    qDebug() << "PointsLeftWidget::updateGroupButtonGroup called";
    groupButtonGroup->updateButtons();
}

void PointsLeftWidget::enableButtons(QAbstractButton* button){
    qDebug() << "PointsLeftWidget::enableButtons called" << button->text();
    enableButtons(button->text());
}

void PointsLeftWidget::enableButtons(QString button){
    points->setPixmapAll(PointView::PixmapType::NORMAL);
    emit resetPathPointViews();
    if(button.compare(lastCheckedId) == 0){
        groupButtonGroup->uncheck();
        lastCheckedId = "";
        disableButtons();
    } else {

        lastCheckedId = button;
        groupButtonGroup->setEditedGroupName(button);
        groupButtonGroup->getLayout()->removeWidget(groupButtonGroup->getModifyEdit());
        groupButtonGroup->getLayout()->addWidget(groupButtonGroup->getModifyEdit());
        disableButtons();
        /// enables the minus button
        actionButtons->getMinusButton()->setEnabled(true);
        if(points->isAGroup(button))
            actionButtons->getMinusButton()->setToolTip("Click to remove the selected group");
        else
            actionButtons->getMinusButton()->setToolTip("Click to remove the selected point");

        /// enables the eye button
        actionButtons->getGoButton()->setEnabled(true);
        if(points->isAGroup(button))
            actionButtons->getGoButton()->setToolTip("Click to display the information of the selected group");
        else
            actionButtons->getGoButton()->setToolTip("Click to display the information of the selected point");

        /// enables the map button
        actionButtons->getMapButton()->setCheckable(true);
        actionButtons->getMapButton()->setEnabled(true);

        if(points->isAGroup(button)){
            if(points->isDisplayed(button)){
                actionButtons->getMapButton()->setChecked(true);
                actionButtons->getMapButton()->setToolTip("Click to hide the selected group on the map");
            } else {
                actionButtons->getMapButton()->setChecked(false);
                actionButtons->getMapButton()->setToolTip("Click to display the selected group on the map");
            }
            /// changes the pointviews of all the points displayed in the group on the map
            for(int i = 0; i < points->getGroups()->value(button)->size(); i++){
                QSharedPointer<PointView> pv = points->getGroups()->value(button)->at(i);
                if(pv->isVisible())
                    pv->setPixmap(PointView::PixmapType::SELECTED);
            }
        } else {
            QSharedPointer<PointView> pv = points->findPointView(button);
            if(pv->isVisible()){
                /// if the point is displayed, changes its pointview on the map
                points->findPointView(button)->setPixmap(PointView::PixmapType::SELECTED);
                actionButtons->getMapButton()->setChecked(true);
                actionButtons->getMapButton()->setToolTip("Click to hide the selected point on the map");
            } else {
                actionButtons->getMapButton()->setChecked(false);
                actionButtons->getMapButton()->setToolTip("Click to display the selected point on the map");
            }
            /// if this point belongs to a path we also need to set the pixmap of the path point point view
            if(QSharedPointer<PointView> pathPv = points->findPathPointView(pv->getPoint()->getPosition().getX(), pv->getPoint()->getPosition().getY())){
                qDebug() << "PATH !";
                pathPv->setPixmap(PointView::PixmapType::SELECTED);
            } else {
                qDebug() << "NOT PATH";
            }
        }

        /// enables the edit button
        actionButtons->getEditButton()->setEnabled(true);
        if(points->isAGroup(button))
            actionButtons->getEditButton()->setToolTip("Click to modify the selected group");
        else
            actionButtons->getEditButton()->setToolTip("click to modify the selected point");
    }
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

int PointsLeftWidget::checkGroupName(QString name){
    qDebug() << "checking while creating" << name;
    groupNameEdit->setText(formatName(name));
    name = name.simplified();
    if(!creatingGroup && !name.compare(groupButtonGroup->getEditedGroupName(), Qt::CaseInsensitive)){
        saveButton->setToolTip("");
        qDebug() << "same name";
        connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));
        return 0;
    }
    if(!name.compare("")){
        emit messageCreationGroup(TEXT_COLOR_NORMAL, "");
        return 1;
    }
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        bool valid(true);
        i.next();
        for(int j = 0; j < i.value()->size(); j++){
            if(!i.value()->at(j)->getPoint()->getName().compare(name.simplified(), Qt::CaseInsensitive)){
                qDebug() << name << "already exists as a point";
                valid = false;
            }
        }
        if(!name.compare(i.key(), Qt::CaseInsensitive)){
            qDebug() << "PointsLeftWidget::checkGroupName" << i.key() << "already exists";
            valid = false;

        }

        if(!valid){
            saveButton->setToolTip("A group with the same name already exists, please choose another name for your group");
            saveButton->setEnabled(false);
            connect(groupNameEdit, SIGNAL(clickSomewhere(QString)), this, SLOT(cancelCreationGroup()));
            emit messageCreationGroup(TEXT_COLOR_WARNING, "A group or point with the same name already exists, please choose another name for your group");
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
            switch(groupButtonGroup->checkEditGroupName(groupButtonGroup->getModifyEdit()->text())){
            case 0:
                emit modifiedGroup(groupButtonGroup->getModifyEdit()->text());
                setLastCheckedId("");
                break;
            case 1:
                emit modifiedGroup("");
                setLastCheckedId("");
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
    points->setPixmapAll(PointView::PixmapType::NORMAL);
    emit resetPathPointViews();
    groupNameEdit->hide();
    groupNameLabel->hide();
    actionButtons->getPlusButton()->setEnabled(true);
    saveButton->hide();
    cancelButton->hide();
    groupButtonGroup->updateButtons();
    QWidget::showEvent(event);
}

void PointsLeftWidget::resetWidget(void){
    qDebug() << "PointsLeftWidget::resetWidget called";
    groupButtonGroup->updateButtons();
    groupButtonGroup->uncheck();
    lastCheckedId=-1;
    disableButtons();
}

void PointsLeftWidget::modifyGroupAfterClick(QString name){
    qDebug() << "PointsLeftWidget::modifyGroupAfterClick called";
    emit modifiedGroupAfterClick(name);
}

void PointsLeftWidget::reconnectModifyEdit(){
    qDebug() << "PointsLeftWidget::reconnectModifyEdit called";
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(clickSomewhere(QString)), this, SLOT(modifyGroupAfterClick(QString)));
    connect(groupButtonGroup->getModifyEdit(), SIGNAL(textEdited(QString)), groupButtonGroup, SLOT(checkEditGroupName(QString)));
}

QString PointsLeftWidget::formatName(const QString name) const {
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

void PointsLeftWidget::sendMessageEditGroup(int code){
    qDebug() << "PointsLeftWidget::sendMessageEditGroup called";
    switch(code){
    case 0:
        emit messageCreationGroup(TEXT_COLOR_INFO, "To save this group press Enter or click the \"Save button\"");
        break;
    case 1:
        emit messageCreationGroup(TEXT_COLOR_INFO, "");
        break;
    case 2:
        emit messageCreationGroup(TEXT_COLOR_WARNING, "A group with the same name already exists, please choose another name for your group");
        break;
    default:
        qDebug() << "if you get here you probably forgot to implement the behavior for one or more error codes";
    }
}

void PointsLeftWidget::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 18;
    setFixedWidth(maxWidth);

    QWidget::resizeEvent(event);
}
