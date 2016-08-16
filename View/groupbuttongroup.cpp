#include "groupbuttongroup.h"
#include "Model/points.h"
#include "Model/point.h"
#include "View/pointview.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QDebug>
#include <QMouseEvent>
#include <QLabel>
#include "View/doubleclickablebutton.h"
#include "View/customizedlineedit.h"
# include "colors.h"

GroupButtonGroup::GroupButtonGroup(const QSharedPointer<Points> &_points, QWidget* _parent): QWidget(_parent), parent(_parent), points(_points)
{
    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    buttonGroup = new QButtonGroup(this);

    /// to modify the name of a group
    modifyEdit = new CustomizedLineEdit(this);
    modifyEdit->setFixedWidth(1.29*modifyEdit->width());
    modifyEdit->hide();


    /// we are going to make this widget visible when a user wants to modify a group
    layout->addWidget(modifyEdit);

    editedGroupName = "";

    int index = 0;
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(NO_GROUP_NAME)!= 0
                && i.key().compare(TMP_GROUP_NAME)!= 0
                && i.key().compare(PATH_GROUP_NAME)!= 0){
            DoubleClickableButton* groupButton = new DoubleClickableButton(i.key(), this);
            groupButton->setFlat(true);
            groupButton->setAutoDefault(true);
            groupButton->setStyleSheet("QPushButton {color: "+text_color+";text-align:left;border: 4px; padding: 10px;}QPushButton:hover{background-color: "+button_hover_color+";}QPushButton:checked{background-color: "+button_checked_color+";}");
            groupButton->setCheckable(true);
            buttonGroup->addButton(groupButton, index);
            groupButton->setAutoExclusive(true);
            layout->addWidget(groupButton);
            groupButton->setIconSize(BUTTON_SIZE);
            if(points->isDisplayed(i.key()))
                groupButton->setIcon(QIcon(":/icons/folder_eye.png"));
            else
                groupButton->setIcon(QIcon(":/icons/folder_space.png"));
            groupButton->setIconSize(BUTTON_SIZE);
            index++;
        }
    }


    /// for the last group we just want to show the points and not "no group"
    if(points->getGroups()->value(NO_GROUP_NAME)){
        for(int i = 0; i < points->getGroups()->value(NO_GROUP_NAME)->size(); i++){
            QSharedPointer<Point> currentPoint = points->getGroups()->value(NO_GROUP_NAME)->at(i)->getPoint();
            DoubleClickableButton* pointButton = new DoubleClickableButton(currentPoint->getName(), this);
            pointButton->setAutoDefault(true);
            pointButton->setFlat(true);
            pointButton->setStyleSheet("QPushButton {color: "+text_color+"; text-align:left;border: 4px; padding: 10px;}QPushButton:hover{background-color: "+button_hover_color+";}QPushButton:checked{background-color: "+button_checked_color+";}");

            pointButton->setCheckable(true);
            buttonGroup->addButton(pointButton, index + i);
            layout->addWidget(pointButton);
            if(points->isDisplayed(NO_GROUP_NAME))
                pointButton->setIcon(QIcon(":/icons/eye_point.png"));
            else
                pointButton->setIcon(QIcon(":/icons/space_point.png"));
            pointButton->setIconSize(BUTTON_SIZE);
        }
    }

    connect(modifyEdit, SIGNAL(textEdited(QString)), this, SLOT(checkEditGroupName(QString)));
    layout->setContentsMargins(0,0,30,0);

}

void GroupButtonGroup::deleteButtons(void){
    qDebug() << "GroupButtonGroup::deleteButtons called";
    layout->removeWidget(modifyEdit);
    while(QLayoutItem* item = layout->takeAt(0)){
        if(item){
            if(QWidget* button = item->widget())
                delete button;
        } else {
            qDebug() << "oops";
        }
    }
    layout->addWidget(modifyEdit);
}

void GroupButtonGroup::updateButtons(){
    qDebug() << "GroupButtonGroup::update called";

    deleteButtons();

    int index = 0;
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(NO_GROUP_NAME)!= 0
                && i.key().compare(TMP_GROUP_NAME)!= 0
                && i.key().compare(PATH_GROUP_NAME)!= 0){

            if(i.key().compare(editedGroupName) == 0){
                modifyEdit = new CustomizedLineEdit(this);
                modifyEdit->setFixedWidth(1.29*modifyEdit->width());
                layout->addWidget(modifyEdit);
                modifyEdit->hide();
            }

            DoubleClickableButton* groupButton = new DoubleClickableButton(i.key(), this);
            groupButton->setFlat(true);
            groupButton->setAutoDefault(true);
            groupButton->setStyleSheet("QPushButton {color: "+text_color+";text-align:left;border: 4px; padding: 10px;}QPushButton:hover{background-color: "+button_hover_color+";}QPushButton:checked{background-color: "+button_checked_color+";}");
            groupButton->setCheckable(true);
            groupButton->setIconSize(BUTTON_SIZE);
            buttonGroup->addButton(groupButton, index);
            layout->addWidget(groupButton);
            if(points->isDisplayed(i.key()))
                groupButton->setIcon(QIcon(":/icons/folder_eye.png"));
            else
                groupButton->setIcon(QIcon(":/icons/folder_space.png"));
            groupButton->setIconSize(BUTTON_SIZE);
            index++;
        }
    }

    /// for the last group we just want to show the points and not "no group"
    if(points->getGroups()->value(NO_GROUP_NAME)){
        for(int i = 0; i < points->getGroups()->value(NO_GROUP_NAME)->size(); i++){
            QSharedPointer<Point> currentPoint = points->getGroups()->value(NO_GROUP_NAME)->at(i)->getPoint();
            DoubleClickableButton* pointButton = new DoubleClickableButton(currentPoint->getName(), this);
            pointButton->setFlat(true);
            pointButton->setAutoDefault(true);
            pointButton->setStyleSheet("QPushButton {color: "+text_color+";text-align:left;border: 4px; padding: 10px;}QPushButton:hover{background-color: "+button_hover_color+";}QPushButton:checked{background-color: "+button_checked_color+";}");
            pointButton->setCheckable(true);
            buttonGroup->addButton(pointButton, index + i);
            layout->addWidget(pointButton);
            if(points->isDisplayed(NO_GROUP_NAME))
                pointButton->setIcon(QIcon(":/icons/eye_point.png"));
            else
                pointButton->setIcon(QIcon(":/icons/space_point.png"));
            pointButton->setIconSize(BUTTON_SIZE);
        }
    }

    emit modifyEditReconnection();
    emit updateConnectionsRequest();
}

void GroupButtonGroup::uncheck(void){
    //qDebug() << "GroupButtonGroup::uncheck called";
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

void GroupButtonGroup::mouseDoubleClickEvent(QMouseEvent * /* unused */){
    qDebug() << "GroupButtonGroup::mouseDoubleClickEvent called";
    if(buttonGroup->checkedButton())
        emit doubleClick(((DoubleClickableButton *)buttonGroup->checkedButton())->getRealName());

}

void GroupButtonGroup::setEnabled(const bool enable){
    qDebug() << "GroupButtonGroup::setEnabled called";
    foreach(QAbstractButton* button, buttonGroup->buttons())
        button->setEnabled(enable);
}

int GroupButtonGroup::checkEditGroupName(QString name){
    qDebug() << "GroupButtonGroup::checkEditGroupName called";
    modifyEdit->setText(formatName(modifyEdit->text()));
    name = modifyEdit->text().simplified();
    if(!name.compare(editedGroupName, Qt::CaseInsensitive)){
        qDebug() << "same name";
        emit codeEditGroup(0);
        return 0;
    }
    if(!name.compare("")){
        emit codeEditGroup(1);
        return 1;
    }

    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
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

QString GroupButtonGroup::formatName(const QString name) const {
    qDebug() << "GroupButtonGroup::formatName called";

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


int GroupButtonGroup::getEditedGroupId(void) const{
    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(((DoubleClickableButton *) getButtonGroup()->buttons().at(i))->getRealName().compare(editedGroupName) == 0){
            return i;
        }
    }
    return -1;
}

QAbstractButton* GroupButtonGroup::getButtonByName(const QString name) const {
    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(((DoubleClickableButton *) getButtonGroup()->buttons().at(i))->getRealName().compare(name) == 0)
            return getButtonGroup()->buttons().at(i);
    }
    return NULL;
}
int GroupButtonGroup::getButtonIdByName(const QString name) const {
    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(((DoubleClickableButton *) getButtonGroup()->buttons().at(i))->getRealName().compare(name) == 0)
            return i;
    }
    return -1;
}
