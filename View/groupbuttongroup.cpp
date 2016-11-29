#include "groupbuttongroup.h"
#include "Model/points.h"
#include "Model/point.h"
#include "View/pointview.h"
#include <QVBoxLayout>
#include <QDebug>
#include <QMouseEvent>
#include <QLabel>
#include "View/customlineedit.h"
#include "View/custompushbutton.h"
#include "View/stylesettings.h"

GroupButtonGroup::GroupButtonGroup(const QSharedPointer<Points> &_points, QWidget* _parent): QWidget(_parent), parentWidget(_parent), points(_points)
{
    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    buttonGroup = new QButtonGroup(this);

    /// to modify the name of a group
    modifyEdit = new CustomLineEdit(this);
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
            CustomPushButton* groupButton = new CustomPushButton(i.key(), this, false, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
            buttonGroup->addButton(groupButton, index);
            layout->addWidget(groupButton);

            if(points->isDisplayed(i.key()))
                groupButton->setIcon(QIcon(":/icons/folder_eye.png"));
            else
                groupButton->setIcon(QIcon(":/icons/folder_space.png"));

            groupButton->setIconSize(xl_icon_size);
            index++;
        }
    }

    /// for the last group we just want to show the points and not "no group"
    if(points->getGroups()->value(NO_GROUP_NAME)){
        for(int i = 0; i < points->getGroups()->value(NO_GROUP_NAME)->size(); i++){
            QSharedPointer<PointView> currentPointView = points->getGroups()->value(NO_GROUP_NAME)->at(i);
            CustomPushButton* pointButton = new CustomPushButton(currentPointView->getPoint()->getName(), this, false, CustomPushButton::ButtonType::LEFT_MENU, "left", true);

            buttonGroup->addButton(pointButton, index + i);
            layout->addWidget(pointButton);
            if(currentPointView->isVisible())
                pointButton->setIcon(QIcon(":/icons/eye_point.png"));
            else
                pointButton->setIcon(QIcon(":/icons/space_point.png"));
            pointButton->setIconSize(xl_icon_size);
        }
    }
    /// to make sure that when the name of a group is being edited, this name is valid (not empty or taken)
    connect(modifyEdit, SIGNAL(textEdited(QString)), this, SLOT(checkEditGroupName(QString)));

}

GroupButtonGroup::~GroupButtonGroup(){
    delete layout;
    layout = Q_NULLPTR;
}

void GroupButtonGroup::deleteButtons(void){
    //qDebug() << "GroupButtonGroup::deleteButtons called";
    layout->removeWidget(modifyEdit);
    while(QLayoutItem* item = layout->takeAt(0)){
        if(item){
            if(QWidget* button = item->widget()){
                delete button;
                button = 0;
            }
            delete item;
            item = 0;
        } else {
            qDebug() << "groupbuttongroup::deletebuttons oops";
        }
    }
    layout->addWidget(modifyEdit);
}

void GroupButtonGroup::updateButtons(){
    //qDebug() << "GroupButtonGroup::update called";

    deleteButtons();

    int index = 0;
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        /// this keeps only the group of points
        if(i.key().compare(NO_GROUP_NAME) != 0
                && i.key().compare(TMP_GROUP_NAME) != 0
                && i.key().compare(PATH_GROUP_NAME) != 0){
            /// sets the modifyEdit at the position of the selected group so that we can show it at the same position if the user wants to update its name
            if(i.key().compare(editedGroupName) == 0){
                modifyEdit = new CustomLineEdit(this);
                modifyEdit->setFixedWidth(1.29*modifyEdit->width());
                layout->addWidget(modifyEdit);
                modifyEdit->hide();
            }

            CustomPushButton* groupButton = new CustomPushButton(i.key(), this, false, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
            groupButton->setIconSize(xl_icon_size);
            buttonGroup->addButton(groupButton, index);
            layout->addWidget(groupButton);
            if(points->isDisplayed(i.key()))
                groupButton->setIcon(QIcon(":/icons/folder_eye.png"));
            else
                groupButton->setIcon(QIcon(":/icons/folder_space.png"));
            groupButton->setIconSize(xl_icon_size);
            index++;
        }
    }

    /// for the last group we just want to show the points and not "no group"
    if(points->getGroups()->value(NO_GROUP_NAME)){
        for(int i = 0; i < points->getGroups()->value(NO_GROUP_NAME)->size(); i++){
            QSharedPointer<Point> currentPoint = points->getGroups()->value(NO_GROUP_NAME)->at(i)->getPoint();
            CustomPushButton* pointButton = new CustomPushButton(currentPoint->getName(), this, false, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
            buttonGroup->addButton(pointButton, index + i);
            layout->addWidget(pointButton);
            if(points->findPointView(currentPoint->getName())->isVisible())
                pointButton->setIcon(QIcon(":/icons/eye_point.png"));
            else
                pointButton->setIcon(QIcon(":/icons/space_point.png"));
            pointButton->setIconSize(xl_icon_size);
        }
    }

    emit modifyEditReconnection();
    emit updateConnectionsRequest();
}

void GroupButtonGroup::uncheck(void){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

void GroupButtonGroup::mouseDoubleClickEvent(QMouseEvent * /* unused */){
    if(buttonGroup->checkedButton())
        emit doubleClick(buttonGroup->checkedButton()->text());
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
    /// if the group has not changed we allow saving it
    if(!name.compare(editedGroupName, Qt::CaseInsensitive)){
        qDebug() << "same name";
        emit codeEditGroup(0);
        return 0;
    }
    /// if the name is empty we forbid it
    if(!name.compare("")){
        emit codeEditGroup(1);
        return 1;
    }

    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        /// if the name is already taken we also forbid it
        if(!name.compare(i.key(), Qt::CaseInsensitive)){
            qDebug() << "GroupButtonGroup::checkEditGroupName" << i.key();
            emit codeEditGroup(2);
            return 2;
        }
    }
    emit codeEditGroup(0);
    return 0;
}

/// removes extra spaces like in " a  name with   extra spaces "
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

/// returns the id of the edited group
int GroupButtonGroup::getEditedGroupId(void) const{
    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(getButtonGroup()->buttons().at(i)->text().compare(editedGroupName) == 0){
            return i;
        }
    }
    return -1;
}

QAbstractButton* GroupButtonGroup::getButtonByName(const QString name) const {
    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(getButtonGroup()->buttons().at(i)->text().compare(name) == 0)
            return getButtonGroup()->buttons().at(i);
    }
    return NULL;
}

/// returns the id of the button whose text is <name>
int GroupButtonGroup::getButtonIdByName(const QString name) const {
    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(getButtonGroup()->buttons().at(i)->text().compare(name) == 0)
            return i;
    }
    return -1;
}

void GroupButtonGroup::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent()->parent()->parent());
    int maxWidth = widget->width() - 10;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}
