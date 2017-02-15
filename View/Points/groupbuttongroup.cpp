#include "groupbuttongroup.h"
#include "Model/Points/points.h"
#include "Model/Points/point.h"
#include "View/Points/pointview.h"
#include <QVBoxLayout>
#include <QDebug>
#include <QMouseEvent>
#include <QLabel>
#include "View/Other/customlineedit.h"
#include "View/Other/custompushbutton.h"
#include "View/Other/stylesettings.h"

GroupButtonGroup::GroupButtonGroup(const QSharedPointer<Points> &points, QWidget* parent)
    : QWidget(parent) {

    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    btnGroup = new QButtonGroup(this);

    /// to modify the name of a group
    modifyEdit = new CustomLineEdit(this);
    modifyEdit->setFixedWidth(1.29*modifyEdit->width());
    modifyEdit->hide();

    /// we are going to make this widget visible when a user wants to modify a group
    layout->addWidget(modifyEdit);

    editedGroupName = "";

    updateButtons(points);
}

void GroupButtonGroup::updateButtons(QSharedPointer<Points> const& points){
    //qDebug() << "GroupButtonGroup::update called";

    /// Remove buttons
    QList<QAbstractButton*> listBtn = btnGroup->buttons();
    for(int i = 0; i < listBtn.size(); i++){
        btnGroup->removeButton(listBtn.at(i));
        layout->removeWidget(listBtn.at(i));
        delete listBtn.at(i);
    }

    /// Add buttons
    int index = 0;
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        /// this keeps only the group of points
        if(i.key().compare(NO_GROUP_NAME) != 0
                && i.key().compare(TMP_GROUP_NAME) != 0
                && i.key().compare(PATH_GROUP_NAME) != 0){
            CustomPushButton* groupButton = new CustomPushButton(i.key(), this, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
            groupButton->setIconSize(xl_icon_size);
            btnGroup->addButton(groupButton, index);
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
            CustomPushButton* pointButton = new CustomPushButton(currentPoint->getName(), this, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
            btnGroup->addButton(pointButton, index + i);
            layout->addWidget(pointButton);
            if(points->findPointView(currentPoint->getName())->isVisible())
                pointButton->setIcon(QIcon(":/icons/eye_point.png"));
            else
                pointButton->setIcon(QIcon(":/icons/space_point.png"));
            pointButton->setIconSize(xl_icon_size);
        }
    }

    emit updateConnectionsRequest();
}

void GroupButtonGroup::uncheck(void){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    btnGroup->setExclusive(false);
    if(btnGroup->checkedButton())
        btnGroup->checkedButton()->setChecked(false);
    btnGroup->setExclusive(true);
}

void GroupButtonGroup::mouseDoubleClickEvent(QMouseEvent * /* unused */){
    if(btnGroup->checkedButton())
        emit doubleClick(btnGroup->checkedButton()->text());
}

void GroupButtonGroup::setEnabled(const bool enable){
    qDebug() << "GroupButtonGroup::setEnabled called";
    foreach(QAbstractButton* button, btnGroup->buttons())
        button->setEnabled(enable);
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
