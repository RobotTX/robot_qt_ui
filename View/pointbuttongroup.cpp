#include "pointbuttongroup.h"
#include "Model/points.h"
#include "Model/point.h"
#include <QButtonGroup>
#include <QVBoxLayout>
#include "View/custompushbutton.h"
#include <QMouseEvent>
#include <QDebug>
#include <QLabel>
#include "View/pointview.h"
#include "View/stylesettings.h"

PointButtonGroup::PointButtonGroup(QSharedPointer<Points> _points, const QString _groupName
                                   , QWidget* parent): QWidget(parent), points(_points){
    layout = new QVBoxLayout(this);

    groupName = _groupName;
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
    createButtons();
}

void PointButtonGroup::setGroup(const QString _groupName){
    qDebug() << "PointButtonGroup::setGroup called";
    deleteButtons();
    groupName = _groupName;
    createButtons();
    setCheckable(true);
    emit updateConnectionsRequest();
}

void PointButtonGroup::createButtons(){

    if(points->isAGroup(groupName) && points->getGroups()->value(groupName) && points->getGroups()->value(groupName)->size() > 0){
        QSharedPointer<QVector<QSharedPointer<PointView>>> group = points->getGroups()->value(groupName);
        for(int j = 0; j < group->size(); j++){
            QSharedPointer<Point> currentPoint = group->at(j)->getPoint();

            CustomPushButton* pointButton = new CustomPushButton(currentPoint->getName(), this);

            buttonGroup->addButton(pointButton);
            layout->addWidget(pointButton);
            if(group->at(j)->isVisible())
                pointButton->setIcon(QIcon(":/icons/eye_point.png"));
            else
                pointButton->setIcon(QIcon(":/icons/space_point.png"));
            pointButton->setIconSize(xl_icon_size);
        }
    }
}

void PointButtonGroup::deleteButtons(void){
    qDebug() << "PointButtonGroup::deleteButtons called";
    while(QLayoutItem* item = layout->takeAt(0)){
        if(QWidget* button = item->widget())
            delete button;
    }
}

void PointButtonGroup::setCheckable(const bool checkable){
    foreach(QAbstractButton* button, buttonGroup->buttons())
        button->setCheckable(checkable);
}

void PointButtonGroup::uncheck(void){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

QAbstractButton* PointButtonGroup::getButtonByName(const QString name) const {
    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(getButtonGroup()->buttons().at(i)->text().compare(name) == 0)
            return getButtonGroup()->buttons().at(i);
    }
    return NULL;
}

int PointButtonGroup::getButtonIdByName(const QString name) const {
    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(getButtonGroup()->buttons().at(i)->text().compare(name) == 0)
            return i;
    }
    return -1;
}

void PointButtonGroup::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent()->parent());
    int maxWidth = widget->width();
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}
