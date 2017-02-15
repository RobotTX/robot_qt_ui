#include "pointbuttongroup.h"
#include "Model/Points/points.h"
#include "Model/Points/point.h"
#include <QButtonGroup>
#include <QVBoxLayout>
#include "View/Other/custompushbutton.h"
#include <QMouseEvent>
#include <QDebug>
#include <QLabel>
#include "View/Points/pointview.h"
#include "View/Other/stylesettings.h"

PointButtonGroup::PointButtonGroup(QSharedPointer<Points> points, const QString _groupName
                                   , QWidget* parent): QWidget(parent){
    layout = new QVBoxLayout(this);

    groupName = _groupName;
    btnGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
    updatePoints(points);
}

void PointButtonGroup::setGroup(const QString _groupName, QSharedPointer<Points> points){
    qDebug() << "PointButtonGroup::setGroup called";
    groupName = _groupName;
    updatePoints(points);
    setCheckable(true);
    emit updateConnectionsRequest();
}

void PointButtonGroup::updatePoints(QSharedPointer<Points> points){
    if(points->isAGroup(groupName)){

        /// Remove buttons
        QList<QAbstractButton*> listBtn = btnGroup->buttons();
        for(int i = 0; i < listBtn.size(); i++){
            btnGroup->removeButton(listBtn.at(i));
            layout->removeWidget(listBtn.at(i));
            delete listBtn.at(i);
        }

        /// Add buttons
        if(points->getGroups()->value(groupName) && points->getGroups()->value(groupName)->size() > 0){
            QSharedPointer<QVector<QSharedPointer<PointView>>> group = points->getGroups()->value(groupName);
            for(int j = 0; j < group->size(); j++){
                QSharedPointer<Point> currentPoint = group->at(j)->getPoint();

                CustomPushButton* pointButton = new CustomPushButton(currentPoint->getName(), this);

                btnGroup->addButton(pointButton);
                layout->addWidget(pointButton);
                if(group->at(j)->isVisible())
                    pointButton->setIcon(QIcon(":/icons/eye_point.png"));
                else
                    pointButton->setIcon(QIcon(":/icons/space_point.png"));
                pointButton->setIconSize(xl_icon_size);
            }
        }
    }
}

void PointButtonGroup::setCheckable(const bool checkable){
    foreach(QAbstractButton* button, btnGroup->buttons())
        button->setCheckable(checkable);
}

void PointButtonGroup::uncheck(void){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    btnGroup->setExclusive(false);
    if(btnGroup->checkedButton())
        btnGroup->checkedButton()->setChecked(false);
    btnGroup->setExclusive(true);
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
    QWidget* widget = static_cast<QWidget*>(parent()->parent()->parent());
    int maxWidth = widget->width() - 10;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}
