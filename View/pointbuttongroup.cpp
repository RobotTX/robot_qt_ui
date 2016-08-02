#include "pointbuttongroup.h"
#include "Model/points.h"
#include "Model/point.h"
#include <QButtonGroup>
#include <QVBoxLayout>
#include "View/doubleclickablebutton.h"
#include <QMouseEvent>
#include <QDebug>
#include "colors.h"
#include <QLabel>
#include "View/pointview.h"

PointButtonGroup::PointButtonGroup(std::shared_ptr<Points> _points, const QString _groupIndex
                                   , QWidget* parent): QWidget(parent), points(_points){
    layout = new QVBoxLayout(this);

    groupIndex = _groupIndex;
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);
    BUTTON_SIZE = parentWidget()->size()/2;
    createButtons();
}

void PointButtonGroup::setGroup(const QString _groupIndex){
    qDebug() << "PointButtonGroup::setGroup called";
    deleteButtons();
    groupIndex = _groupIndex;
    createButtons();
    emit updateConnectionsRequest();
}

void PointButtonGroup::createButtons(){

    if(points->isAGroup(groupIndex) && points->getGroups()->value(groupIndex) && points->getGroups()->value(groupIndex)->size() > 0){
        std::shared_ptr<QVector<std::shared_ptr<PointView>>> group = points->getGroups()->value(groupIndex);
        for(int j = 0; j < group->size(); j++){
            std::shared_ptr<Point> currentPoint = group->at(j)->getPoint();

            DoubleClickableButton* pointButton = new DoubleClickableButton(currentPoint->getName(), this);
            pointButton->setAutoDefault(true);
            pointButton->setFlat(true);
            pointButton->setStyleSheet("QPushButton {color: "+text_color+";text-align:left;border: 4px; padding: 10px;}QPushButton:hover{background-color: "+button_hover_color+";}QPushButton:checked{background-color: "+button_checked_color+";}");

            buttonGroup->addButton(pointButton);
            layout->addWidget(pointButton);
            if(group->at(j)->isVisible())
                pointButton->setIcon(QIcon(":/icons/eye_point.png"));
            else
                pointButton->setIcon(QIcon(":/icons/space_point.png"));
            pointButton->setIconSize(BUTTON_SIZE);


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
    qDebug() << "PointButtonGroup::setCheckable called";
    foreach(QAbstractButton* button, buttonGroup->buttons())
        button->setCheckable(checkable);
}

void PointButtonGroup::uncheck(void){
    qDebug() << "PointButtonGroup::uncheck called";
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

QAbstractButton* PointButtonGroup::getButtonByName(const QString name) const {
    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(getButtonGroup()->button(i)->text().compare(name) == 0)
            return getButtonGroup()->button(i);
    }
    return NULL;
}

int PointButtonGroup::getButtonIdByName(const QString name) const {
    qDebug() << "size="<<getButtonGroup()->buttons().size();

    for(int i = 0; i < getButtonGroup()->buttons().size(); i++){
        if(getButtonGroup()->buttons().at(i)->text().compare(name) == 0)
            return i;
    }
    return -1;
}
