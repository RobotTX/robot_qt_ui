#include "pointlist.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"

PointList::PointList(Points& _points, QWidget* parent):QWidget(parent){
    points = std::shared_ptr<Points> (&_points);
    layout = new QVBoxLayout(this);

    widgetsList = new QListWidget(this);
    //widgetsList->setDragDropMode(QAbstractItemView::InternalMove);


    layout->setAlignment(Qt::AlignTop);
    layout->addWidget(widgetsList);
}

void PointList::display(void) const {
    std::cout << *points;
}

void PointList::setNewGroupToDisplay(const int index){
    widgetsList->clear();
    qDebug() << points->getGroups().size();
    for(int i = 0; i < points->getGroups()[index]->getPoints().size(); i++){
        std::shared_ptr<Point> curr_point = points->getGroups()[index]->getPoints()[i];
        widgetsList->addItem(curr_point->getName() + " (" + QString::number(curr_point->getPosition().getX()) + ", " + QString::number(curr_point->getPosition().getY()) + ")");
    }
}
