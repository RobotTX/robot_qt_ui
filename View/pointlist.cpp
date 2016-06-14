#include "pointlist.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"

PointList::PointList(Points& _points)
{
    points = std::shared_ptr<Points> (&_points);
    layout = new QVBoxLayout();

    widgetsList = new QListWidget();
    //widgetsList->setDragDropMode(QAbstractItemView::InternalMove);


    layout->setAlignment(Qt::AlignTop);
    layout->addWidget(widgetsList);
    setLayout(layout);
}

PointList::~PointList(){
    delete layout;
    delete widgetsList;
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
