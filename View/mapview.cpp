#include "mapview.h"
#include "View/pointview.h"
#include "Model/points.h"
#include "Model/point.h"
#include <QMainWindow>
#include <QMouseEvent>
#include <QDebug>
#include <QGraphicsSceneMouseEvent>
#include "mainwindow.h"
#include "Model/map.h"


MapView::MapView (const QPixmap& pixmap, const QSize _size, std::shared_ptr<Map> _map, QMainWindow* _mainWindow) :
    QGraphicsPixmapItem(pixmap), size(_size), state(GraphicItemState::NO_STATE), mainWindow(_mainWindow), map(_map), idTmp(0)
{
    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton);

    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);
    connect(this, SIGNAL(pointLeftClicked(QString)), mainWindow, SLOT(setSelectedPoint(QString)));

}


void MapView::mousePressEvent(QGraphicsSceneMouseEvent *event){
    /// On mouse press event, we set the drag start position
    dragStartPosition = this->pos();
    QGraphicsPixmapItem::mousePressEvent(event);
}

void MapView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    qDebug() << "mapview mouseReleaseEvent";

    float x = dragStartPosition.x() - this->pos().x();
    float y = dragStartPosition.y() - this->pos().y();
    /// we compare the start position of the drag event & the drop position
    /// if we have moved for more than 10 pixels, it's a drag, else it's a click
    /// and we create a temporary point
    if (abs(x) <= 10 && abs(y) <= 10){
        /// click
        if(state == GraphicItemState::NO_STATE){
            qDebug() << "(MapView) NO_STATE";
            std::shared_ptr<PointView> tmpPointView = points->getTmpPointView();
            tmpPointView->show();
            /// might be useless code
            tmpPointView->setPos(event->pos().x(), event->pos().y());
            emit pointLeftClicked(tmpPointView->getPoint()->getName());
        } else if(state == GraphicItemState::CREATING_PATH){
            /// if it's not a white point of the map we cannot add it to the path
            qDebug() << "(MapView) CREATING_PATH";
   /*         if(map->getMapImage().pixelColor(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height()).red() >= 254){

                qDebug() << "Clicked on the map while creating a path";
                Point tmpPoint("tmpPoint" + QString::number(idTmp++), 0.0, 0.0, Point::PointType::PATH);
                PointView* newPointView = new PointView(std::make_shared<Point>(tmpPoint), this);

                newPointView->setState(GraphicItemState::CREATING_PATH);
                newPointView->getPoint()->setPosition(event->pos().x(), event->pos().y());

                newPointView->setPos(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height());

                newPointView->setParentItem(this);

                pathCreationPoints.push_back(newPointView);
                emit addPathPointMapView(&(*(newPointView->getPoint())));


           } else {
               emit newMessage("You cannot create a point here because your robot cannot go there. You must click known areas of the map");
               qDebug() << "sorry cannot create a point here";
            }*/
        } else if(state == GraphicItemState::EDITING_PERM){
            qDebug() << "(MapView) EDITING_PERM " << event->pos().x() << event->pos().y();;
            /// to notify the point information menu that the position has changed and so the point can be displayed at its new position
            emit newCoordinates(event->pos().x(), event->pos().y());
        } else if(state == GraphicItemState::SELECTING_HOME){
            qDebug() << "(MapView) SELECTING_HOME";
            std::shared_ptr<PointView> tmpPointView = points->getTmpPointView();
            tmpPointView->show();
            /// might be useless code
            tmpPointView->setPos(event->pos().x(), event->pos().y());
            emit homeSelected(tmpPointView->getPoint()->getName());
        } else if(state == GraphicItemState::EDITING_HOME){
            qDebug() << "(MapView) EDITING_HOME";
            std::shared_ptr<PointView> tmpPointView = points->getTmpPointView();
            tmpPointView->show();
            /// might be useless code
            tmpPointView->setPos(event->pos().x(), event->pos().y());
            emit homeEdited(tmpPointView->getPoint()->getName());
        } else if(state == GraphicItemState::EDITING){
            qDebug() << "(MapView) EDITING" << event->pos().x() << event->pos().y();;
            /// to notify that a point which belongs to the path of a robot has been changed
            //emit newCoordinatesPathPoint(event->pos().x(), event->pos().y());
        } else {
            qDebug() << "(MapView) NO EVENT";
        }
    }
    /// else drag
    QGraphicsPixmapItem::mouseReleaseEvent(event);
}
void MapView::setState(const GraphicItemState _state){
    qDebug() << "mapview setstate" << _state;
    state = _state;
}

 void MapView::updateHover(QString oldName, QString newName){
     qDebug() << "MapView::updateHover from" << oldName << "to" << newName;
     /*PointView* pointView = permanentPoints->getPointViewFromName(oldName);
     pointView->setToolTip(newName);*/
 }

 /// so that the icon of a point view remains consistent while editing a point of a path and after
 void MapView::updatePixmapHover(PointView::PixmapType type, QString pointName){
    Q_UNUSED(type)
    std::shared_ptr<PointView> pointView = points->findPointView(pointName);
    if(pointView && state == GraphicItemState::EDITING)
        pointView->setType(PointView::PixmapType::HOVER);
 }

 void MapView::addPathPointMapViewSlot(PointView* _pointView){
     qDebug() << "addPathPointMapViewSlot called";
     //emit addPathPointMapView(&(*(_pointView->getPoint())));
 }


 void MapView::addPathPoint(PointView* pointView){
     qDebug() << "MapView::addPathPoint called";
/*
     PointView* newPointView = new PointView(std::make_shared<Point>(*(pointView->getPoint())), this);

     newPointView->setState(GraphicItemState::CREATING_PATH);
     //newPointView->setPos(pointView->pos().x()+tmpPointPixmap.width()/2, pointView->pos().y()+tmpPointPixmap.height());
     //newPointView->setPos(pointView->pos().x(), pointView->pos().y());
     newPointView->setParentItem(this);

     pathCreationPoints.push_back(newPointView);*/
 }

 void MapView::addPointEditPath(Point pt){

    qDebug() << "mapview addPointEditPath point" << pt.getName();
    /*PointView* newPointView = new PointView(std::make_shared<Point>(pt), this);

    newPointView->setState(GraphicItemState::CREATING_PATH);

    newPointView->setParentItem(this);
    pathCreationPoints.push_back(newPointView);

    emit addPathPointMapView(&(*(newPointView->getPoint())));*/
 }

 void MapView::deletePointView(Point pt){
     /*for(int j = 0; j < pathCreationPoints.size(); j++){
        if(pt.comparePos(pathCreationPoints.at(j)->getPoint()->getPosition().getX(),
            pathCreationPoints.at(j)->getPoint()->getPosition().getY())){
            pathCreationPoints.remove(j);
        }
     }*/
 }

 void MapView::changeOrderPathPoints(const int start, const int row){
     /*PointView* pv = pathCreationPoints.takeAt(start);
     if(row > pathCreationPoints.size()){
         qDebug() << "putting pv at the end";
         pathCreationPoints.push_back(pv);
     }
     else{
         if(start < row){
             qDebug() << "inserting pv in position" << row-1;
             pathCreationPoints.insert(row-1, pv);
         } else {
             qDebug() << " inserting pv in position " << row;
             pathCreationPoints.insert(row, pv);
         }
     }*/
 }

 /// called when a permanent point is added to the path
 void MapView::addPermanentPointToPath(PointView *pointV){
    //pathCreationPoints.push_back(pointV);
 }

 int MapView::findIndexInPathByName(const QString name){
     /*for(int i = 0; i < pathCreationPoints.count(); i++){
         std::shared_ptr<Point> currPoint = pathCreationPoints.at(i)->getPoint();
         if(currPoint->isPermanent()){
             if(!currPoint->getName().compare(name))
                 return i;
         }
     }*/
     return -1;
 }

 void MapView::replacePermanentPathPoint(const int index, PointView *const pv){
     qDebug() << "replace permanent path point called";
     //pathCreationPoints.replace(index, pv);
 }


 void MapView::setPoints(std::shared_ptr<Points> _points){
     qDebug() << "MapView::setPoints called" << _points->count();
    points = _points;
 }
