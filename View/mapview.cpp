#include "mapview.h"
#include "View/pointview.h"
#include "View/pointsview.h"
#include "Model/points.h"
#include "View/groupview.h"
#include "Model/point.h"
#include <QMainWindow>
#include <QMouseEvent>
#include <QDebug>
#include <QGraphicsSceneMouseEvent>
#include "mainwindow.h"
#include "Model/map.h"


MapView::MapView (const QPixmap& pixmap, const QSize _size, std::shared_ptr<Map> _map, QMainWindow* _mainWindow) :
    QGraphicsPixmapItem(pixmap), size(_size), state(GraphicItemState::NO_STATE), mainWindow(_mainWindow), map(_map)
{
    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton);

    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);

    /// Temporary point icon
    Point tmpPoint("tmpPoint", 0.0, 0.0, false);

    tmpPointView = new PointView(std::make_shared<Point>(tmpPoint), this);
    tmpPointView->setPixmap(PointView::PixmapType::MID);
    connect(this, SIGNAL(pointLeftClicked(PointView*, bool)), mainWindow, SLOT(setSelectedPoint(PointView*, bool)));
    connect(tmpPointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));

    connect(tmpPointView, SIGNAL(addPointPath(PointView*)), this, SLOT(addPathPointMapViewSlot(PointView*)));
    connect(tmpPointView, SIGNAL(homeSelected(PointView*, bool)), mainWindow, SLOT(homeSelected(PointView*, bool)));
    connect(tmpPointView, SIGNAL(homeEdited(PointView*, bool)), mainWindow, SLOT(homeEdited(PointView*, bool)));

    point = static_cast<QSharedPointer<PointView>>(tmpPointView);

    tmpPointView->hide();
}

MapView::~MapView(){
    qDebug() << "creation mapview";
    delete permanentPoints;
    qDeleteAll(pathCreationPoints.begin(), pathCreationPoints.end());
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
            tmpPointView->show();
            /// might be useless code
            point->getPoint()->setPosition(event->pos().x(), event->pos().y());
            point->setPos(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height());
            point->setParentItem(this);
            emit pointLeftClicked(&(*point), true);
        } else if(state == GraphicItemState::CREATING_PATH){
            /// if it's not a white point of the map we cannot add it to the path

            if(map->getMapImage().pixelColor(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height()).red() >= 254){
                qDebug() << "Clicked on the map while creating a path";
                Point tmpPoint("tmpPoint", 0.0, 0.0, false);
                PointView* newPointView = new PointView(std::make_shared<Point>(tmpPoint), this);

                connect(newPointView, SIGNAL(addPointPath(PointView*)), mainWindow, SLOT(addPathPoint(PointView*)));
                connect(newPointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));
                connect(newPointView, SIGNAL(pathPointChanged(double, double, PointView*)), mainWindow, SLOT(updatePathPoint(double, double, PointView*)));

                newPointView->setState(GraphicItemState::CREATING_PATH);
                newPointView->getPoint()->setPosition(event->pos().x(), event->pos().y());

                newPointView->setPos(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height());

                newPointView->setParentItem(this);

                pathCreationPoints.push_back(newPointView);
                connect(newPointView, SIGNAL(hoverEventSignal(PointView::PixmapType, PointView*)), this, SLOT(updatePixmapHover(PointView::PixmapType, PointView*)));
                emit addPathPointMapView(&(*(newPointView->getPoint())));


           } else {
               emit newMessage("You cannot create a point here because your robot cannot go there. You must click known areas of the map");
               qDebug() << "sorry cannot create a point here";
            }
        } else if(state == GraphicItemState::EDITING_PERM){
            qDebug() << "(MapView) EDITING_PERM " << event->pos().x() << event->pos().y();;
            /// to notify the point information menu that the position has changed and so the point can be displayed at its new position
            emit newCoordinates(event->pos().x(), event->pos().y());
        } else if(state == GraphicItemState::SELECTING_HOME){
            qDebug() << "(MapView) SELECTING_HOME";
            Point tmpPoint("tmpPoint", 0.0, 0.0, false);
            PointView* newPointView = new PointView(std::make_shared<Point>(tmpPoint), this);

            connect(newPointView, SIGNAL(pointLeftClicked(PointView*)), mainWindow, SLOT(displayPointEvent(PointView*)));
            connect(newPointView, SIGNAL(addPointPath(PointView*)), mainWindow, SLOT(addPathPoint(PointView*)));
            connect(newPointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));
            connect(newPointView, SIGNAL(homeSelected(PointView*, bool)), mainWindow, SLOT(homeSelected(PointView*, bool)));
            connect(newPointView, SIGNAL(homeEdited(PointView*, bool)), mainWindow, SLOT(homeEdited(PointView*, bool)));

            newPointView->setState(GraphicItemState::SELECTING_HOME);
            newPointView->getPoint()->setPosition(event->pos().x(), event->pos().y());
            newPointView->setPos(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height());
            newPointView->setParentItem(this);
            emit homeSelected(newPointView, true);
        } else if(state == GraphicItemState::EDITING_HOME){
            qDebug() << "(MapView) EDITING_HOME";
            Point tmpPoint("tmpPoint", 0.0, 0.0, false);
            PointView* newPointView = new PointView(std::make_shared<Point>(tmpPoint), this);

            connect(newPointView, SIGNAL(addPointPath(PointView*)), mainWindow, SLOT(addPathPoint(PointView*)));
            connect(newPointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));
            connect(newPointView, SIGNAL(homeSelected(PointView*, bool)), mainWindow, SLOT(homeSelected(PointView*, bool)));
            connect(newPointView, SIGNAL(homeEdited(PointView*, bool)), mainWindow, SLOT(homeEdited(PointView*, bool)));

            newPointView->setState(GraphicItemState::EDITING_HOME);
            newPointView->getPoint()->setPosition(event->pos().x(), event->pos().y());
            newPointView->setPos(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height());
            newPointView->setParentItem(this);
            emit homeEdited(newPointView, true);
        } else if(state == GraphicItemState::EDITING){
            qDebug() << "(MapView) EDITING" << event->pos().x() << event->pos().y();;
            /// to notify that a point which belongs to the path of a robot has been changed
            emit newCoordinatesPathPoint(event->pos().x(), event->pos().y());
        } else {
            qDebug() << "(MapView) NO EVENT";
        }
    }
    /// else drag
    QGraphicsPixmapItem::mouseReleaseEvent(event);
}

void MapView::addPathPointMapViewSlot(PointView* _pointView){
    qDebug() << "addPathPointMapViewSlot called";
    emit addPathPointMapView(&(*(_pointView->getPoint())));
}

void MapView::setState(const GraphicItemState _state, const bool clear){
    qDebug() << "mapview setstate";

    state = _state;
    tmpPointView->setState(state);
    if(clear){
        qDeleteAll(pathCreationPoints.begin(), pathCreationPoints.end());
        pathCreationPoints.clear();
    } else {
        for(int i = 0; i < pathCreationPoints.size(); i++){
            pathCreationPoints.at(i)->setState(state);
        }
    }
}

 void MapView::updateHover(QString oldName, QString newName){
     qDebug() << "gotta update the hover of a point";
     PointView* pointView = permanentPoints->getPointViewFromName(oldName);
     pointView->setToolTip(newName);
 }

 void MapView::addPathPoint(PointView* pointView){
     qDebug() << "MAP VIEW : addpathpoint called";
     PointView* newPointView = new PointView(std::make_shared<Point>(*(pointView->getPoint())), this);

     connect(newPointView, SIGNAL(addPointPath(PointView*)), mainWindow, SLOT(addPathPoint(PointView*)));
     connect(newPointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));


     newPointView->setState(GraphicItemState::CREATING_PATH);
     //newPointView->setPos(pointView->pos().x()+tmpPointPixmap.width()/2, pointView->pos().y()+tmpPointPixmap.height());
     //newPointView->setPos(pointView->pos().x(), pointView->pos().y());
     newPointView->setParentItem(this);
     pathCreationPoints.push_back(newPointView);
 }

 void MapView::clearPointViews(){
    point = static_cast<QSharedPointer<PointView>>(tmpPointView);
    permanentPoints->getGroups().clear();
 }

 void MapView::setPermanentPoints(std::shared_ptr<Points> const& points){
    qDebug() << "setPermanentPoints" << permanentPoints->getGroups().size()
             << permanentPoints->getPoints()->getGroups().size();
    tmpPointView->hide();
    delete permanentPoints;
    permanentPoints = new PointsView(points, this);

    for(size_t i = 0; i < permanentPoints->getGroups().size(); i++){
        for(size_t j = 0; j < permanentPoints->getGroups().at(i)->getPointViews().size(); j++){
            PointView* currentPointView = permanentPoints->getGroups().at(i)->getPointViews().at(j);
            currentPointView->setParentItem(this);
            /// in case the point is not displayed we hide the point view
            if(!currentPointView->getPoint()->isDisplayed())
                currentPointView->hide();
            connect(&(*currentPointView), SIGNAL(pointLeftClicked(PointView*)), mainWindow, SLOT(displayPointEvent(PointView*)));
            /// to update the coordinates of the point displayed on the left when a user drags a point to change its position
            connect(&(*currentPointView), SIGNAL(editedPointPositionChanged(double, double)), mainWindow, SLOT(updateCoordinates(double, double)));
        }
    }
 }

 void MapView::setPermanentPoints(PointsView* pointsView){
     permanentPoints = pointsView;
     for(size_t i = 0; i < permanentPoints->getGroups().size(); i++){
         for(size_t j = 0; j < permanentPoints->getGroups().at(i)->getPointViews().size(); j++){
             PointView* currentPointView = permanentPoints->getGroups().at(i)->getPointViews().at(j);
             currentPointView->setParentItem(this);
             /// in case the point is not displayed we hide the point view
             if(!currentPointView->getPoint()->isDisplayed())
                 currentPointView->hide();
             connect(&(*currentPointView), SIGNAL(pointLeftClicked(PointView*)), mainWindow, SLOT(displayPointEvent(PointView*)));
             /// to update the coordinates of the point displayed on the left when a user drags a point to change its position
             connect(&(*currentPointView), SIGNAL(editedPointPositionChanged(double, double)), mainWindow, SLOT(updateCoordinates(double, double)));
         }
     }
 }

 void MapView::addPointView(PointView* const& _pointView){
     qDebug() << "addpointview called";
     _pointView->setParentItem(this);
    connect(_pointView, SIGNAL(pointLeftClicked(PointView*)), mainWindow, SLOT(displayPointEvent(PointView*)));

    /// to update the coordinates of the point displayed on the left when a user drags a point to change its position
    connect(_pointView, SIGNAL(editedPointPositionChanged(double, double)), mainWindow, SLOT(updateCoordinates(double, double)));

    connect(_pointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));

    connect(_pointView, SIGNAL(addPointPath(PointView*)), this, SLOT(addPathPointMapViewSlot(PointView*)));

    connect(_pointView, SIGNAL(homeSelected(PointView*, bool)), mainWindow, SLOT(homeSelected(PointView*, bool)));
    connect(_pointView, SIGNAL(homeEdited(PointView*, bool)), mainWindow, SLOT(homeEdited(PointView*, bool)));

    connect(_pointView, SIGNAL(pathPointChanged(double,double, PointView*)), mainWindow, SLOT(pathPointChanged(double, double, PointView*)));

    permanentPoints->addPointView(_pointView);
}

 /// so that the icon of a point view remains consistent while editing a point of a path and after
 void MapView::updatePixmapHover(PointView::PixmapType type, PointView *pv){
    if(state == GraphicItemState::EDITING){
        qDebug() << "putting the right pixmaps back";
        qDebug() << type;
        pv->setLastType(type);
        pv->setType(PointView::PixmapType::HOVER);
    }
 }

 void MapView::addPointEditPath(Point pt)
 {

    qDebug() << "mapview addPointEditPath";
       PointView* newPointView = new PointView(std::make_shared<Point>(pt), this);
      connect(newPointView, SIGNAL(addPointPath(PointView*)), mainWindow, SLOT(addPathPoint(PointView*)));
      connect(newPointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));
      connect(newPointView, SIGNAL(pathPointChanged(double, double, PointView*)), mainWindow, SLOT(updatePathPoint(double, double, PointView*)));

      newPointView->setState(GraphicItemState::CREATING_PATH);

      newPointView->setParentItem(this);
      pathCreationPoints.push_back(newPointView);

      connect(newPointView, SIGNAL(hoverEventSignal(PointView::PixmapType, PointView*)), this, SLOT(updatePixmapHover(PointView::PixmapType, PointView*)));
       emit addPathPointMapView(&(*(newPointView->getPoint())));

 }
