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

MapView::MapView (const QPixmap& pixmap, const QSize _size, PointsView* const& points, QMainWindow* _mainWindow) :
    QGraphicsPixmapItem(pixmap), size(_size), state(GraphicItemState::NO_STATE){

    mainWindow = _mainWindow;
    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton);

    permanentPoints = points;

    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);

    for(size_t i = 0; i < permanentPoints->getGroups().size(); i++){
        for(size_t j = 0; j < permanentPoints->getGroups().at(i).getPointViews().size(); j++){
            permanentPoints->getGroups().at(i).getPointViews().at(j)->setParentItem(this);
            connect(&(*permanentPoints->getGroups().at(i).getPointViews().at(j)), SIGNAL(pointLeftClicked(PointView*)), _mainWindow, SLOT(displayPointEvent(PointView*)));
        }
    }

    /// Temporary point icon
    Point tmpPoint("tmpPoint", 0.0, 0.0, false);

    tmpPointView = new PointView(std::make_shared<Point>(tmpPoint));
    connect(this, SIGNAL(pointLeftClicked(PointView*, bool)), _mainWindow, SLOT(setSelectedPoint(PointView*, bool)));
    connect(tmpPointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));

    connect(tmpPointView, SIGNAL(addPointPath(PointView*)), this, SLOT(addPathPointMapViewSlot(PointView*)));
    point = static_cast<QSharedPointer<PointView>>(tmpPointView);

}

MapView::~MapView(){
    delete permanentPoints;
    qDeleteAll(pathCreationPoints.begin(), pathCreationPoints.end());
    delete tmpPointView;
}

void MapView::mousePressEvent(QGraphicsSceneMouseEvent *event){
    /// On mouse press event, we set the drag start position
    dragStartPosition = this->pos();
    QGraphicsPixmapItem::mousePressEvent(event);
}

void MapView::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    float x = dragStartPosition.x() - this->pos().x();
    float y = dragStartPosition.y() - this->pos().y();

    /// we compare the start position of the drag event & the drop position
    /// if we have moved for more than 10 pixels, it's a drag, else it's a click
    /// and we create a temporary point
    if (abs(x)<=10 && abs(y)<=10){
        /// click
        if(state == GraphicItemState::NO_STATE){
            point->getPoint()->setPosition(event->pos().x(), event->pos().y());
            point->setPos(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height());
            point->setParentItem(this);
            emit pointLeftClicked(&(*point), true);
        } else if(state == GraphicItemState::CREATING_PATH){
            qDebug() << "Clicked on the map while creating a path";
            Point tmpPoint("tmpPoint", 0.0, 0.0, false);
            PointView* newPointView = new PointView(std::make_shared<Point>(tmpPoint));

            connect(newPointView, SIGNAL(addPointPath(PointView*)), mainWindow, SLOT(addPathPoint(PointView*)));
            connect(newPointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));

            newPointView->setState(GraphicItemState::CREATING_PATH);
            newPointView->getPoint()->setPosition(event->pos().x(), event->pos().y());
            newPointView->setPos(event->pos().x()-tmpPointPixmap.width()/2, event->pos().y()-tmpPointPixmap.height());
            newPointView->setParentItem(this);
            pathCreationPoints.push_back(newPointView);
            emit addPathPointMapView(&(*(newPointView->getPoint())));
        } else if(state == GraphicItemState::EDITING_PERM){

        } else {
            qDebug() << "(MapView) NO EVENT";
        }
    }
    /// else drag
    QGraphicsPixmapItem::mouseReleaseEvent(event);
}

void MapView::updatePoints(const Points& points){
    for(size_t i = 0; i < permanentPoints->getGroups().size(); i++){
        for(size_t j = 0; j < permanentPoints->getGroups().at(i).getPointViews().size(); j++){
            permanentPoints->getGroups().at(i).getPointViews().at(j)->hide();
        }
    }
    permanentPoints = new PointsView(points);
    for(size_t i = 0; i < permanentPoints->getGroups().size(); i++){
        for(size_t j = 0; j < permanentPoints->getGroups().at(i).getPointViews().size(); j++){
            permanentPoints->getGroups().at(i).getPointViews().at(j)->setParentItem(this);
            connect(&(*permanentPoints->getGroups().at(i).getPointViews().at(j)), SIGNAL(pointLeftClicked(PointView*)), mainWindow, SLOT(displayPointEvent(PointView*)));
        }
    }
}

void MapView::addPathPointMapViewSlot(PointView* _pointView){
    qDebug() << "addPathPointMapViewSlot called";
    emit addPathPointMapView(&(*(_pointView->getPoint())));
}

void MapView::setState(const GraphicItemState _state, const bool clear){
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
     std::shared_ptr<PointView> pointView = permanentPoints->getPointViewFromName(oldName);
     pointView->setToolTip(newName);
 }

 void MapView::addPathPoint(PointView* pointView){
     PointView* newPointView = new PointView(std::make_shared<Point>(*(pointView->getPoint())));

     connect(newPointView, SIGNAL(addPointPath(PointView*)), mainWindow, SLOT(addPathPoint(PointView*)));
     connect(newPointView, SIGNAL(moveTmpEditPathPoint()), mainWindow, SLOT(moveTmpEditPathPointSlot()));

     newPointView->setState(GraphicItemState::CREATING_PATH);
     //newPointView->setPos(pointView->pos().x()+tmpPointPixmap.width()/2, pointView->pos().y()+tmpPointPixmap.height());
     //newPointView->setPos(pointView->pos().x(), pointView->pos().y());
     newPointView->setParentItem(this);
     pathCreationPoints.push_back(newPointView);
 }
