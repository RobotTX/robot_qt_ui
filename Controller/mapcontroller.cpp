#include "mapcontroller.h"
#include "View/drawobstacles.h"
#include "Model/robots.h"
#include "View/pointview.h"
#include <QSharedPointer>
#include <QFileInfo>
#include <QDir>
#include <fstream>
#include "Controller/mainwindow.h"
#include "Controller/pointscontroller.h"

MapController::MapController(QSharedPointer<Robots> _robots, MainWindow *mainWindow): QObject(mainWindow)
{
    map = QSharedPointer<Map> (new Map());

    scene = new QGraphicsScene();

    scene->setSceneRect(0, 0, 800, 600);

    graphicsView = new CustomQGraphicsView(scene, mainWindow);

    createMapView(mainWindow, _robots);

    graphicsView->scale(std::max(graphicsView->parentWidget()->width()/scene->width(), graphicsView->parentWidget()->height()/scene->height()),
                        std::max(graphicsView->parentWidget()->width()/scene->width(), graphicsView->parentWidget()->height()/scene->height()));

    /// hides the scroll bars
    graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

    scene->addItem(view);

    /// to know what message to display when a user is creating a path
    connect(this, SIGNAL(newMessage(QString)), mainWindow, SLOT(setMessageCreationPath(QString)));
}

void MapController::createMapView(MainWindow *mainWindow, QSharedPointer<Robots> _robots){

    view = new MapView(QPixmap::fromImage(map->getMapImage()), mainWindow->geometry().size());

    connect(view, SIGNAL(leftClick()), mainWindow, SLOT(setSelectedTmpPoint()));

    /// to link the map and the point information menu when a point is being edited
    connect(view, SIGNAL(newCoordinates(double, double)), mainWindow->getPointsController(), SLOT(updateCoordinates(double, double)));

    /// to link the map and the path information when a path point is being edited (associated to a robot or not)
    connect(view, SIGNAL(newCoordinatesPathPoint(double, double)), mainWindow, SLOT(updateEditedPathPoint(double, double)));

    /// to link the map and the coordinates where we want to send the robot
    connect(view, SIGNAL(testCoord(double, double)), mainWindow, SLOT(testCoordSlot(double, double)));

    /// to create a path
    connect(view, SIGNAL(addPathPoint(QString, double, double)), this, SLOT(relayPathPoint(QString, double, double)));

    obstaclesPainter = new DrawObstacles(view->getSize(), _robots, view);
}

void MapController::setMapState(const GraphicItemState state){
    view->setState(state);
}

void MapController::updateMap(){
    view->update();
}

void MapController::modifyMap(const QByteArray mapArray, const bool fromPgm, const QString id, const QString date){
    map->setMapFromArray(mapArray, fromPgm);
    view->setPixmap(QPixmap::fromImage(map->getMapImage()));
    if(fromPgm && !id.isEmpty() && !date.isEmpty()){
        map->setMapId(QUuid(id));
        map->setDateTime(QDateTime::fromString(date, "yyyy-MM-dd-hh-mm-ss"));
    }
}

QColor MapController::getPixelColor(const double x, const double y){
    return map->getMapImage().pixelColor(x, y);
}

void MapController::updateMetadata(const int width, const int height, const float resolution, const float originX, const float originY){

    bool modif(false);

    if(width != map->getWidth()){
        map->setWidth(width);
        modif = true;
    }

    if(height != map->getHeight()){
        map->setHeight(height);
        modif = true;
    }

    if(resolution != map->getResolution()){
        map->setResolution(resolution);
        modif = true;
    }

    if(originX != map->getOrigin().getX() || originY != map->getOrigin().getY()){
        map->setOrigin(Position(originX, originY));
        modif = true;
    }

    if(modif){
        saveMapConfig((QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + QString::fromStdString(getMapFile())).toStdString());
        saveMapState();
    }
}

bool MapController::saveMapConfig(const std::string fileName){
    qDebug() << "MainWindow::saveMapConfig saving map to " << QString::fromStdString(fileName);
    std::ofstream file(fileName, std::ios::out | std::ios::trunc);
    if(file){
        file << map->getMapFile() << " " << std::endl <<
                map->getHeight() << " " << map->getWidth() << std::endl
             << map->getMapState().first.x() << " " << map->getMapState().first.y() << std::endl
             << map->getMapState().second << std::endl
             << map->getOrigin().getX() << " " << map->getOrigin().getY() << std::endl
             << map->getResolution() << std::endl
             << map->getMapId().toString().toStdString();
        file.close();
        return true;
    } else
        return false;
}

QImage MapController::getImageFromArray(const QByteArray array, const bool fromPgm){
    return map->getImageFromArray(array, fromPgm);
}

void MapController::saveMapState(){

    QFileInfo newMapInfo(QDir::currentPath(), "../gobot-software/currentMap.txt");
    std::ofstream file(newMapInfo.absoluteFilePath().toStdString(), std::ios::out | std::ios::trunc);

    map->setMapPosition(view->pos());
    map->setZoomCoeff(graphicsView->getZoomCoeff());

    /// saves the current configuration into the current configuration file
    if(file){
        file << map->getMapFile() << " " << std::endl <<
                map->getHeight() << " " << map->getWidth() << std::endl
             << map->getMapState().first.x() << " " << map->getMapState().first.y() << std::endl
             << map->getMapState().second << std::endl
             << map->getOrigin().getX() << " " << map->getOrigin().getY() << std::endl << map->getResolution() << std::endl
             << map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss").toStdString() << std::endl
             << map->getMapId().toString().toStdString();
        file.close();
    }
}

bool MapController::loadMapConfig(const std::string fileName){
    /// loads the configuration contained in the file <fileName> into the application
    std::ifstream file(fileName, std::ios::in);
    if(file){
        std::string _mapFile;
        int _height, _width;
        double centerX, centerY, originX, originY, resolution;
        QPair<QPointF, float> _mapState;
        std::string mapId;
        file >> _mapFile >> _height >> _width >> centerX >> centerY >> _mapState.second >> originX >> originY >> resolution >> mapId;
        qDebug() << "Loading map with config : \n\t" <<
                    "Height:" << _height << "\n\t" <<
                    "Width:" << _width << "\n\t" <<
                    "center X:" << centerX << "\n\t" <<
                    "center Y:" << centerY << "\n\t" <<
                    "zoom:" << _mapState.second << "\n\t" <<
                    "originX:" << originX << "\n\t" <<
                    "originY:" << originY << "\n\t" <<
                    "resolution:" << resolution << "\n\t" <<
                    "map ID:" << QString::fromStdString(mapId);
        map->setMapFile(_mapFile);
        map->setHeight(_height);
        map->setWidth(_width);
        map->setMapPosition(QPointF(centerX, centerY));
        map->setOrigin(Position(originX, originY));
        map->setResolution(resolution);
        map->setMapId(QUuid(QString::fromStdString(mapId)));
        map->setMapFromFile(QString::fromStdString(_mapFile));
        map->setDateTime(QDateTime::currentDateTime());
        file.close();
    } else
        return false;

    /// saves the configuration contained in the file <fileName> as the current configuration
    return saveMapConfig((QDir::currentPath() + QDir::separator() + "currentMap.txt").toStdString());
}

void MapController::updateMap(const std::string mapFile, const float resolution, const int width, const int height, const Position& origin, const QPixmap& pix, const QUuid id, const QDateTime date){
    Q_UNUSED(id)
    Q_UNUSED(date)
    map->setMapFile(mapFile);
    map->setResolution(resolution);
    map->setWidth(width);
    map->setHeight(height);
    map->setOrigin(origin);
    map->setMapId(QUuid::createUuid());
    map->setMapImage(pix.toImage());
    map->setDateTime(QDateTime::currentDateTime());
    modifyMap(pix);
}

void MapController::updateMap(const std::string mapFile, const float resolution, const int width, const int height, const Position& origin, const QImage& image, const QUuid id, const QDateTime date){
    Q_UNUSED(id)
    Q_UNUSED(date)
    map->setMapFile(mapFile);
    map->setResolution(resolution);
    map->setWidth(width);
    map->setHeight(height);
    map->setOrigin(origin);
    map->setMapImage(image);
    map->setMapId(QUuid::createUuid());
    map->setDateTime(QDateTime::currentDateTime());
    modifyMap(QPixmap::fromImage(image));
}

void MapController::relayPathPoint(QString name, double x, double y){
    (getPixelColor(x, y).red() >= 254) ? emit pathPointSignal(name, x, y) : emit newMessage("You cannot create a point here because your robot cannot go there. You must click known areas of the map");
}


