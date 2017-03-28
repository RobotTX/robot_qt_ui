#include "mapcontroller.h"
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QQmlContext>
#include <QQmlApplicationEngine>
#include <fstream>
#include <QAbstractListModel>
#include "editmapcontroller.h"
#include "View/editmappainteditem.h"
#include "mergemapcontroller.h"

MapController::MapController(QQmlApplicationEngine* engine, QObject *applicationWindow, QObject *parent) : QObject(parent){
    map = new Map(this);

    QObject *mapViewFrame = applicationWindow->findChild<QObject*>("mapViewFrame");
    if (mapViewFrame){
        connect(this, SIGNAL(setMap(QVariant)), mapViewFrame, SLOT(setMap(QVariant)));
        connect(this, SIGNAL(setMapPosition(QVariant, QVariant, QVariant)), mapViewFrame, SLOT(setMapPosition(QVariant ,QVariant, QVariant)));
        connect(mapViewFrame, SIGNAL(savePosition(double, double, double, QString)), this, SLOT(savePositionSlot(double, double, double, QString)));
        connect(mapViewFrame, SIGNAL(loadPosition()), this, SLOT(loadPositionSlot()));
        connect(map, SIGNAL(mapFileChanged()), mapViewFrame, SLOT(test()));
    } else {
        qDebug() << "MapController::MapController could not find the mapViewFrame";
        Q_UNREACHABLE();
    }

    QObject *mapMenuFrame = applicationWindow->findChild<QObject*>("mapMenuFrame");
    if (mapMenuFrame){
        connect(mapMenuFrame, SIGNAL(loadPosition()), this, SLOT(loadPositionSlot()));

    } else {
        qDebug() << "MapController::MapController could not find the mapMenuFrame";
        Q_UNREACHABLE();
    }

    connect(this, SIGNAL(requestReloadMap(QVariant)), applicationWindow, SLOT(reloadMapImage(QVariant)));

    initializeMap();

    editMapController = new EditMapController(engine, applicationWindow, this);

    mergeMapController = new MergeMapController(applicationWindow, this);

}

void MapController::initializeMap(void){
    QString currentPathFile = QDir::currentPath() + QDir::separator() + "currentMap.txt";
    qDebug() << currentPathFile;
    std::ifstream file(currentPathFile.toStdString(), std::ios::in);

    if(file){
        /// We get the path of the map to use so that we can deduce the path of its config file
        std::string _dateTime, osef, stdMapFile;
        double centerX, centerY, zoom;
        file >> stdMapFile >> osef >> osef >> centerX >> centerY >> zoom >> osef >> osef >> osef >> _dateTime >> osef;
        file.close();

        /// our map file as a QString
        QString qMapFile = QString::fromStdString(stdMapFile);

        map->setMapFile(qMapFile);
        qDebug() << "Map::initializeMap full map path :" << qMapFile;
        /// We get the config file from the map file
        QString fileName = qMapFile;
        fileName.remove(0, fileName.lastIndexOf(QDir::separator()) + 1);
        fileName.remove(fileName.length() - 4, 4);
        QString configPath = QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + fileName + ".config";

        if(QFile(qMapFile).exists()){
            qDebug() << "Map::initializeMap config path :" << configPath;
            /// We get the map informations from the map config file
            std::ifstream pathFile(configPath.toStdString(), std::ios::in);
            if(pathFile){
                double originX, originY, resolution;
                std::string _mapId;
                int height, width;
                pathFile >> osef >> height >> width >> osef >> osef >> osef >> originX >> originY >> resolution >> _mapId;
                qDebug() << "Map::initializeMap all info :" << map->getMapFile() << height << width
                         << centerX << centerY << originX << originY << resolution
                         << QString::fromStdString(_dateTime) << QString::fromStdString(_mapId);
                map->setHeight(height);
                map->setWidth(width);
                map->setMapImage(QImage(map->getMapFile()));
                map->setResolution(resolution);
                map->setOrigin(QPointF(originX, originY));
                map->setDateTime(QDateTime::fromString(QString::fromStdString(_dateTime), "yyyy-MM-dd-hh-mm-ss"));
                map->setId(QUuid(QString::fromStdString(_mapId)));
                pathFile.close();
                emit setMap(QVariant::fromValue(qMapFile));
                emit setMapPosition(QVariant::fromValue(centerX), QVariant::fromValue(centerY), QVariant::fromValue(zoom));
            } else
                qDebug() << "Map::initializeMap could not find the map config file at :" << configPath;
        } else
            qDebug() << "Map::initializeMap could not find the map file at :" << qMapFile;
    } else
        qDebug() << "Map::initializeMap could not find the currentMap file at :" << currentPathFile;
}

void MapController::savePositionSlot(double posX, double posY, double zoom, QString mapSrc){
    QString currentPathFile = QDir::currentPath() + QDir::separator() + "currentMap.txt";
    std::ofstream file(currentPathFile.toStdString(), std::ios::out | std::ios::trunc);

    /// saves the current configuration into the current configuration file
    if(file){
        qDebug() << "Map::savePositionSlot called with following parameters";
        qDebug() << "map file - height - width - centerX - centerY - zoom - originX - originY - resolution - date - id";
        qDebug() << mapSrc << map->getHeight() << map->getWidth() << posX << posY
                 << zoom << map->getOrigin().x() << map->getOrigin().y() << map->getResolution()
                 << map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss")
                 << map->getMapId().toString();

        file << mapSrc.toStdString() << " " << std::endl
             << map->getHeight() << " " << map->getWidth() << std::endl
             << posX << " " << posY << std::endl
             << zoom << std::endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << std::endl
             << map->getResolution() << std::endl
             << map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss").toStdString() << std::endl
             << map->getMapId().toString().toStdString();
        file.close();
    } else
        qDebug() << "Map::savePositionSlot could not find the currentMap file at :" << currentPathFile;
}

void MapController::loadPositionSlot(){
    qDebug() << "Map::loadStateSlot called";
    QString currentPathFile = QDir::currentPath() + QDir::separator() + "currentMap.txt";
    std::ifstream file(currentPathFile.toStdString(), std::ios::in);

    if(file){
        /// We get the path of the map to use so that we can deduce the path of its config file
        std::string osef;
        double centerX, centerY, zoom;
        file >> osef >> osef >> osef >> centerX >> centerY >> zoom;
        file.close();
        emit setMapPosition(QVariant::fromValue(centerX), QVariant::fromValue(centerY), QVariant::fromValue(zoom));
    } else
        qDebug() << "Map::loadStateSlot could not find the currentMap file at :" << currentPathFile;
}

bool MapController::saveMapConfig(const std::string fileName, const double centerX, const double centerY, const double zoom) const {
    qDebug() << "MainWindow::saveMapConfig saving map to " << QString::fromStdString(fileName);
    std::ofstream file(fileName, std::ios::out | std::ios::trunc);
    if(file){
        qDebug() << "saving map with file " << map->getMapFile();

        file << map->getMapFile().toStdString() << " " << std::endl <<
                map->getHeight() << " " << map->getWidth() << std::endl
             << centerX << " " << centerY << std::endl
             << zoom << std::endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << std::endl
             << map->getResolution() << std::endl
             << map->getMapId().toString().toStdString();

        file.close();
        return true;
    } else
        return false;
}

void MapController::saveMapToFile(const QString fileName) const {
    /// Qt has is own function to save the QImage to a PGM file
    map->getMapImage().save(fileName, "PGM");

    /// When the map is saved, no need to tell the user to save it again when closing the app
    map->setModified(false);
}

bool MapController::loadMapConfig(const std::string fileName) {
    qDebug() << "MapController::loadMapConfig from" << QString::fromStdString(fileName);
    /// loads the configuration contained in the file <fileName> into the application
    std::ifstream file(fileName, std::ios::in);
    if(file){
        std::string _mapFile;
        int _height, _width;
        double centerX, centerY, originX, originY, resolution, zoom;
        QPair<QPointF, float> _mapState;
        std::string mapId;
        file >> _mapFile >> _height >> _width >> centerX >> centerY >> zoom >> originX >> originY >> resolution >> mapId;
        qDebug() << "Loading map with config : \n\t" <<
                    "Height:" << _height << "\n\t" <<
                    "Width:" << _width << "\n\t" <<
                    "center X:" << centerX << "\n\t" <<
                    "center Y:" << centerY << "\n\t" <<
                    "zoom:" << zoom << "\n\t" <<
                    "originX:" << originX << "\n\t" <<
                    "originY:" << originY << "\n\t" <<
                    "resolution:" << resolution << "\n\t" <<
                    "map ID:" << QString::fromStdString(mapId);
        map->setMapFile(QString::fromStdString(_mapFile));
        qDebug() << "requestloadmap" << "file:/" + QString::fromStdString(_mapFile);
        emit requestReloadMap("file:/" + QString::fromStdString(_mapFile));
        map->setHeight(_height);
        map->setWidth(_width);
        map->setOrigin(QPointF(originX, originY));
        map->setResolution(resolution);
        map->setId(QUuid(QString::fromStdString(mapId)));
        map->setMapFile(QString::fromStdString(_mapFile));
        map->setDateTime(QDateTime::currentDateTime());
        centerMap(centerX, centerY, zoom);
        /// saves the configuration contained in the file <fileName> as the current configuration
        saveMapConfig(QString(QDir::currentPath() + QDir::separator() + "currentMap.txt").toStdString(), centerX, centerY, zoom);
        file.close();
        return true;
    }
    return false;
}

void MapController::centerMap(double centerX, double centerY, double zoom) {
    emit setMapPosition(centerX, centerY, zoom);
}

void MapController::saveEditedImage(QString location){
    editMapController->getPaintedItem()->saveImage(map->getMapImage(), location);
    emit requestReloadMap("file:/" + location);
    /*
    map->setMapFile(map->getMapFile());
    map->setMapImage(QImage(map->getMapFile()));
    */
}
