#include "mapcontroller.h"
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QQmlContext>
#include <QQmlApplicationEngine>
#include <QAbstractListModel>
#include <fstream>
#include "Helper/helper.h"
#include "Controller/Map/editmapcontroller.h"
#include "Controller/Map/mergemapcontroller.h"
#include "Controller/maincontroller.h"
#include "Controller/Map/scanmapcontroller.h"
#include "View/editmappainteditem.h"

MapController::MapController(QQmlApplicationEngine* engine, QObject *applicationWindow, MainController *parent) : QObject(parent) {

    map = QPointer<Map>(new Map(this));

    QObject *mapViewFrame = applicationWindow->findChild<QObject*>("mapViewFrame");
    if (mapViewFrame){
        connect(this, SIGNAL(setMap(QVariant)), mapViewFrame, SLOT(setMap(QVariant)));
        connect(this, SIGNAL(setMapPosition(QVariant, QVariant, QVariant)), mapViewFrame, SLOT(setMapPosition(QVariant ,QVariant, QVariant)));
        connect(mapViewFrame, SIGNAL(savePosition(double, double, double, QString)), this, SLOT(savePositionSlot(double, double, double, QString)));
        connect(mapViewFrame, SIGNAL(loadPosition()), this, SLOT(loadPositionSlot()));
        connect(map, SIGNAL(mapFileChanged()), mapViewFrame, SLOT(mapFileChanged()));
        connect(mapViewFrame, SIGNAL(posClicked(double, double)), this, SLOT(posClicked(double, double)));
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

    mergeMapController = QPointer<MergeMapController>(new MergeMapController(parent, engine, applicationWindow));

    editMapController = QPointer<EditMapController>(new EditMapController(engine, applicationWindow, this));

    scanMapController = QPointer<ScanMapController>(new ScanMapController(parent, engine, applicationWindow));
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

        setMapFile(qMapFile);
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
                map->setResolution(resolution);
                map->setOrigin(QPointF(originX, originY));
                map->setDateTime(QDateTime::fromString(QString::fromStdString(_dateTime), "yyyy-MM-dd-hh-mm-ss"));
                map->setMapId(QUuid(QString::fromStdString(_mapId)));
                pathFile.close();
                emit setMapPosition(centerX, centerY, zoom);
            } else
                qDebug() << "Map::initializeMap could not find the map config file at :" << configPath;
        } else
            qDebug() << "Map::initializeMap could not find the map file at :" << qMapFile;
    } else
        qDebug() << "Map::initializeMap could not find the currentMap file at :" << currentPathFile;
}

void MapController::savePositionSlot(const double posX, const double posY, const double zoom, const QString mapSrc){

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

        file << mapSrc.toStdString() << std::endl
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
        emit setMapPosition(centerX, centerY, zoom);
    } else
        qDebug() << "Map::loadStateSlot could not find the currentMap file at :" << currentPathFile;
}

bool MapController::saveMapConfig(const QString fileName, const double centerX, const double centerY, const double zoom) const {
    qDebug() << "MainWindow::saveMapConfig saving map to " << fileName;
    std::ofstream file(fileName.toStdString(), std::ios::out | std::ios::trunc);
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
    qDebug() << "saving a map of size " << map->getMapImage().size();
    /// Qt has is own function to save the QImage to a PGM file
    map->getMapImage().save(fileName, "PGM");

    /// When the map is saved, no need to tell the user to save it again when closing the app
    map->setModified(false);
}

bool MapController::loadMapConfig(const QString fileName) {
    qDebug() << "MapController::loadMapConfig from" << fileName;
    /// loads the configuration contained in the file <fileName> into the application
    std::ifstream file(fileName.toStdString(), std::ios::in);
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

        setMapFile(QString::fromStdString(_mapFile));

        qDebug() << "requestloadmap" << "file:/" + QString::fromStdString(_mapFile);
        /// for the qml side to reload the main window map file
        emit requestReloadMap("file:/" + QString::fromStdString(_mapFile));
        map->setHeight(_height);
        map->setWidth(_width);
        map->setOrigin(QPointF(originX, originY));
        map->setResolution(resolution);
        map->setMapId(QUuid(QString::fromStdString(mapId)));
        map->setDateTime(QDateTime::currentDateTime());
        /// centers on (centerX, centerY) with the proper zoom coefficient
        centerMap(centerX, centerY, zoom);
        /// saves the configuration contained in the file <fileName> as the current configuration
        saveMapConfig(QDir::currentPath() + QDir::separator() + "currentMap.txt", centerX, centerY, zoom);
        file.close();
        return true;
    }
    return false;
}

void MapController::centerMap(const double centerX, const double centerY, const double zoom) {
    emit setMapPosition(centerX, centerY, zoom);
}

void MapController::saveEditedImage(const QString location){
    /// to save the image being edited in the edit map window
    editMapController->getPaintedItem()->saveImage(map->getMapImage(), location);
    /// and request the main map to be reload on the qml side
    emit requestReloadMap("file:/" + location);
}

/// helper function to print out the position where the map has been clicked
void MapController::posClicked(const double x, const double y){
    QPointF pos = Helper::Convert::pixelCoordToRobotCoord(
    QPointF(x, y),
    getOrigin().x(),
    getOrigin().y(),
    getResolution(),
    getHeight());
    qDebug() << "MapController::posClicked" << x << y << "to" << pos.x() << pos.y();
}

QImage MapController::getImageFromArray(const QByteArray& mapArrays, const int map_width, const int map_height, const bool fromPgm){

    qDebug() << "MapController::getImageFromArray" << map_width << map_height << fromPgm;
    QImage image = QImage(map_width, map_height, QImage::Format_Grayscale8);

    uint32_t index = 0;

    /// depending on where we get the map from, the system of coordinates is not the same
    /// so the formula is adjusted using <shift> and <sign>
    int shift = 0;
    int sign = 1;
    if(!fromPgm){
        shift = map_height-1;
        sign = -1;
    }

    QVector<int> countVector;
    int countSum = 0;

    qDebug() << "Shift and sign" << shift << sign;

    /// We set each pixel of the image
    for(int i = 0; i < mapArrays.size(); i += 5){
        int color = static_cast<int> (static_cast<uint8_t> (mapArrays.at(i)));

        uint32_t count = static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+1)) << 24) + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+2)) << 16)
                        + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+3)) << 8) + static_cast<uint32_t> (static_cast<uint8_t> (mapArrays.at(i+4)));

        countVector.push_back(count);
        countSum += count;

        for(int j = 0; j < static_cast<int> (count); j++){
            /// Sometimes we receive too much informations so we need to check
            if(index >= static_cast<uint>(map_width*map_height))
                return image;

            image.setPixelColor(QPoint(static_cast<int>(index%map_width), shift + sign * (static_cast<int>(index/map_width))), QColor(color, color, color));
            index++;
        }
    }

    return image;
}

void MapController::newMapFromRobot(const QByteArray& mapArray, const QString mapId, const QString mapDate){
    /// Convert the map from a byteArray to a QImage and save it
    map->setMapImage(getImageFromArray(mapArray, map->getWidth(), map->getHeight(), true));
    map->setMapId(QUuid(mapId));
    map->setDateTime(QDateTime::fromString(mapDate, "yyyy-MM-dd-hh-mm-ss"));
    map->getMapImage().save(QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.pgm", "PGM");
    setMapFile(QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.pgm");

    double centerX = 0;
    double centerY = 0;
    double zoom = 0;

    /// Save in currentMap.txt
    QFile file(QDir::currentPath() + QDir::separator() + "currentMap.txt");
    if(file.open(QFile::ReadWrite)){
        QTextStream stream(&file);
        QString osef;
        stream >> osef >> osef >> osef >> centerX >> centerY >> zoom >> osef >> osef >> osef >> osef >> osef;
        file.close();
    }

    QFile file2(QDir::currentPath() + QDir::separator() + "currentMap.txt");
    if(file2.open(QFile::WriteOnly|QFile::Truncate)){
        QTextStream stream(&file2);
        stream << map->getMapFile() << endl
             << map->getHeight() << " " << map->getWidth() << endl
             << centerX << " " << centerY << endl
             << zoom << endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << endl
             << map->getResolution() << endl
             << map->getMapId().toString();
        file.close();
        saveMapConfig(QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.config", centerX, centerY, zoom);
    }
}

void MapController::setMapFile(const QString file) {
    qDebug() << "MapController::setMapFile to" << file;
    map->setMapFile(file);
    QImage img(map->getMapFile(), "PGM");
    qDebug() << "imported a map of size " << img.size();
    map->setMapImage(QImage(map->getMapFile(), "PGM"));
    /// so that the qml side can load the map
    emit setMap(file);
}

QString MapController::getMetadataString(void) const {
    return QString::number(map->getWidth()) + ' ' + QString::number(map->getHeight()) +
            ' ' + QString::number(map->getResolution()) + ' ' + QString::number(map->getOrigin().x()) +
            ' ' + QString::number(map->getOrigin().y());
}

void MapController::saveNewMap(const QString file_name){
    map->getMapImage().save(file_name, "PGM");
    map->setMapFile(file_name);
    /// no need to ask the user to save the map again
    map->setModified(false);
    emit setMap(file_name);
}
