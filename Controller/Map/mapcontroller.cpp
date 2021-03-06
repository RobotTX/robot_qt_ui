#include "mapcontroller.h"
#include <QApplication>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QQmlContext>
#include <QQmlApplicationEngine>
#include <QAbstractListModel>
#include <QStandardPaths>
#include <fstream>
#include "Helper/helper.h"
#include "Controller/Map/editmapcontroller.h"
#include "Controller/maincontroller.h"
#include "Controller/Map/scanmapcontroller.h"
#include "View/EditMap/editmappainteditem.h"
#include "Controller/Robot/robotscontroller.h"

MapController::MapController(QQmlApplicationEngine* engine, QObject *applicationWindow, MainController *parent) : QObject(parent) {

    /// Create the mapConfig folder if it does not exist desktop
    if(!QDir(Helper::getAppPath() + QDir::separator() + "mapConfigs").exists())
        QDir().mkdir(Helper::getAppPath() + QDir::separator() + "mapConfigs");

    if(!QDir(Helper::getAppPath() + QDir::separator() + "data").exists())
        QDir().mkdir(Helper::getAppPath() + QDir::separator() + "data");

    /// Create Gobot folder, then mapConfigs folder if they do not exist android version
    /// Files will be located in Document location android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);

//    if(!QDir(location + QDir::separator() + "Gobot").exists())
//        QDir().mkdir(location + QDir::separator() + "Gobot");

//    QString gobotLocation = location + "/Gobot";

//    if(!QDir(gobotLocation+ QDir::separator() + "mapConfigs").exists())
//        QDir().mkdir(gobotLocation + QDir::separator() + "mapConfigs");

    map = QPointer<Map>(new Map(this));

    QObject *mapViewFrame = applicationWindow->findChild<QObject*>("mapViewFrame");
    if (mapViewFrame){
        connect(this, SIGNAL(setMap(QVariant)), mapViewFrame, SLOT(setMap(QVariant)));
        connect(this, SIGNAL(setMapPosition(QVariant, QVariant, QVariant, QVariant)), mapViewFrame, SLOT(setMapPosition(QVariant ,QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(setRotation(QVariant)), mapViewFrame, SLOT(setRotation(QVariant)));
        connect(this, SIGNAL(centerPosition(QVariant,QVariant)), mapViewFrame, SLOT(centerMap(QVariant, QVariant)));
        connect(this, SIGNAL(saveCenterMapPos()), mapViewFrame, SLOT(emitPosition()));
        connect(mapViewFrame, SIGNAL(savePosition(double, double, double, int, QString)), this, SLOT(savePositionSlot(double, double, double, int, QString)));
        connect(mapViewFrame, SIGNAL(loadPosition()), this, SLOT(loadPositionSlot()));
        connect(mapViewFrame, SIGNAL(posClicked(double, double)), this, SLOT(posClicked(double, double)));
        connect(mapViewFrame, SIGNAL(centerMapSignal()), this, SLOT(centerMapSlot()));
        connect(this, SIGNAL(requestReloadMap(QVariant)), mapViewFrame, SLOT(setMap(QVariant)));
    } else {
        /// NOTE can probably remove that when testing phase is over
//        // // qDebug() << "MapController::MapController could not find the mapViewFrame";
        Q_UNREACHABLE();
    }

    QObject *mapMenuFrame = applicationWindow->findChild<QObject*>("mapMenuFrame");
    if (mapMenuFrame){
        connect(mapMenuFrame, SIGNAL(loadPosition()), this, SLOT(loadPositionSlot()));
        connect(mapMenuFrame, SIGNAL(centerMap()), this, SLOT(centerMapSlot()));
    } else {
        /// NOTE can probably remove that when testing phase is over
//        // // qDebug() << "MapController::MapController could not find the mapMenuFrame";
        Q_UNREACHABLE();
    }

    connect(this, SIGNAL(sendMapToRobots(QString, QString, QString, QImage)), parent, SLOT(sendMapToAllRobots(QString, QString, QString, QImage)));

    initializeMap();

    editMapController = QPointer<EditMapController>(new EditMapController(engine, applicationWindow, this));

    scanMapController = QPointer<ScanMapController>(new ScanMapController(parent, engine, applicationWindow));
}

void MapController::initializeMap(void){

    /// for desktop version
    QString currentPathFile = Helper::getAppPath() + QDir::separator() + "data" + QDir::separator() + "currentMap.txt";
    std::ifstream file(currentPathFile.toStdString(), std::ios::in);

    /// for android version
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QString currentPathFile = location + QDir::separator() + "currentMap.txt";
//    std::ifstream file(currentPathFile.toStdString(), std::ios::in);

    if(file){
        /// We get the path of the map to use so that we can deduce the path of its config file
        std::string _dateTime, osef, stdMapFile;
        double centerX, centerY, zoom;
        int mapRotation;
        file >> stdMapFile >> osef >> osef >> centerX >> centerY >> zoom >> mapRotation >> osef >> osef >> osef >> osef >> _dateTime >> osef;
        file.close();

        /// our map file as a QString
        const QString qMapFile = QString::fromStdString(stdMapFile);

        if(setMapFile(qMapFile)){
//            // // qDebug() << "Map::initializeMap full map path :" << qMapFile;
            /// We get the config file from the map file
            QString fileName = qMapFile;
            fileName.remove(0, fileName.lastIndexOf(QDir::separator()) + 1);
            fileName.remove(fileName.length() - 4, 4);

            /// desktop
            QString configPath = Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + fileName + ".config";

            /// android
//            QString configPath = location + QDir::separator() + "mapConfigs" + QDir::separator() + fileName + ".config";

//            // // qDebug() << "Map::initializeMap config path :" << configPath;
            /// We get the map informations from the map config file
            std::ifstream pathFile(configPath.toStdString(), std::ios::in);
//            qDebug() << "pathFile = " << pathFile << " - configPath.toStdString() = " << configPath.toStdString();
            if(pathFile){
                double originX, originY, resolution;
                std::string _mapId;
                int height, width;
                pathFile >> osef >> width  >> height>> osef >> osef >> osef >> osef >> originX >> originY >> resolution >> _mapId;
//                // // qDebug() << "Map::initializeMap all info :" << map->getMapFile() << height << width
//                         << centerX << centerY << originX << originY << resolution
//                         << QString::fromStdString(_dateTime) << QString::fromStdString(_mapId);
                map->setHeight(height);
                map->setWidth(width);
                map->setResolution(resolution);
                map->setOrigin(QPointF(originX, originY));
                map->setDateTime(QDateTime::fromString(QString::fromStdString(_dateTime), "yyyy-MM-dd-hh-mm-ss"));
                map->setMapId(QUuid(QString::fromStdString(_mapId)));
                pathFile.close();
                emit setMapPosition(centerX, centerY, zoom, mapRotation);
            } else {}
                // // qDebug() << "Map::initializeMap could not find the map config file at :" << configPath;
        } else {}
            // // qDebug() << "Map::initializeMap could not find the map file at :" << qMapFile;
    } else {}
        // // qDebug() << "Map::initializeMap could not find the currentMap file at :" << currentPathFile;
}

void MapController::savePositionSlot(const double posX, const double posY, const double zoom, const int mapRotation, const QString mapSrc){

    /// desktop
    QString currentPathFile = Helper::getAppPath() + QDir::separator() + "data" + QDir::separator() + "currentMap.txt";
    QFileInfo mapFileInfo(static_cast<QDir> (mapSrc.mid(0, mapSrc.length()-4)), "");
    QString filePath(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");
    QString fileConfigPath(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QString currentPathFile = location + QDir::separator() + "currentMap.txt";
//    QFileInfo mapFileInfo(static_cast<QDir> (mapSrc.mid(0, mapSrc.length()-4)), "");
//    QString filePath(location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");
//    QString fileConfigPath(location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");


    std::ofstream configfile(fileConfigPath.toStdString(), std::ios::out | std::ios::trunc);
    /// saves the current configuration into the directed configuration file
    if(configfile){
//          qDebug() << "Map::savePositionSlot config file"<<fileConfigPath;

        configfile << filePath.toStdString() << std::endl
             << map->getWidth() << " " << map->getHeight() << std::endl
             << posX << " " << posY << std::endl
             << zoom << " " << mapRotation << std::endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << std::endl
             << map->getResolution() << std::endl
             << map->getMapId().toString().toStdString();
        configfile.close();
    }
    else {}
        // // qDebug() << "Map::savePositionSlot could not find the directed config file at :" << fileConfigPath;

    std::ofstream file(currentPathFile.toStdString(), std::ios::out | std::ios::trunc);
    /// saves the current configuration into the current configuration file
    if(file){
//        // // qDebug() << "Map::savePositionSlot called with following parameters";
//        // // qDebug() << "map file - width - height - centerX - centerY - zoom - originX - originY - resolution - date - id";
//        // // qDebug() << filePath << map->getWidth() << map->getHeight() << posX << posY
//                 << zoom << mapRotation << map->getOrigin().x() << map->getOrigin().y() <<  map->getResolution()
//                 << map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss")
//                 << map->getMapId().toString();

        file << filePath.toStdString() << std::endl
             << map->getWidth() << " " << map->getHeight() << std::endl
             << posX << " " << posY << std::endl
             << zoom << " " << mapRotation << std::endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << std::endl
             << map->getResolution() << std::endl
             << map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss").toStdString() << std::endl
             << map->getMapId().toString().toStdString();
        qDebug() << mapRotation;
        file.close();
    }
    else {}
        // // qDebug() << "Map::savePositionSlot could not find the currentMap file at :" << currentPathFile;
}

void MapController::savePositionSlot2(const double posX, const double posY, const double zoom, const int mapRotation, const QString mapSrc){

    /// destkop
    QFileInfo mapFileInfo(static_cast<QDir> (mapSrc.mid(0, mapSrc.length()-4)), "");
    QString filePath(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

    QString fileConfigPath(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QFileInfo mapFileInfo(static_cast<QDir> (mapSrc.mid(0, mapSrc.length()-4)), "");
//    QString filePath(location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");
//    QString fileConfigPath(location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");

    std::ofstream configfile(fileConfigPath.toStdString(), std::ios::out | std::ios::trunc);
    /// saves the current configuration into the directed configuration file
    if(configfile){
//        // // qDebug() << "Map::savePositionSlot config file"<<fileConfigPath;
        configfile << filePath.toStdString() << std::endl
             << map->getWidth() << " " << map->getHeight() << std::endl
             << posX << " " << posY << std::endl
             << zoom << " " << mapRotation << std::endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << std::endl
             << map->getResolution() << std::endl
             << map->getMapId().toString().toStdString();
        configfile.close();
    }
    else {}
        // // qDebug() << "Map::savePositionSlot could not find the directed config file at :" << fileConfigPath;
}

void MapController::loadPositionSlot(){

    /// desktop
    QString currentPathFile = Helper::getAppPath() + QDir::separator() + "data" + QDir::separator() + "currentMap.txt";
    std::ifstream file(currentPathFile.toStdString(), std::ios::in);

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QString currentPathFile = location + QDir::separator() + "currentMap.txt";
//    std::ifstream file(currentPathFile.toStdString(), std::ios::in);

    if(file){
        /// to store the values we don't need
        std::string osef;
        double centerX, centerY, zoom;
        int mapRotation;
        file >> osef >> osef >> osef >> centerX >> centerY >> zoom >> mapRotation;
        file.close();
        qDebug() << "mapRotation in loadPositionSlot " << mapRotation;
        emit setMapPosition(centerX, centerY, zoom, mapRotation);
    } else {}
        // // qDebug() << "Map::loadStateSlot could not find the currentMap file at :" << currentPathFile;
}

bool MapController::saveMapConfig(const QString fileName, const double centerX, const double centerY, const double zoom, const int mapRotation) const {
      qDebug() << "MapController::saveMapConfig saving map to " << fileName;
    std::ofstream file(fileName.toStdString(), std::ios::out | std::ios::trunc);
    if(file){
          qDebug() << "saving map with file " << map->getMapFile();

        file << map->getMapFile().toStdString() << " " << std::endl
             << map->getWidth() << " " << map->getHeight() << std::endl
             << centerX << " " << centerY << std::endl
             << zoom << " " << mapRotation << std::endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << std::endl
             << map->getResolution() << std::endl
             << map->getMapId().toString().toStdString();

        file.close();
        return true;
    } else {
          qDebug() << "MaoController::saveMapConfig - no file";
        return false;
    }
}

void MapController::saveMapToFile(const QString fileName) const {
//    // // qDebug() << "saving a map of size " << map->getMapImage().size();
    /// Qt has is own function to save the QImage to a PGM file
    map->getMapImage().save(fileName, "PGM");
}

bool MapController::loadMapConfig(const QString fileName) {
//    // // qDebug() << "MapController::loadMapConfig from" << fileName;
    /// loads the configuration contained in the file <fileName> into the application
    std::ifstream file(fileName.toStdString(), std::ios::in);
    if(file){
        std::string _mapFile;
        int _height, _width, mapRotation;
        double centerX, centerY, originX, originY, resolution, zoom;
        QPair<QPointF, double> _mapState;
        std::string mapId;
        file >> _mapFile >> _width >> _height >> centerX >> centerY >> zoom >> mapRotation >> originX >> originY >> resolution >> mapId;
//        // // qDebug() << "Loading map with config : \n\t" <<
//                    "Width:" << _width << "\n\t" <<
//                    "Height:" << _height << "\n\t" <<
//                    "center X:" << centerX << "\n\t" <<
//                    "center Y:" << centerY << "\n\t" <<
//                    "zoom:" << zoom << "\n\t" <<
//                    "mapRotation:" << mapRotation << "\n\t" <<
//                    "originX:" << originX << "\n\t" <<
//                    "originY:" << originY << "\n\t" <<
//                    "resolution:" << resolution << "\n\t" <<
//                    "map ID:" << QString::fromStdString(mapId);

        if(setMapFile(QString::fromStdString(_mapFile))){
//            // // qDebug() << "requestloadmap" << "file:/" + QString::fromStdString(_mapFile);
            /// for the qml side to reload the main window map file
//            emit requestReloadMap("file:/" + QString::fromStdString(_mapFile));
            emit requestReloadMap(QString::fromStdString(_mapFile));
            map->setWidth(_width);
            map->setHeight(_height);
            map->setOrigin(QPointF(originX, originY));
            map->setResolution(resolution);
            map->setMapId(QUuid(QString::fromStdString(mapId)));
            map->setDateTime(QDateTime::currentDateTime());
            /// centers on (centerX, centerY) with the proper zoom coefficient
            centerMap(centerX, centerY, zoom, mapRotation);

            /// saves the configuration contained in the file <fileName> as the current configuration desktop
            saveMapConfig(Helper::getAppPath() + QDir::separator() + "data" + QDir::separator() + "currentMap.txt", centerX, centerY, zoom, mapRotation); /// desktop
            qDebug() << "mapRotation in loadMapConfig" << mapRotation;

            /// android
//            QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//            saveMapConfig(location + QDir::separator() + "currentMap.txt", centerX, centerY, zoom, mapRotation);

            file.close();
            return true;
        }
    }
    return false;
}

void MapController::centerMap(const double centerX, const double centerY, const double zoom, const int mapRotation) {
    emit setMapPosition(centerX, centerY, zoom, mapRotation);
}

void MapController::saveEditedImage(const QString locationD, int mapRotation) {
    /// modifies the map id so that when a robot reconnects you get asked which map to choose
    map->setMapId(QUuid::createUuid());
    double centerX = 0;
    double centerY = 0;
    double zoom = 0;

    emit setRotation(mapRotation);

//    emit setMapPosition(centerX, centerY, zoom, mapRotation);

    /// desktop
    QFile file(Helper::getAppPath() + QDir::separator() + "data" + QDir::separator() + "currentMap.txt");

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QFile file(location + QDir::separator() + "currentMap.txt");

    int mapRot = mapRotation;

    if(file.open(QFile::ReadWrite)){
        QTextStream stream(&file);
        QString osef;
        stream >> osef >> osef >> osef >> centerX >> centerY >> zoom >> mapRotation >> osef >> osef >> osef >> osef >> osef;
        file.close();
    }

    /// desktop
    QFile file2(Helper::getAppPath() + QDir::separator() + "data" + QDir::separator() + + "currentMap.txt");

    /// android
//    QFile file2(location + QDir::separator() + "currentMap.txt");
    if(file2.open(QFile::WriteOnly|QFile::Truncate)){
        QTextStream stream(&file2);
        stream << map->getMapFile() << endl
             << map->getWidth() << " " << map->getHeight() << endl
             << centerX << " " << centerY << endl
             << zoom << " " << mapRot << endl
             << map->getOrigin().x() << " " << map->getOrigin().y() << endl
             << map->getResolution() << endl
             << map->getMapId().toString();
        file.close();
    }

    saveMapConfig(map->getMapFile().mid(0, map->getMapFile().size()-4) + ".config", centerX, centerY, zoom, mapRotation);

    /// to save the image being edited in the edit map window
    editMapController->getPaintedItem()->saveImage(map->getMapImage(), locationD);

    setMapFile(locationD);
    /// and request the main map to be reload on the qml side
//    emit requestReloadMap("file:/" + locationD);
    #if defined(Q_OS_WIN)
        emit requestReloadMap("C" + locationD);
    #endif

    #if defined(Q_OS_LINUX)
        emit requestReloadMap("file:/" + locationD);
    #endif

    #if defined(Q_OS_MAC)
        emit requestReloadMap("file:/" + locationD);
    #endif

    emit sendMapToRobots("EDIT"+map->getMapId().toString(), map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss"), getMetadataString(), QImage(map->getMapFile()));
}

/// helper function to print out the position where the map has been clicked
/// NOTE to remove when dev phase is over
void MapController::posClicked(const double x, const double y){
    QPointF pos = Helper::Convert::pixelCoordToRobotCoord(
    QPointF(x, y),
    getOrigin().x(),
    getOrigin().y(),
    getResolution(),
    getHeight());
//    // // qDebug() << "MapController::posClicked" << x << y << "to" << pos.x() << pos.y() << "\nMetadata" << getOrigin() << getWidth() << getHeight() << getResolution();
}

QImage MapController::getImageFromArray(const QByteArray& mapArrays, const int map_width, const int map_height, const bool fromPgm){

//    // // qDebug() << "MapController::getImageFromArray" << map_width << map_height << fromPgm;
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

//    // // qDebug() << "Shift and sign" << shift << sign;

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

//    // // qDebug() << "countSum" << countSum << index << sign;

    return image;
}

void MapController::newMapFromRobot(const QByteArray& mapArray, const QString mapId, const QString mapDate){
    /// Convert the map from a byteArray to a QImage and save it
    map->setMapImage(getImageFromArray(mapArray, map->getWidth(), map->getHeight(), true));
    QUuid mapId_quuid = QUuid(mapId);
    if(mapId_quuid == QUuid()){
        /// NOTE check that gobot_move is launched when launching gobot_software ??
//        // // qDebug() << "The robot gave us a null or invalid QUUID, did you launch gobot_move ??";
        Q_UNREACHABLE();
    }

    map->setMapId(mapId_quuid);
    map->setDateTime(QDateTime::fromString(mapDate, "yyyy-MM-dd-hh-mm-ss"));

    /// desktop
//    // // qDebug() << Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.pgm";
    map->getMapImage().save(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.pgm", "PGM");

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    map->getMapImage().save(location + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.pgm", "PGM");

    if(setMapFile(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.pgm")){ /// desktop
//    if(setMapFile(location + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.pgm")){ /// android
        double centerX = 0;
        double centerY = 0;
        double zoom = 0;
        int mapRotation = 0;

        /// Save in currentMap.txt
        QFile file(Helper::getAppPath() + QDir::separator() + "data" + QDir::separator() + "currentMap.txt"); /// desktop
//        QFile file(location + QDir::separator() + "currentMap.txt"); /// android
        if(file.open(QFile::ReadWrite)){
            QTextStream stream(&file);
            QString osef;
            qDebug() << "mapRotation in newMapFromRobot " << mapRotation;
            stream >> osef >> osef >> osef >> centerX >> centerY >> zoom >> mapRotation >> osef >> osef >> osef >> osef >> osef;
            file.close();
        }

        QFile file2(Helper::getAppPath() + QDir::separator() + "data" + QDir::separator() + "currentMap.txt"); /// desktop
//        QFile file2(location + QDir::separator() + "currentMap.txt"); /// android
        if(file2.open(QFile::WriteOnly|QFile::Truncate)){
            QTextStream stream(&file2);
            stream << map->getMapFile() << endl
                 << map->getWidth() << " " << map->getHeight() << endl
                 << centerX << " " << centerY << endl
                 << zoom << " " << mapRotation << endl
                 << map->getOrigin().x() << " " << map->getOrigin().y() << endl
                 << map->getResolution() << endl
                 << map->getMapId().toString();
            file.close();
            saveMapConfig(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.config", centerX, centerY, zoom, mapRotation); /// desktop
//             saveMapConfig(location + QDir::separator() + "mapConfigs" + QDir::separator() + "tmpImage.config", centerX, centerY, zoom, mapRotation); /// android
        }
    }
}

bool MapController::setMapFile(const QString file) {
      qDebug() << "file in setMapFile = " << file;
    if(QFile(file).exists()){
//          qDebug() << "MapController::setMapFile to" << file;
        map->setMapFile(file);
//        QImage img(map->getMapFile(), "PGM");
//        // // qDebug() << "imported a map of size " << img.size();
        map->setMapImage(QImage(map->getMapFile(), "PGM"));
        qDebug() << map->getMapFile();
        /// so that the qml side can load the map
        emit setMap(file); /// qml side
        return true;
    } else {
          qDebug() << "MapController::setMapFile -> file = "<< file << "does not exist";
        return false;
    }
}

QString MapController::getMetadataString(void) const {
    /// We send -150 as initial position so we know we don't use it in readnewmap.cpp on the robot
    /// so we'll use the robot home position instead
    return QString::number(map->getWidth()) + ' ' + QString::number(map->getHeight()) +
            ' ' + QString::number(map->getResolution()) + QString::number(map->getOrigin().x()) + ' ' + QString::number(map->getOrigin().y());
}

void MapController::saveNewMap(const QString file_name){
      qDebug() << "MapController::saveNewMap " << file_name;
    map->setMapFile(file_name);
    emit setMap(file_name);
}

void MapController::updateMetadata(int width, int height, double resolution, double originX, double originY){
//    // // qDebug() << "MapController::updateMetadata width height resolution originX originY" << width << height << resolution << originX << originY;
    setOrigin(QPointF(originX, originY));
    setWidth(width);
    setHeight(height);
    setResolution(resolution);
}

void MapController::centerMapSlot(){
    QImage image = map->getMapImage();

    int top = 0;
    int bottom = image.height();
    int left = image.width();
    int right = 0;

    /// We want to find the center of the rectangle constructed by getting rid of all the extra grey
    /// around the image, this is the position we want to center on
    for(int i = 0; i < image.width(); i++){
        for(int j = 0; j < image.height(); j++){
            int color = image.pixelColor(i, j).red();
            if(color == 255 || color == 0){
                if(bottom > j)
                    bottom = j;
                if(top < j)
                    top = j;
                if(left > i)
                    left = i;
                if(right < i)
                    right = i;
            }
        }
    }
    emit centerPosition(left + (right-left)/2, top + (bottom - top)/2);
}

void MapController::saveCenterMapPosSlot(){
    emit saveCenterMapPos();
}
