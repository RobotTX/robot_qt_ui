#include "maincontroller.h"
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QVariant>
#include <QImage>
#include "Controller/Map/mapcontroller.h"
#include "Controller/Point/pointcontroller.h"
#include <QFileInfo>
#include <QDir>
#include "Model/Point/xmlparser.h"

MainController::MainController(QQmlApplicationEngine *engine, QObject* parent) : QObject(parent) {

    QList<QObject*> qmlList = engine->rootObjects();
    if(qmlList.size() == 1){
        /// The main parent element in the QML tree
        QObject *applicationWindow = qmlList.at(0);

        /// Map Controller
        mapController = new MapController(applicationWindow, this);

        /// Point Controller
        pointController = new PointController(applicationWindow, this);

        qDebug() << applicationWindow->objectName();

        connect(applicationWindow, SIGNAL(mapConfig(QString, double, double, double)), this, SLOT(saveMapConfig(QString, double, double, double)));

    } else {
        qDebug() << "MainController::MainController We are supposed to only have 1 item, the ApplicationWindow";
        Q_UNREACHABLE();
    }
}

void MainController::checkPoint(QString name, QString oldName, double x, double y){
    /// When creating/editing a point we need the map to chekc if the point is on a wall or unknown place
    pointController->checkErrorPoint(mapController->getMapImage(), name, oldName, x, y);
}

void MainController::saveMapConfig(QString fileName, double zoom, double centerX, double centerY) const {
    qDebug() << "MapController::saveMapConfig called with" << fileName << zoom << centerX << centerY;


    if(fileName.indexOf(".pgm", fileName.length()-4) != -1)
        fileName = fileName.mid(0, fileName.length()-4);

    //emit setMessageTop(TEXT_COLOR_INFO, "The current configuration of the map has been saved");

    QFileInfo mapFileInfo(static_cast<QDir> (fileName), "");
    QString filePath(QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");
    qDebug() << filePath;

    mapController->setMapFile(fileName + ".pgm");

    mapController->saveStateSlot(centerX, centerY, zoom, fileName);

    mapController->saveMapConfig(filePath.toStdString(), centerX, centerY, zoom);

    /// saves the new configuration to the map configuration file
    XMLParser::save(pointController, fileName + "_points.xml");

    /// saves the new configuration to the current configuration file
    XMLParser::save(pointController, QDir::currentPath() + QDir::separator() + "currentPoints.xml");

    /// saves the map
    mapController->saveMapToFile(fileName + ".pgm");
/*
    const QString pathsFile = fileName + "_paths.dat";

    /// saves the current configuration for the paths (this configuration will be associated to the map
    /// when you load the map in the future

    pathsController->serializePaths(pathsFile);

    // TODO add this when possible
    for(int i = 0; i < robotsController->getRobots()->getRobotsVector().size(); i++)
        robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->sendNewMap(mapController->getMap());
    */

}
