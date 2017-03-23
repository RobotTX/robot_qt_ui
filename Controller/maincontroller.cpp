#include "maincontroller.h"
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QVariant>
#include <QImage>
#include <QFileInfo>
#include <QDir>
#include <QMessageBox>
#include "Controller/Map/mapcontroller.h"
#include "Controller/Point/pointcontroller.h"
#include "Controller/Path/pathcontroller.h"
#include "Controller/Robot/robotscontroller.h"
#include "Model/Point/xmlparser.h"

MainController::MainController(QQmlApplicationEngine *engine, QObject* parent) : QObject(parent) {

    QList<QObject*> qmlList = engine->rootObjects();

    if(qmlList.size() == 1){
        /// The main parent element in the QML tree
        QObject *applicationWindow = qmlList.at(0);

        /// to allow the map model and the map view to communicate with each other
        /// and to ensure that they are consistent with each other
        mapController = QPointer<MapController>(new MapController(applicationWindow, this));

        /// to allow the point model and the point view to communicate with each other
        /// and to ensure that they are consistent with each other
        pointController = QPointer<PointController>(new PointController(applicationWindow, this));

        pathController = QPointer<PathController>(new PathController(applicationWindow, this));

        robotsController = QPointer<RobotsController>(new RobotsController(applicationWindow, this));

        connect(applicationWindow, SIGNAL(mapConfig(QString, double, double, double)), this, SLOT(saveMapConfig(QString, double, double, double)));
        connect(applicationWindow, SIGNAL(shortcutAddRobot()), robotsController, SLOT(shortcutAddRobot()));
        connect(applicationWindow, SIGNAL(shortcutDeleteRobot()), robotsController, SLOT(shortcutDeleteRobot()));

        QObject* mapMenuFrame = applicationWindow->findChild<QObject*>("mapMenuFrame");
        if(mapMenuFrame){
            connect(mapMenuFrame, SIGNAL(importMap(QString)), this, SLOT(loadMapConfig(QString)));
        } else {
            qDebug() << "MapController::MapController could not find the mapMenuFrame";
            Q_UNREACHABLE();
        }

    } else {
        qDebug() << "MainController::MainController We are supposed to only have 1 item, the ApplicationWindow";
        Q_UNREACHABLE();
    }
}

void MainController::checkPoint(QString name, QString oldName, double x, double y){
    /// When creating/editing a point we need the map to check if the point is on a wall or unknown place
    pointController->checkErrorPoint(mapController->getMapImage(), name, oldName, x, y);
}

void MainController::checkTmpPosition(int index, double x, double y){
    /// When creating/editing a path we need the map to check if the path point is on a wall or unknown place
    pathController->checkPosition(mapController->getMapImage(), index, x, y);
}

void MainController::saveMapConfig(QString fileName, double zoom, double centerX, double centerY) const {
    qDebug() << "MapController::saveMapConfig called with" << fileName << zoom << centerX << centerY;


    if(fileName.lastIndexOf(".pgm", fileName.length()-4) != -1){
        qDebug() << "filename" << fileName;
        fileName = fileName.mid(0, fileName.length()-4);
        qDebug() << "fileName after" << fileName;
    }

    QFileInfo mapFileInfo(static_cast<QDir> (fileName), "");

    QString filePath(QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");
    qDebug() << filePath;

    mapController->setMapFile(QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

    mapController->savePositionSlot(centerX, centerY, zoom, QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

    mapController->saveMapConfig(filePath.toStdString(), centerX, centerY, zoom);

    /// saves the new configuration to the map configuration file
    XMLParser::save(pointController, QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml");

    /// saves the new configuration to the current configuration file
    XMLParser::save(pointController, QDir::currentPath() + QDir::separator() + "currentPoints.xml");

    /// saves the map
    mapController->saveMapToFile(QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");
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

void MainController::loadMapConfig(QString fileName) const {
    qDebug() << "MainWindow::loadMapConfig called";

    if(!fileName.isEmpty()){
        QString fileNameWithoutExtension;
        if(fileName.indexOf(".pgm", fileName.length()-4) != -1)
            fileNameWithoutExtension = fileName.mid(0, fileName.length()-4);

        QFileInfo mapFileInfo(static_cast<QDir> (fileNameWithoutExtension), "");
        QString filePath(QDir::currentPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");
        qDebug() << "MainWindow::loadMapBtnEvent map to load :" << filePath;
        /// if we are able to find the configuration then we load the map
        if(mapController->loadMapConfig(filePath.toStdString())){
/*
            for(int i = 0; i < robotsController->getRobots()->getRobotsVector().size(); i++)
                robotsController->getRobots()->getRobotsVector().at(i)->getRobot()->sendNewMap(mapController->getMap());
*/
            /// clears the map of all paths and points
            // TODO need some kind of equivalent
            pointController->clearPoints();

/*
            clearNewMap();

            mapController->saveMapState();

            mapController->modifyMap();

            mapController->updateScene();
*/
            /// imports points associated to the map and save them in the current file
            qDebug() << " reading points from" << QDir::currentPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml";
            XMLParser::readPoints(pointController, QDir::currentPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml");
            //pointsController->loadPoints(fileNameWithoutExtension + "_points.xml");

            /// saves the new configuration to the current configuration file
            XMLParser::save(pointController, QDir::currentPath() + QDir::separator() + "points.xml");
            //pointsController->savePoints(QDir::currentPath() + QDir::separator() + "points.xml");

            /// updates the group box so that new points can be added
            //pointsController->getCreatePointWidget()->updateGroupBox(pointsController->getPoints());

            // TODO put this back when paths are implemented
            /*

            /// imports paths associated to the map and save them in the current file
            pathsController->deserializePaths(fileNameWithoutExtension + "_paths.dat");

            /// saves the imported paths in the current paths file
            pathsController->serializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");

            /// updates the groups of paths menu using the paths that have just been imported
            pathsController->updateGroupsPaths();
            */
        } else {
            QMessageBox warningBox;
            warningBox.setText("No configuration found for this map.");
            Q_UNREACHABLE();
            warningBox.setStandardButtons(QMessageBox::Ok);
            warningBox.setDefaultButton(QMessageBox::Ok);
            warningBox.exec();
        }
    }
}




















