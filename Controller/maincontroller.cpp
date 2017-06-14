#include "maincontroller.h"
#include <QApplication>
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QVariant>
#include <QImage>
#include <QFileInfo>
#include <QDir>
#include <QThread>
#include <QMessageBox>
#include <QtMath>
#include "Helper/helper.h"
#include "Controller/Map/mapcontroller.h"
#include "Controller/Map/mergemapcontroller.h"
#include "Controller/Map/scanmapcontroller.h"
#include "Controller/Point/pointcontroller.h"
#include "Controller/Path/pathcontroller.h"
#include "Controller/Robot/robotscontroller.h"
#include "Controller/Map/scanmapcontroller.h"
#include "Model/Point/xmlparser.h"
#include "Model/Point/point.h"
#include "Model/Path/pathxmlparser.h"
#include "Model/Path/paths.h"
#include "Model/Path/path.h"
#include "Model/Path/pathgroup.h"
#include "Model/Path/pathpoint.h"

MainController::MainController(QQmlApplicationEngine *engine, QObject* parent) : QObject(parent), discardMap(true) {

    QList<QObject*> qmlList = engine->rootObjects();

    if(qmlList.size() == 1){
        /// The main parent element in the QML tree
        QObject *applicationWindow = qmlList.at(0);

        /// to allow the map model and the map view to communicate with each other
        /// and to ensure that they are consistent with each other
        mapController = QPointer<MapController>(new MapController(engine, applicationWindow, this));
        connect(this, SIGNAL(sendImageToMerge(QImage, double)), mapController->getMergeMapController(), SLOT(importMap(QImage, double)));

        /// to allow the point model and the point view to communicate with each other
        /// and to ensure that they are consistent with each other
        pointController = QPointer<PointController>(new PointController(applicationWindow, this));

        pathController = QPointer<PathController>(new PathController(applicationWindow, this));

        robotsController = QPointer<RobotsController>(new RobotsController(applicationWindow, engine, this));

        connect(applicationWindow, SIGNAL(mapConfig(QString, double, double, double, int)), this, SLOT(saveMapConfig(QString, double, double, double, int)));
        connect(applicationWindow, SIGNAL(shortcutAddRobot()), robotsController, SLOT(shortcutAddRobot()));
        connect(applicationWindow, SIGNAL(shortcutDeleteRobot()), robotsController, SLOT(shortcutDeleteRobot()));
        connect(applicationWindow, SIGNAL(test()), this, SLOT(testSlot()));
        connect(this, SIGNAL(openMapChoiceMessageDialog(QVariant, QVariant)), applicationWindow, SLOT(openMapChoiceMessageDialog(QVariant, QVariant)));
        connect(this, SIGNAL(openWarningDialog(QVariant, QVariant)), applicationWindow, SLOT(openWarningDialog(QVariant, QVariant)));
        connect(applicationWindow, SIGNAL(requestOrSendMap(QString, bool)), this, SLOT(requestOrSendMap(QString, bool)));
        connect(this, SIGNAL(emitBatteryThreshold(QVariant)), applicationWindow, SLOT(setBatteryThreshold(QVariant)));


        /// to initialize tutorial messages
        QObject* tuto = applicationWindow->findChild<QObject*>("tutorialModel");
        if(tuto){
            connect(this, SIGNAL(updateTutorialMessageVisibility(QVariant, QVariant)), tuto, SLOT(setVisibleMessage(QVariant, QVariant)));
            connect(tuto, SIGNAL(updateFile(int, bool)), this, SLOT(updateTutoFile(int, bool)));
        } else {
            /// NOTE can probably remove that when testing phase is over
            qDebug() << "could not find tuto model";
            Q_UNREACHABLE();
        }

        QObject* mapMenuFrame = applicationWindow->findChild<QObject*>("mapMenuFrame");
        if(mapMenuFrame){
            connect(mapMenuFrame, SIGNAL(importMap(QString)), this, SLOT(loadMapConfig(QString)));
        } else {
            /// NOTE can probably remove that when testing phase is over
            qDebug() << "MapController::MapController could not find the mapMenuFrame";
            Q_UNREACHABLE();
        }

        QObject* settings = applicationWindow->findChild<QObject*>("settings");
        if(settings){
            connect(this, SIGNAL(emitSettings(QVariant)), settings, SLOT(setSettings(QVariant)));
            connect(settings, SIGNAL(saveSettingsSignal(int, double)), this, SLOT(saveSettings(int, double)));
        } else {
            /// NOTE can probably remove that when testing phase is over
            qDebug() << "MapController::MapController could not find the settings";
            Q_UNREACHABLE();
        }

        QObject* scanWindow = applicationWindow->findChild<QObject*>("scanWindow");
        if(scanWindow){
            connect(this, SIGNAL(openRestartScanMessageDialog(QVariant)), scanWindow, SLOT(openRestartScanMessageDialog(QVariant)));
        } else {
            /// NOTE can probably remove that when testing phase is over
            qDebug() << "MapController::MapController could not find the scanWindow";
            Q_UNREACHABLE();
        }

        QObject* topView = applicationWindow->findChild<QObject*>("topView");
        if(settings){
            connect(this, SIGNAL(setMessageTop(QVariant, QVariant)), topView, SLOT(setMessageTop(QVariant, QVariant)));
        } else {
            /// NOTE can probably remove that when testing phase is over
            qDebug() << "MapController::MapController could not find the topView";
            Q_UNREACHABLE();
        }

        /// get settings from file
        QFile file(Helper::getAppPath() + QDir::separator() + "settings.txt");
        qDebug() << "App path :" << Helper::getAppPath();
        if(file.open(QFile::ReadWrite)){
            QTextStream in(&file);
                while (!in.atEnd()){
                    QString line = in.readLine();
                    int mapChoice(2);
                    double batteryThreshold(0.3);
                    QStringList list = line.split(' ');
                    if(list.size() == 2){
                        mapChoice = list.at(0).toInt();
                        if(list.at(1).toDouble() >= 0 && list.at(1).toDouble() <= 1)
                            batteryThreshold = list.at(1).toDouble();
                    } else {
                        /// TODO tell the user and rewrite the settings file as it has been corrupted
                        /// or let the user rewrite it the next time he goes in the settings menu, which will save them
                    }
                    emit emitSettings(mapChoice);
                    emit emitBatteryThreshold(batteryThreshold);
               }
               file.close();
        } else {
            qDebug() << "MainController::MainController could not find the settings file";
            /// TODO create the file/folder if it does not exist
            Q_UNREACHABLE();
        }
    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "MainController::MainController We are supposed to only have 1 item, the ApplicationWindow";
        Q_UNREACHABLE();
    }

    QFile tutoFile(Helper::getAppPath() + QDir::separator() + "tutorial.txt");
    if(tutoFile.exists() && tutoFile.open(QIODevice::ReadOnly)){
        QTextStream in(&tutoFile);
        int counter(0);

        while(!in.atEnd()){

            QString line = in.readLine();

            switch(counter++){

            /// careful to either cast the int to bool or to use == on the qml side instead of === which
            /// also looks at the type
            case 0: updateTutorialMessageVisibility("edit_map", static_cast<bool>(line.toInt()));
                break;

            case 1: updateTutorialMessageVisibility("merge_map", static_cast<bool>(line.toInt()));
                break;

            case 2: updateTutorialMessageVisibility("recover_position", static_cast<bool>(line.toInt()));
                break;

            case 3: updateTutorialMessageVisibility("scan_map", static_cast<bool>(line.toInt()));
                break;
            default:
                /// NOTE can probably remove that when testing phase is over, helpful during the test phase
                /// to make sure one message has not been left behind
                Q_UNREACHABLE();
                break;
            }
        }
        tutoFile.close();
    } else {
        tutoFile.open(QIODevice::ReadWrite);
        QTextStream out(&tutoFile);
        /// 1 for each message
        /// NOTE if a message was to be added, you would have to modify this as well as the code above
        out << '1' << '\n' << '1' << '\n' << '1' << '\n' << '1';
        tutoFile.close();
    }

    connect(this, SIGNAL(updateRobotPos(QString,double,double,double)), robotsController, SLOT(updateRobotPos(QString, double, double, double)));

}

void MainController::checkPoint(QString name, QString oldName, double x, double y){
    /// When creating/editing a point we need the map to check if the point is on a wall or unknown place
    pointController->checkErrorPoint(mapController->getMapImage(), name, oldName, x, y);
}

void MainController::checkTmpPosition(int index, double x, double y){
    /// When creating/editing a path we need the map to check if the path point is on a wall or unknown place
    pathController->checkPosition(mapController->getMapImage(), index, x, y);
}

void MainController::saveMapConfig(QString fileName, double zoom, double centerX, double centerY, int mapRotation, bool new_config) const {
    qDebug() << "MainController::saveMapConfig called with" << fileName << zoom << mapRotation << centerX << centerY;

    if(fileName.lastIndexOf(".pgm", fileName.length()-4) != -1){
        qDebug() << "filename" << fileName;
        fileName = fileName.mid(0, fileName.length()-4);
        qDebug() << "fileName after" << fileName;
    }

    QFileInfo mapFileInfo(static_cast<QDir> (fileName), "");
    QString filePath(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");

    if(!new_config){

        /// saves the image as a pgm file
        mapController->saveMapToFile(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

        mapController->savePositionSlot(centerX, centerY, zoom, mapRotation, Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

        mapController->saveMapConfig(filePath, centerX, centerY, zoom, mapRotation);

        /// saves the current points to the points file associated with the new configuration
        XMLParser::save(pointController, Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml");

        /// saves the new configuration to the current configuration file
        XMLParser::save(pointController, Helper::getAppPath() + QDir::separator() + "currentPoints.xml");

        /// saves the current points to the points file associated with the new configuration
        PathXMLParser::save(pathController, Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_paths.xml");

        /// saves the new configuration to the current configuration file
        PathXMLParser::save(pathController, Helper::getAppPath() + QDir::separator() + "currentPaths.xml");

    } else {

        mapController->savePositionSlot(centerX, centerY, zoom, mapRotation, Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

        mapController->saveMapConfig(filePath, 0, 0, 1.0, 0);

    }

    mapController->saveNewMap(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");
}

void MainController::loadMapConfig(QString fileName) {
    qDebug() << "MainController::loadMapConfig called with file" << fileName;

    if(!fileName.isEmpty()){
        QString fileNameWithoutExtension;
        if(fileName.indexOf(".pgm", fileName.length()-4) != -1)
            fileNameWithoutExtension = fileName.mid(0, fileName.length()-4);

        QFileInfo mapFileInfo(static_cast<QDir> (fileNameWithoutExtension), "");
        QString filePath(Helper::getAppPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");
        qDebug() << "MainController::loadMapBtnEvent map to load :" << filePath;

        /// if we are able to find the configuration then we load the map
        if(mapController->loadMapConfig(filePath)){
            robotsController->sendMapToAllRobots(mapController->getMapId().toString(),
                                                 mapController->getDateTime().toString("yyyy-MM-dd-hh-mm-ss"),
                                                 mapController->getMetadataString(),
                                                 mapController->getMapImage());

            /// clears the map of all paths and points
            pointController->clearPoints();

            pathController->clearPaths();

            /// imports points associated to the map and save them in the current file
            qDebug() << " reading points from" << Helper::getAppPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml";
            XMLParser::readPoints(pointController, Helper::getAppPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml");

            /// saves the new configuration to the current configuration file
            XMLParser::save(pointController, Helper::getAppPath() + QDir::separator() + "currentPoints.xml");

            PathXMLParser::readPaths(pathController, Helper::getAppPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_paths.xml");

            /// saves the imported paths in the current paths file
            PathXMLParser::save(pathController, Helper::getAppPath() + QDir::separator() + "currentPaths.xml");

            setMessageTopSlot(2, "Loaded the map: " + mapFileInfo.fileName());
        } else
            emit openWarningDialog("WARNING", "No configuration found for this map.\n\n\tPlease select another file.");
    }
}

void MainController::saveSettings(int mapChoice, double batteryThreshold){
    qDebug() << "save settings called" << mapChoice << batteryThreshold;
    QFile file(Helper::getAppPath() + QDir::separator() + "settings.txt");
    if(file.open(QFile::WriteOnly)){
        QTextStream stream(&file);
        stream << mapChoice << " " << batteryThreshold ;
        file.close();
        emit emitBatteryThreshold(batteryThreshold);
    }
    setMessageTopSlot(2, "Settings saved");
}

void MainController::newRobotPosSlot(QString ip, double posX, double posY, double ori){

    QPointF robotPos = Helper::Convert::robotCoordToPixelCoord(
                    QPointF(posX, posY),
                    mapController->getOrigin().x(),
                    mapController->getOrigin().y(),
                    mapController->getResolution(),
                    mapController->getHeight());
    double orientation = -ori * 180.0 / M_PI + 90;
    robotsController->setRobotPos(ip, robotPos.x(), robotPos.y(), orientation);
    //qDebug() << "maincontroller: update robot pos" << robotPos << posX << posY << mapController->getOrigin() << mapController->getResolution() << mapController->getHeight();
    emit updateRobotPos(ip, robotPos.x(), robotPos.y(), orientation);
    mapController->getScanMapController()->updateRobotPos(ip, robotPos.x(), robotPos.y(), orientation);
}

void MainController::updatePathSlot(QString ip, QStringList strList){
    if(strList.size() > 0){
        if(strList.size() % 4 == 1){
            emit setPath(ip, strList.takeFirst());
            for(int i = 0; i < strList.size(); i+=4){
                QPointF pathPointPos = Helper::Convert::robotCoordToPixelCoord(
                                QPointF(static_cast<QString>(strList.at(i+1)).toDouble(), static_cast<QString>(strList.at(i+2)).toDouble()),
                                mapController->getOrigin().x(),
                                mapController->getOrigin().y(),
                                mapController->getResolution(),
                                mapController->getHeight());
                emit addPathPoint(ip, strList.at(i), pathPointPos.x(), pathPointPos.y(), static_cast<QString>(strList.at(i+3)).toInt());
            }
        } else
            qDebug() << "RobotsController::updatePathSlot" << ip << "got a wrong number of param for the path :" << strList.size() << ", supposed to have the path name + a multiple of 4";
    }
}

void MainController::updateHomeSlot(QString ip, double homeX, double homeY, double homeOri){
    QPointF homePos = Helper::Convert::robotCoordToPixelCoord(
                    QPointF(homeX, homeY),
                    mapController->getOrigin().x(),
                    mapController->getOrigin().y(),
                    mapController->getResolution(),
                    mapController->getHeight());

    emit setHome(ip, homePos.x(), homePos.y(), homeOri);
}

void MainController::sendCommandNewHome(QString ip, double homeX, double homeY, int homeOri){
    qDebug() << "MainController::sendCommandNewHome";
    QPointF homePos = Helper::Convert::pixelCoordToRobotCoord(
    QPointF(homeX, homeY),
    mapController->getOrigin().x(),
    mapController->getOrigin().y(),
    mapController->getResolution(),
    mapController->getHeight());
    QString cmd = QString("n") + QChar(31) + QString::number(homePos.x()) + QChar(31) + QString::number(homePos.y()) + QChar(31) + QString::number(homeOri);
    robotsController->sendCommand(ip, cmd);
}

void MainController::sendCommandNewPath(QString ip, QString groupName, QString pathName){
    QVector<QPointer<PathPoint>> pathPointVector = pathController->getPath(groupName, pathName);
    QString pathStr("");
    for(int i = 0; i < pathPointVector.size(); i++){
        QPointF pathPointPos = Helper::Convert::pixelCoordToRobotCoord(
                        pathPointVector.at(i)->getPoint()->getPos(),
                        mapController->getOrigin().x(),
                        mapController->getOrigin().y(),
                        mapController->getResolution(),
                        mapController->getHeight());
        pathStr += QChar(31) + pathPointVector.at(i)->getPoint()->getName()
                + QChar(31) + QString::number(pathPointPos.x())
                + QChar(31) + QString::number(pathPointPos.y())
                + QChar(31) + QString::number(pathPointVector.at(i)->getWaitTime());
    }
    QString cmd = QString("i") + QChar(31) + pathName + pathStr;
    robotsController->sendCommand(ip, cmd);
}

void MainController::checkMapInfoSlot(QString ip, QString mapId, QString mapDate){

    /// Check if the robot has the current map
    qDebug() << "MainController::updateMapInfo Robot" << ip << "comparing ids" << mapId << "and" << mapController->getMapId();
    if(mapId.compare(mapController->getMapId().toString()) == 0){
        qDebug() << "MainController::updateMapInfo Robot" << ip << "has the current map";
    } else {
        qDebug() << "ids of app and robot " << mapController->getMapId().toString() << mapId;
        QDateTime mapDateTime = QDateTime::fromString(mapDate, "yyyy-MM-dd-hh-mm-ss");

        bool robotOlder = (mapDateTime <= mapController->getDateTime());
        if(robotOlder){
            qDebug() << "MainController::updateMapInfo Robot" << ip << "has a different and older map";
        } else {
            qDebug() << "MainController::updateMapInfo Robot" << ip << "has a different and newer map";
        }

        int mapChoice = -1;
        QFile file(Helper::getAppPath() + QDir::separator() + "settings.txt");
        if(file.open(QFile::ReadOnly)){
            QTextStream stream(&file);
            stream >> mapChoice;
            file.close();
        }
        qDebug() << "MainController::updateMapInfo Robot" << ip << "map choice :" << mapChoice;

        switch(mapChoice){
            case Helper::ALWAYS_NEW:
               if(robotOlder)
                   sendNewMap(ip);
               else
                   robotsController->requestMap(ip);
            break;
            case Helper::ALWAYS_OLD:
                if(robotOlder)
                    robotsController->requestMap(ip);
                else
                    sendNewMap(ip);
            break;
            case Helper::ALWAYS_ROBOT:
                robotsController->requestMap(ip);
            break;
            case Helper::ALWAYS_APPLICATION:
                sendNewMap(ip);
            break;
            default:
                qDebug() << "MainController::updateMapInfo ALWAYS_ASK choice";
                /// if there is no map on the application side, no need to ask, just take the robot's one
                if(mapController->getMapImage().size().width() != 0){
                    qDebug() << "MainController::checkMapInfoSlot There is a map on the application's side so the choice is offered to the user";
                    emit openMapChoiceMessageDialog(ip, robotOlder);
                } else {
                    qDebug() << "MainController::checkMapInfoSlot There is no map on the application side so we take the robot's one";
                    robotsController->requestMap(ip);
                }
            break;
        }
    }
}

void MainController::sendNewMap(QString ip){

    QString mapId = mapController->getMapId().toString();

    QString date = mapController->getDateTime().toString("yyyy-MM-dd-hh-mm-ss");

    qDebug() << mapController->getMetadataString() << mapController->getMapImage().size();

    QString mapMetadata = mapController->getMetadataString();

    robotsController->sendNewMap(ip, mapId, date, mapMetadata, mapController->getMapImage());
}

void MainController::newMapFromRobotSlot(QString ip, QByteArray mapArray, QString mapId, QString mapDate, QString resolution, QString originX, QString originY, int map_width, int map_height){
    mapController->updateMetadata(map_width, map_height, resolution.toDouble(), originX.toDouble(), originY.toDouble());
    mapController->newMapFromRobot(mapArray, mapId, mapDate);

    QString mapMetadata = mapController->getMetadataString();

    /// When we receive a map from a robot, we send it to all the other robots
    robotsController->sendNewMapToAllExcept(ip, mapId, mapDate, mapMetadata, mapController->getMapImage());

    pointController->clearPoints();
    XMLParser::save(pointController, Helper::getAppPath() + QDir::separator() + "currentPoints.xml");

    pathController->clearPaths();
    PathXMLParser::save(pathController, Helper::getAppPath() + QDir::separator() + "currentPaths.xml");
}

void MainController::requestOrSendMap(QString ip, bool request){
    if(request)
        robotsController->requestMap(ip);
    else
        robotsController->sendMapToAllRobots(mapController->getMapId().toString(),
                                             mapController->getDateTime().toString("yyyy-MM-dd-hh-mm-ss"),
                                             mapController->getMetadataString(),
                                             mapController->getMapImage());
}

void MainController::getMapFromRobot(QString ip){
    robotsController->requestMapForMerging(ip);
}

void MainController::processMapForMerge(QByteArray mapArray, QString resolution){
    qDebug() << "MainController::processMapForMerge" << resolution << mapController->getWidth() << mapController->getHeight();
    QImage image = mapController->getImageFromArray(mapArray, mapController->getWidth(), mapController->getHeight(), true);
    qDebug() << "Saving image of size" << image.size();
    emit sendImageToMerge(image, resolution.toDouble());
}

void MainController::resetMapConfiguration(QString file_name, bool scan, double centerX, double centerY){
    qDebug() << "MainController::resetMapConfiguration" << file_name;

    QString cpp_file_name = file_name.mid(7);
    /// this is the position of the robot at the end of the scan
    QPointF initPos(0.0f, 0.0f);
    /// orientation of the robot at the end of the scan
    double robotOri(0.0f);

    mapController->setMapFile(cpp_file_name);

    if(scan){

        ScanMapPaintedItem* map_reference = mapController->getScanMapController()->getPaintedItems().begin().value();
        qDebug() << "\n\n\nSaving map config after scan with origin" << map_reference->robotOrientation();

        /// have to compute the difference between the old origin and the position of the robot at the beginning of the scan,
        /// this is used as the new origin for the scanned map
        initPos = Helper::Convert::pixelCoordToRobotCoord(QPointF(map_reference->x() + map_reference->robotX(),
                                                                                 map_reference->y() + map_reference->robotY()),
                                                                         mapController->getOrigin().x(), mapController->getOrigin().y(), mapController->getResolution(),
                                                                         mapController->getHeight());
        /// Reset the orientation to reset the initial pose of the robot
        robotOri = map_reference->robotOrientation()-90;
    }
    pointController->clearPoints();
    pathController->clearPaths();

    /// to create a new configuration
    mapController->setMapId(QUuid::createUuid());
    mapController->setDateTime(QDateTime::currentDateTime());


    /// although this is a new configuraton we have to pass false in order to reset properly the paths and points
    /// in the files
    saveMapConfig(cpp_file_name, 1.0, 0, 0, false);

    /// we send this information to the robot, init pos is used to determine the position of the robot directly after gobot move
    /// is relaunched
    QString infoRobot = QString::number(mapController->getWidth()) + ' ' + QString::number(mapController->getHeight()) +
            ' ' + QString::number(mapController->getResolution()) + ' ' + QString::number(initPos.x()) +
            ' ' + QString::number(initPos.y()) + ' ' + QString::number(robotOri);;

    QImage img(cpp_file_name);

    //qDebug() << "sending map with metadata " << mapMetadata << " size of map is " << img.size();

    robotsController->sendMapToAllRobots(mapController->getMapId().toString(), mapController->getDateTime().toString("yyyy-MM-dd-hh-mm-ss"), infoRobot, img);
}

/************************* SCANNING *************************/

void MainController::startScanningSlot(QString ip){
    robotsController->sendCommand(ip, QString("t"));
}

void MainController::stopScanningSlot(QString ip, bool killGobotMove){
    robotsController->sendCommand(ip, QString("u") + QChar(31) + QString::number(killGobotMove));
}

void MainController::playPauseScanningSlot(QString ip, bool wasScanning, bool scanningOnConnection){
    if(wasScanning){
        /// Then we pause the scan
        robotsController->sendCommand(ip, QString("f"));
    } else {
        /// We pause the scan
        /// If the robot was already scanning, gmapping is launched so we just want to subscribe to get the map
        /// else the robot shut down while scanning and we need to relaunch gmapping to restart the scan
        /// so we ask if we want to relaunch gmapping and start the scan from the beggining or keep the previously scanned map
        if(!scanningOnConnection)
            emit openRestartScanMessageDialog(ip);
        else
            robotsController->sendCommand(ip, QString("e"));
    }
}

void MainController::receivedScanMapSlot(QString ip, QByteArray map, QString resolution, QString originX, QString originY, int map_width, int map_height){
    if(!discardMap){
        mapController->updateMetadata(map_width, map_height, resolution.toDouble(), originX.toDouble(), originY.toDouble());
        QImage image = mapController->getImageFromArray(map, mapController->getWidth(), mapController->getHeight(), false);
        mapController->getScanMapController()->receivedScanMap(ip, image, resolution);
    } else
        qDebug() << "Received a map to be discardeded";
}

void MainController::sendTeleopSlot(QString ip, int teleop){
    robotsController->sendTeleop(ip, teleop);
}

void MainController::removeScanMapSlot(QString ip){
    mapController->getScanMapController()->removeMap(ip);
}

void MainController::sendScanGoal(QString ip, double x, double y){
    QPointF goal_in_robot_coords(Helper::Convert::pixelCoordToRobotCoord(QPointF(x, y), mapController->getOrigin().x(), mapController->getOrigin().y(),
                                                                         mapController->getResolution(), mapController->getHeight()));
    qDebug() << "Sending command" << QString("c") + QChar(31) + QString::number(goal_in_robot_coords.x()) + QChar(31) + QString::number(goal_in_robot_coords.y());
    robotsController->sendCommand(ip, QString("c") + QChar(31) + QString::number(goal_in_robot_coords.x()) + QChar(31) + QString::number(goal_in_robot_coords.y()));
}

void MainController::setMessageTopSlot(int status, QString msg){
    emit setMessageTop(status, msg);
}

void MainController::updateTutoFile(int index, bool visible){
    QFile tutoFile(Helper::getAppPath() + QDir::separator() + "tutorial.txt");
    tutoFile.open(QIODevice::ReadWrite);
    int counter(0);
    QStringList list;
    QTextStream in(&tutoFile);
    while(!in.atEnd()){

        QString line = in.readLine();

        /// this is the value that needs to be replaced
        if(counter++ == index)
            line = QString::number(visible);
        list.append(line);
    }

    /// erases the old content
    tutoFile.resize(0);

    QTextStream out(&tutoFile);
    /// 1 for each message
    /// NOTE if a message was to be added, you would have to modify this as well as the code above
    for(int i = 0; i < list.size(); i++)
        out << list.at(i) << '\n';
    tutoFile.close();
}

void MainController::clearPointsAndPathsAfterScan(){
    /// clears the map of all paths and points
    pointController->clearPoints();
    pathController->clearPaths();
}

void MainController::sendMapToAllRobots(QString id, QString date, QString metadata, QImage img){
    robotsController->sendMapToAllRobots(id, date, metadata, img);
}


/**********************************************************************************************************/

void MainController::testSlot(){
    qDebug() << "MainController::testSlot called";
}
