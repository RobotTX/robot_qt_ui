#include "maincontroller.h"
#include <QApplication>
#include <QQmlProperty>
#include <QQmlApplicationEngine>
#include <QStandardPaths>
#include <QDebug>
#include <QVariant>
#include <QImage>
#include <QFileInfo>
#include <QDir>
#include <QThread>
#include <QMessageBox>
#include <QtMath>
#include <QString>
#include <QFileDialog>
#include <QProcess>
#include <QDebug>
#include "Helper/helper.h"
#include "Controller/Map/mapcontroller.h"
#include "Controller/Map/scanmapcontroller.h"
#include "Controller/Point/pointcontroller.h"
#include "Controller/Speech/speechcontroller.h"
#include "Controller/Path/pathcontroller.h"
#include "Controller/Robot/robotscontroller.h"
#include "Controller/Map/scanmapcontroller.h"
#include "Controller/Robot/robotcontroller.h"
#include "Controller/Robot/cmdrobotworker.h"
#include "Model/Point/xmlparser.h"
#include "Model/Point/point.h"
#include "Model/Speech/speechxmlparser.h"
#include "Model/Speech/speech.h"
#include "Model/Path/pathxmlparser.h"
#include "Model/Path/paths.h"
#include "Model/Path/path.h"
#include "Model/Path/pathgroup.h"
#include "Model/Path/pathpoint.h"

MainController::MainController(QQmlApplicationEngine *engine, QObject* parent) : QObject(parent), discardMap(true) {

    QList<QObject*> qmlList = engine->rootObjects();

    if(qmlList.size() >= 1){
        /// The main parent element in the QML tree
        QObject *applicationWindow = qmlList.at(0);

        /// to allow the map model and the map view to communicate with each other
        /// and to ensure that they are consistent with each other
        mapController = QPointer<MapController>(new MapController(engine, applicationWindow, this));

        /// to allow the point model and the point view to communicate with each other
        /// and to ensure that they are consistent with each other
        pointController = QPointer<PointController>(new PointController(applicationWindow, this));

        speechController = QPointer<SpeechController>(new SpeechController(applicationWindow, this));

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

        connect(this, SIGNAL(openScanWindowForAutomaticScan(QVariant)), applicationWindow, SLOT(openScanWindowForAutomaticScan(QVariant)));

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
            connect(this, SIGNAL(emitWifiList(QVariant, QVariant)), settings, SLOT(getWifiList(QVariant, QVariant)));
            connect(this, SIGNAL(emitSizeWifiList(QVariant)), settings, SLOT(getSizeWifiList(QVariant)));
            connect(settings, SIGNAL(saveSettingsSignal(int, double)), this, SLOT(saveSettings(int, double)));
            connect(settings, SIGNAL(saveWifiSignal(QString, QString, QString)), this, SLOT(saveWifi(QString, QString, QString)));
            connect(settings, SIGNAL(saveVelocitySignal(QString, double, double)), this, SLOT(saveVelocity(QString, double, double)));
            connect(settings, SIGNAL(saveBatterySignal(QString, double)), this, SLOT(saveBattery(QString, double)));
            connect(settings, SIGNAL(changeLanguage(QString)), this, SLOT(changeLanguage(QString)));

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

//        QObject* deconnexionBtn = applicationWindow->findChild<QObject*>("deconnexionBtn");
//        if(deconnexionBtn){
//            connect(deconnexionBtn, SIGNAL(deconnexion()), this, SLOT(deconnexionSlot()));
//        } else {
//            /// NOTE can probably remove that when testing phase is over
//            qDebug() << "Authentification::Authentification could not find the deconnexionBtn";
//            Q_UNREACHABLE();
//        }

        /// script shell script
        /// desktop
        #if defined(Q_OS_LINUX)

            QString cmd = QString(" > " + Helper::getAppPath() + "/wifi.txt && nmcli -t -f ssid dev wifi >> " + Helper::getAppPath() + "/wifi.txt");
            QProcess process;
            process.startDetached("sh", QStringList() << "-c" << cmd);
            process.waitForFinished();
            QFile ssid(Helper::getAppPath() + QDir::separator() + "wifi.txt");
            int line_count = 0;
            if (ssid.open(QFile::ReadWrite)) {
                QTextStream ssidIn(&ssid);
                while (!ssidIn.atEnd()) {
                    QString line = ssidIn.readLine();
                    line_count++;
                    emitWifiList(line, line_count);
                }
                qDebug() << "count wifi = " << line_count;
                emitSizeWifiList(line_count);
            }

            ssid.close();
        #endif

        /// get wifi android
//        QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//        QFile ssid(location + QDir::separator() + "wifi.txt");
//        QString cmd1 = QString("service call wifi 11 > " + location + "/wifi.txt ");
//        QString cmd = QString("> " + location + "/wifi.txt service call wifi 13 >>" + location +"/wifi.txt");

        /// get wifi Windows
        #if defined(Q_OS_WIN)
            QString cmd = QString("> " + Helper::getAppPath() + QDir::separator() + "wifi.txt netsh wlan show network >> " + Helper::getAppPath() + QDir::separator() + "wifi.txt");
            QProcess process;
            process.startDetached("cmd", QStringList() << "/c" << cmd);
            QFile ssid(Helper::getAppPath() + QDir::separator() + "wifi.txt");
            int line_count = 0;
            if (ssid.open(QFile::ReadWrite)) {
                QTextStream ssidIn(&ssid);
                while (!ssidIn.atEnd()) {
                    QString line = ssidIn.readLine();
                    if (line.startsWith("SSID")) {
                        QStringList parse = line.split(": ");
                        QString wifiName = parse.last();
                        line_count++;
                        emitWifiList(wifiName, line_count);
                    } else {
                    }
                }
                emitSizeWifiList(line_count);
            }
            ssid.close();
        #endif

        /// get wifi MacOs
        #if defined(Q_OS_MAC)
            QString cmd = QString("> " + Helper::getAppPath() + QDir::separator() + "wifi.txt && /System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -s | awk '{print $1}' >>" + Helper::getAppPath() + "/wifi.txt");
            qDebug() << cmd;
            QProcess process;
            process.startDetached("sh", QStringList() << "-c" << cmd);
            process.waitForFinished();
            QFile ssid(Helper::getAppPath() + QDir::separator() + "wifi.txt");
            int line_count = 0;
            if (ssid.open(QFile::ReadWrite)) {
                QTextStream ssidIn(&ssid);
                while (!ssidIn.atEnd()) {
                    QString line = ssidIn.readLine();
                    if (line.contains("SSID")) {
                    } else {
                        qDebug() << "line = " << line;
                        line_count++;
                        emitWifiList(line, line_count);
                    }
                }
                qDebug() << "count = " << line_count;
                emitSizeWifiList(line_count);
            }
            ssid.close();
        #endif

        /// get settings from file
        /// desktop
        QFile file(Helper::getAppPath() + QDir::separator() + "settings.txt");

        /// android
//        QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//        QFile file(location + QDir::separator() + "settings.txt");
//        qDebug() << "App path :" << Helper::getAppPath();
        if(file.open(QFile::ReadWrite)){
            QTextStream in(&file);
            /// the file is empty for example because it did not exist so we just
            /// created it
            if(in.atEnd()){
                QTextStream stream(&file);
                /// if the file was corrupted we put as default the mode
                /// "always ask" as for the choice of the map and 10% for
                /// the battery
                stream << 2 << " " << 0.0;
                emit emitSettings(2);
                emit emitBatteryThreshold(0.0);
            } else {
                while (!in.atEnd()){
                    QString line = in.readLine();
                    int mapChoice(2);
                    double batteryThreshold(0.3);
                    QStringList list = line.split(' ');
                    qDebug() << "settings " << list;
                    if(list.size() == 2){
                        mapChoice = list.at(0).toInt();
                        if(list.at(1).toDouble() >= 0 && list.at(1).toDouble() <= 1)
                            batteryThreshold = list.at(1).toDouble();
                    } else {
                        file.resize(0);
                        QTextStream stream(&file);
                        /// if the file was corrupted we put as default the mode
                        /// "always ask" as for the choice of the map and 10% for
                        /// the battery
                        stream << 2 << " " << 0.0;
                    }
                    emit emitSettings(mapChoice);
                    emit emitBatteryThreshold(batteryThreshold);
                }
                file.close();
            }
        }
    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "MainController::MainController We are supposed to only have 1 item, the ApplicationWindow";
        Q_UNREACHABLE();
    }

    QFile tutoFile(Helper::getAppPath() + QDir::separator() + "tutorial.txt"); /// desktop
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QFile tutoFile(location + QDir::separator() + "tutorial.txt");
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

            case 1: qDebug("ok");
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

void MainController::deconnexionSlot() {
    emit deco();
}

void MainController::checkPoint(QString name, QString oldName, double x, double y){
    /// When creating/editing a point we need the map to check if the point is on a wall or unknown place
    pointController->checkErrorPoint(mapController->getMapImage(), name, oldName, x, y);
}

void MainController::checkSpeech(QString name, QString oldName) {
    speechController->checkErrorSpeech(name, oldName);
}

void MainController::checkTmpPosition(int index, double x, double y){
    /// When creating/editing a path we need the map to check if the path point is on a wall or unknown place
    pathController->checkPosition(mapController->getMapImage(), index, x, y);
}

void MainController::saveMapConfig(QString fileName, double zoom, double centerX, double centerY, int mapRotation, bool new_config) const {
    qDebug() << "MainController::saveMapConfig called with" << fileName << zoom << mapRotation << centerX << centerY << new_config;

    if(fileName.lastIndexOf(".pgm", fileName.length()-4) != -1){
        qDebug() << "save map to:" << fileName;
        fileName = fileName.mid(0, fileName.length()-4);
        qDebug() << "fileName after" << fileName;
    }

    QFileInfo mapFileInfo(static_cast<QDir> (fileName), "");

    /// desktop
    QString filePath(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QString filePath(location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");

    QString oldFilePath = mapController->getMapFile();
    qDebug() << "save map from:" << oldFilePath;

    /// desktop
    if(!new_config){
        if(fileName != (Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName())){
            ///saves the image to the user given directory
            mapController->saveMapToFile(fileName + ".pgm");
        }

        /// saves the image as a pgm file to the software directory
        mapController->saveMapToFile(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

        mapController->savePositionSlot2(centerX, centerY, zoom, mapRotation, Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

        mapController->setMapFile(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

        mapController->saveMapConfig(filePath, centerX, centerY, zoom, mapRotation);

        /// saves the current points to the points file associated with the new configuration
        XMLParser::save(pointController, Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml");

        /// saves the new configuration to the current configuration file
        XMLParser::save(pointController, Helper::getAppPath() + QDir::separator() + "currentPoints.xml");

        /// saves the current points to the points file associated with the new configuration
        PathXMLParser::save(pathController, Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_paths.xml");

        /// saves the new configuration to the current configuration file
        PathXMLParser::save(pathController, Helper::getAppPath() + QDir::separator() + "currentPaths.xml");
    }
    else {

        mapController->savePositionSlot(centerX, centerY, zoom, mapRotation, Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

        mapController->saveMapConfig(filePath, 0, 0, 1, 0);

    }

    /// android
//    if(!new_config){
//        if(fileName != (location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName())){
//            ///saves the image to the user given directory
//            mapController->saveMapToFile(fileName + ".pgm");
//        }

//        /// saves the image as a pgm file to the software directory
//        mapController->saveMapToFile(location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

//        mapController->savePositionSlot2(centerX, centerY, zoom, mapRotation, location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

//        mapController->setMapFile(location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

//        mapController->saveMapConfig(filePath, centerX, centerY, zoom, mapRotation);

//        /// saves the current points to the points file associated with the new configuration
//        XMLParser::save(pointController, location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml");

//        /// saves the new configuration to the current configuration file
//        XMLParser::save(pointController, location + QDir::separator() + "currentPoints.xml");

//        /// saves the current speechs to the speechs file associated with the new configuration
//        SpeechXMLParser::save(speechController, location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_speechs.xml");

//        /// saves the new configuration to the current configuration file
//        SpeechXMLParser::save(speechController, location + QDir::separator() + "currentSpeechs.xml");

//        /// saves the current paths to the paths file associated with the new configuration
//        PathXMLParser::save(pathController, location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_paths.xml");

//        /// saves the new configuration to the current configuration file
//        PathXMLParser::save(pathController, location + QDir::separator() + "currentPaths.xml");
//        qDebug() << "currentpaths.xml file saved in " << location + QDir::separator();
//    }
//    else {

//        mapController->savePositionSlot(centerX, centerY, zoom, mapRotation, location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");

//        mapController->saveMapConfig(filePath, 0, 0, 1, 0);

//    }

    mapController->saveNewMap(oldFilePath);
    ///mapController->saveNewMap(Helper::getAppPath() + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".pgm");
}

void MainController::loadMapConfig(QString fileName) {
    qDebug() << "MainController::loadMapConfig called with file" << fileName;

    if(!fileName.isEmpty()){
        QString fileNameWithoutExtension;
        if(fileName.indexOf(".pgm", fileName.length()-4) != -1)
            fileNameWithoutExtension = fileName.mid(0, fileName.length()-4);

        ///save the current points and paths to the attached map before clearing it for new map
        QString oldfilePaths = mapController->getMapFile().mid(0, mapController->getMapFile().length()-4) + "_paths.xml";
        QString oldfilePoints = mapController->getMapFile().mid(0, mapController->getMapFile().length()-4) + "_points.xml";
        qDebug() << "MainController::loadMapConfig save current map paths to old path file:"<<oldfilePaths;
        PathXMLParser::save(pathController,oldfilePaths);
        qDebug() << "MainController::loadMapConfig save current map points to old point file:"<<oldfilePoints;
        XMLParser::save(pointController,oldfilePoints);


        QFileInfo mapFileInfo(static_cast<QDir> (fileNameWithoutExtension), "");

        /// desktop
        QString filePath(Helper::getAppPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");
        qDebug() << "MainController::loadMapBtnEvent map to load :" << filePath;

        /// android
//        QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//        QString filePath(location + QDir::separator() + "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + ".config");

//        setMessageTopSlot(1,  "GobotLocation path = " + filePath); /// no debugging mode so use topmessage to show output

        /// if we are able to find the configuration then we load the map
        if(mapController->loadMapConfig(filePath)){
            /// clears the map of all paths and points
            pointController->clearPoints();
            pathController->clearPaths();

            /// imports points associated to the map and save them in the current file
            /// desktop
            qDebug() << " reading points from" << Helper::getAppPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml";
            XMLParser::readPoints(pointController, Helper::getAppPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml");

            /// saves the new configuration to the current configuration file
            XMLParser::save(pointController, Helper::getAppPath() + QDir::separator() + "currentPoints.xml");

            PathXMLParser::readPaths(pathController, Helper::getAppPath() + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_paths.xml");

            /// saves the imported paths in the current paths file
            PathXMLParser::save(pathController, Helper::getAppPath() + QDir::separator() + "currentPaths.xml");

            /// android
//            XMLParser::readPoints(pointController, location + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_points.xml");

//            /// saves the new configuration to the current configuration file
//            XMLParser::save(pointController, location + QDir::separator() + "currentPoints.xml");

//            PathXMLParser::readPaths(pathController, location + QDir::separator() +  "mapConfigs" + QDir::separator() + mapFileInfo.fileName() + "_paths.xml");

//            /// saves the imported paths in the current paths file
//            PathXMLParser::save(pathController, location + QDir::separator() + "currentPaths.xml");

            QVector<double> new_home = pointController->getHome();
            qDebug() << "?????????? New home: x-" << new_home.at(0) << "y-"<<new_home.at(1) << "z-"<<new_home.at(2);
            QMap<QString, QPointer<RobotController>> robots =robotsController->getRobots();
            QMapIterator<QString, QPointer<RobotController>> it(robots);
            while(it.hasNext()){
                it.next();
                qDebug() << "connected robot:"<<it.key();
                sendCommandNewHome(it.key(),new_home.at(0),new_home.at(1),new_home.at(2));
            }

            robotsController->sendMapToAllRobots("IMPT"+mapController->getMapId().toString(),
                                                 mapController->getDateTime().toString("yyyy-MM-dd-hh-mm-ss"),
                                                 mapController->getMetadataString(),
                                                 mapController->getMapImage());
            QString message = "";
            if (langue == "English") {
                message = "Loaded the map chinese: ";
            } else {
                message = "Loaded the map: ";
            }
//            setMessageTopSlot(2, "Loaded the map: " + mapFileInfo.fileName());
            setMessageTopSlot(2, message + mapFileInfo.fileName());
        } else
            emit openWarningDialog("WARNING", "No configuration found for this map.\n\n\tPlease select another file.");
    }
}

void MainController::saveSettings(int mapChoice, double batteryThreshold){
    qDebug() << "save settings called" << mapChoice << batteryThreshold;

    /// desktop
    QFile file(Helper::getAppPath() + QDir::separator() + "settings.txt");

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QFile file(location + QDir::separator() + "settings.txt");

    if(file.open(QFile::WriteOnly)){
        QTextStream stream(&file);
        stream << mapChoice << " " << batteryThreshold ;
        file.close();
        emit emitBatteryThreshold(batteryThreshold);
    }
    QString message = "";
    if (langue == "English") {
        message = "Settings save chinese";
    } else {
        message = "Settings save";
    }
//    setMessageTopSlot(2, "Settings saved");
    setMessageTopSlot(2, message);
}

void MainController::saveWifi(QString ip, QString wifi, QString pwd) {
//    qDebug() << "\nWE ARE IN MAINCONTROLLER::saveWifi()";
    qDebug() << "saveWifi called" << ip << wifi << " " << pwd;
    QString cmd = QString("y") + QChar(31) + QString(wifi) + QChar(31) + QString(pwd);// + QChar(31) + QChar(23) + QChar(31);
    qDebug() << cmd;
    robotsController->sendCommand(ip, cmd);
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
//    qDebug() << "maincontroller: update robot pos" << robotPos << posX << posY << mapController->getOrigin() << mapController->getResolution() << mapController->getWidth() << mapController->getHeight();
    emit updateRobotPos(ip, robotPos.x(), robotPos.y(), orientation);
    mapController->getScanMapController()->updateRobotPos(ip, robotPos.x(), robotPos.y(), orientation);
}

void MainController::updatePathSlot(QString ip, QStringList strList){
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
    if(strList.size() > 0){
        qDebug() << "RobotsController::updatePathSlot"<<strList.size();
        if(strList.size() % 7 == 1){
            qDebug() << "RobotsController::updatePathSlot" << ip << " updating the path";
            emit setPath(ip, strList.takeFirst());
            for(int i = 0; i < strList.size(); i+=7){
                QPointF pathPointPos = Helper::Convert::robotCoordToPixelCoord(
                                QPointF(static_cast<QString>(strList.at(i+1)).toDouble(), static_cast<QString>(strList.at(i+2)).toDouble()),
                                mapController->getOrigin().x(),
                                mapController->getOrigin().y(),
                                mapController->getResolution(),
                                mapController->getHeight());
                emit addPathPoint(ip, strList.at(i), pathPointPos.x(), pathPointPos.y(), static_cast<QString>(strList.at(i+3)).toInt(), static_cast<QString>(strList.at(i+4)).toInt(), "", static_cast<QString>(strList.at(i+5)).toInt(), static_cast<QString>(strList.at(i+6)).toInt());
//                PathXMLParser::save(pathController,location + "/robot_command_sent.xml");
            }
        } else
            qDebug() << "RobotsController::updatePathSlot" << ip << "got a wrong number of param for the path :" << strList.size() << ", supposed to have the path name + a multiple of 5";
    } else
        qDebug() << "RobotsController::updatePathSlot" << ip << "the path is empty";

}

void MainController::updateHomeSlot(QString ip, double homeX, double homeY, double homeOri){
    QPointF homePos = Helper::Convert::robotCoordToPixelCoord(
                    QPointF(homeX, homeY),
                    mapController->getOrigin().x(),
                    mapController->getOrigin().y(),
                    mapController->getResolution(),
                    mapController->getHeight());

    qDebug() << "\nMainController::set home" << homePos.x() << homePos.y() << homeOri;
    emit setHome(ip, homePos.x(), homePos.y(), homeOri);
}

void MainController::sendCommandNewHome(QString ip, double homeX, double homeY, int homeOri){
    QString cmd;
    if(homeX==0 && homeY==0 && homeOri==0){
        cmd = QString("n") + QChar(31) + QString::number(homeX) + QChar(31) + QString::number(homeY) + QChar(31) + QString::number(homeOri);
    }
    else{
        QPointF homePos = Helper::Convert::pixelCoordToRobotCoord(
                        QPointF(homeX, homeY),
                        mapController->getOrigin().x(),
                        mapController->getOrigin().y(),
                        mapController->getResolution(),
                        mapController->getHeight());
        qDebug() << "\nMainController::sendCommandNewHome" << homeX << homeY << homePos << homeOri;
        cmd = QString("n") + QChar(31) + QString::number(homePos.x()) + QChar(31) + QString::number(homePos.y()) + QChar(31) + QString::number(homeOri);
    }
    robotsController->sendCommand(ip, cmd);
}

void MainController::sendCommandSavePlace(QString ip, QString name, double posX, double posY, double orientation, bool home) {
    int homeBool = 0;
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";

    QPointF pointReal = Helper::Convert::pixelCoordToRobotCoord(
                QPointF(posX,posY),
                mapController->getOrigin().x(),
                mapController->getOrigin().y(),
                mapController->getResolution(),
                mapController->getHeight());
    QString cmd;
    if (home == true) {
        homeBool = 1;
    } else {
        homeBool = 0;
    }
    cmd = QString("k") + QChar(31) + name + QChar(31) + QString::number(pointReal.x()) + QChar(31) + QString::number(pointReal.y()) + QChar(31) + QString::number(orientation) + QChar(31) + QString::number(homeBool);
    qDebug() << cmd;
    qDebug() << "\nMainController::sendCommandSavePlace" << name << posX << pointReal.x() << posY << pointReal.y() << orientation << homeBool;
    robotsController->sendCommand(ip, cmd);
//    XMLParser::save(pointController, location + "/point_command_sent.xml");
}

void MainController::sendCommandTtsToRobot(QString ip, QString tts) {
    QString cmd = QString("4") + QChar(31) + tts;
    robotsController->sendCommand(ip, cmd);
}

void MainController::sendCommandNewPath(QString ip, QString groupName, QString pathName){
//    qDebug() << "\nWE ARE IN maincontroller.cpp for sendCommandNewPath";
    QVector<QPointer<PathPoint>> pathPointVector = pathController->getPath(groupName, pathName);
    QString pathStr("");
    for(int i = 0; i < pathPointVector.size(); i++){
        QPointF pathPointPos = Helper::Convert::pixelCoordToRobotCoord(
                        pathPointVector.at(i)->getPoint()->getPos(),
                        mapController->getOrigin().x(),
                        mapController->getOrigin().y(),
                        mapController->getResolution(),
                        mapController->getHeight());

        /// if speechContent empty, problem while sending it to robot
        if (pathPointVector.at(i)->getSpeechContent().isEmpty()) {
            pathStr += QChar(31) + pathPointVector.at(i)->getPoint()->getName()
                    + QChar(31) + QString::number(pathPointPos.x())
                    + QChar(31) + QString::number(pathPointPos.y())
                    + QChar(31) + QString::number(pathPointVector.at(i)->getWaitTime())
                    + QChar(31) + QString::number(pathPointVector.at(i)->getPoint()->getOrientation())

                    + QChar(31) + "\" \""
                    + QChar(31) + QString::number(pathPointVector.at(i)->getSpeechTime());
        } else {
            pathStr += QChar(31) + pathPointVector.at(i)->getPoint()->getName()
                    + QChar(31) + QString::number(pathPointPos.x())
                    + QChar(31) + QString::number(pathPointPos.y())
                    + QChar(31) + QString::number(pathPointVector.at(i)->getWaitTime())
                    + QChar(31) + QString::number(pathPointVector.at(i)->getPoint()->getOrientation())
                    + QChar(31) + pathPointVector.at(i)->getSpeechContent()
                    + QChar(31) + QString::number(pathPointVector.at(i)->getSpeechTime());
        }
    }
    QString cmd = QString("i") + QChar(31) + pathName + pathStr;
    robotsController->sendCommand(ip, cmd);
}

void MainController::checkMapInfoSlot(QString ip, QString mapId, QString mapDate){
    //emit openScanWindowForAutomaticScan(ip);
    /// Neither the application nor the robot has a map so we send a command to start scanning automatically
    if(mapController->getMapImage().size().width() == 0 && !mapId.compare("{00000000-0000-0000-0000-000000000000}")){
        /// opens the scan window and adds the robot with ip <ip> to the list
        qDebug() << "NO map on either side so we start scanning automatically";
        emit openScanWindowForAutomaticScan(ip);
    }

    /// Check if the robot has the current map
    qDebug() << "MainController::updateMapInfo Robot" << ip << "comparing ids" << mapId << "and" << mapController->getMapId();
    if(mapId.compare(mapController->getMapId().toString()) == 0){
        qDebug() << "MainController::updateMapInfo Robot" << ip << "has the current map";
    }
    else {
        qDebug() << "ids of app and robot " << mapController->getMapId().toString() << mapId;
        QDateTime mapDateTime = QDateTime::fromString(mapDate, "yyyy-MM-dd-hh-mm-ss");

        bool robotOlder = (mapDateTime <= mapController->getDateTime());
        qDebug() << "Robot map date:" << mapDateTime << ". Console map date:"<< mapController->getDateTime();
        qDebug() << robotOlder;

        if(robotOlder){
            qDebug() << "MainController::updateMapInfo Robot" << ip << "has a different and older map";
        }
        else {
            qDebug() << "MainController::updateMapInfo Robot" << ip << "has a different and newer map";
        }

        ///save the current points and paths to the attached map before clearing it for new map
        QString oldfilePaths = mapController->getMapFile().mid(0, mapController->getMapFile().length()-4) + "_paths.xml";
        QString oldfilePoints = mapController->getMapFile().mid(0, mapController->getMapFile().length()-4) + "_points.xml";
        qDebug() << "MainController::loadMapConfig save current map paths to old path file:"<<oldfilePaths;
        PathXMLParser::save(pathController,oldfilePaths);
        qDebug() << "MainController::loadMapConfig save current map points to old point file:"<<oldfilePoints;
        XMLParser::save(pointController,oldfilePoints);

        int mapChoice = -1;

        QFile file(Helper::getAppPath() + QDir::separator() + "settings.txt"); /// desktop

        /// android
//        QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//        QFile file(location + QDir::separator() + "settings.txt");

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
                }
                else {
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

    qDebug() << "MainController::sendNewMap" << mapController->getMetadataString() << mapController->getMapImage().size();

    QString mapMetadata = mapController->getMetadataString();

    QVector<double> new_home = pointController->getHome();
    qDebug() << "New home: x-" << new_home.at(0) << "y-"<<new_home.at(1) << "z-"<<new_home.at(2);
    sendCommandNewHome(ip,new_home.at(0),new_home.at(1),new_home.at(2));


    robotsController->sendNewMap(ip, "IMPT"+mapId, date, mapMetadata, mapController->getMapImage());
}

void MainController::newMapFromRobotSlot(QString ip, QByteArray mapArray, QString mapId, QString mapDate, QString resolution, QString originX, QString originY, int map_width, int map_height){
    mapController->updateMetadata(map_width, map_height, resolution.toDouble(), originX.toDouble(), originY.toDouble());
    mapController->newMapFromRobot(mapArray, mapId, mapDate);
    QString mapMetadata = mapController->getMetadataString();
    ///Update center of map
    mapController->centerMapSlot();
    /// When we receive a map from a robot, we send it to all the other robots
    robotsController->sendNewMapToAllExcept(ip, mapId, mapDate, mapMetadata, mapController->getMapImage());

    /// desktop
    pointController->clearPoints();
    XMLParser::save(pointController, Helper::getAppPath() + QDir::separator() + "currentPoints.xml");
    pathController->clearPaths();
    PathXMLParser::save(pathController, Helper::getAppPath() + QDir::separator() + "currentPaths.xml");

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    pointController->clearPoints();
//    XMLParser::save(pointController, location + QDir::separator() + "currentPoints.xml");
//    pathController->clearPaths();
//    PathXMLParser::save(pathController, location + QDir::separator() + "currentPaths.xml");
//    qDebug() << "currentPaths.xml saved in " << location + QDir::separator();
}

void MainController::requestOrSendMap(QString ip, bool request){
    if(request)
        robotsController->requestMap(ip);
    else{
        QVector<double> new_home = pointController->getHome();
        qDebug() << "New home: x-" << new_home.at(0) << "y-"<<new_home.at(1) << "z-"<<new_home.at(2);
        QMap<QString, QPointer<RobotController>> robots =robotsController->getRobots();
        QMapIterator<QString, QPointer<RobotController>> it(robots);
        while(it.hasNext()){
            it.next();
            qDebug() << "connected robot:"<<it.key();
            sendCommandNewHome(it.key(),new_home.at(0),new_home.at(1),new_home.at(2));
        }

        robotsController->sendMapToAllRobots("IMPT"+mapController->getMapId().toString(),
                                             mapController->getDateTime().toString("yyyy-MM-dd-hh-mm-ss"),
                                             mapController->getMetadataString(),
                                             mapController->getMapImage());
    }
}

void MainController::getMapFromRobot(QString ip){
    robotsController->requestMapForMerging(ip);
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
        robotOri = map_reference->robotOrientation()-180;
        if (robotOri>180)
            robotOri = robotOri-360;
        else if (robotOri<-180)
            robotOri = robotOri+360;
        qDebug() << "MainController::resetMapConfiguration::Scan Initial pose in new map" << initPos.x()<<initPos.y()<<robotOri;
    }
    pointController->clearPoints();
    pathController->clearPaths();

    /// to create a new configuration
    mapController->setMapId(QUuid::createUuid());
    mapController->setDateTime(QDateTime::currentDateTime());


    /// although this is a new configuraton we have to pass false in order to reset properly the paths and points
    /// in the files
    saveMapConfig(cpp_file_name, 1, 0, 0, 0, false);

    /// desktop
    QString currentPathFile = Helper::getAppPath() + QDir::separator() + "currentMap.txt";

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QString currentPathFile = location + QDir::separator() + "currentMap.txt";

    //change current map to new map
    mapController->saveMapConfig(currentPathFile, 0, 0, 1, 0);

    /// we send this information to the robot, init pos is used to determine the position of the robot directly after gobot move
    /// is relaunched
    QString infoRobot = QString::number(mapController->getWidth()) + ' ' + QString::number(mapController->getHeight()) +
            ' ' + QString::number(mapController->getResolution()) + ' ' + QString::number(mapController->getOrigin().x()) +
            ' ' + QString::number(mapController->getOrigin().y()) + ' ' + QString::number(robotOri);;

    QImage img(cpp_file_name);

    //qDebug() << "sending map with metadata " << mapMetadata << " size of map is " << img.size();

    QVector<double> new_home = pointController->getHome();
    QString map_type = "IMPT";
    QMap<QString, QPointer<RobotController>> robots =robotsController->getRobots();
    QMapIterator<QString, QPointer<RobotController>> it(robots);

    if (scan){
        mapController->centerMapSlot();
        new_home[0] = initPos.x();
        new_home[1] = initPos.y();
        new_home[2] = robotOri;
        qDebug() << "New home: x-" << new_home.at(0) << "y-"<<new_home.at(1) << "z-"<<new_home.at(2);
        map_type = "SCAN";
        ///send home in scanned map to robot, SCAN+x
        QString cmd = QString("n") + QChar(31) + "S"+QString::number(new_home.at(0)) + QChar(31) + QString::number(new_home.at(1)) + QChar(31) + QString::number(new_home.at(2));
        while(it.hasNext()){
            it.next();
            qDebug() << "connected robot:"<<it.key();
            robotsController->sendCommand(it.key(), cmd);
        }
    }
    else{
        qDebug() << "New home: x-" << new_home.at(0) << "y-"<<new_home.at(1) << "z-"<<new_home.at(2);
        while(it.hasNext()){
            it.next();
            qDebug() << "connected robot:"<<it.key();
            sendCommandNewHome(it.key(),new_home.at(0),new_home.at(1),new_home.at(2));
        }
    }

    robotsController->sendMapToAllRobots(map_type+mapController->getMapId().toString(), mapController->getDateTime().toString("yyyy-MM-dd-hh-mm-ss"), infoRobot, img);
}

/************************* SCANNING *************************/

void MainController::startScanningSlot(QString ip){
    robotsController->sendCommand(ip, QString("t"));
}

void MainController::startAutomaticScanSlot(QString ip){
    robotsController->sendCommand(ip, QString("g"));
}

void MainController::stopScanningSlot(QString ip, bool killGobotMove){
    robotsController->sendCommand(ip, QString("u") + QChar(31) + QString::number(killGobotMove));
}

void MainController::playPauseScanningSlot(QString ip, bool wasScanning, bool scanningOnConnection){
    if(wasScanning){
        /// Then we pause the scan
        robotsController->sendCommand(ip, QString("f"));
    } else {
        /// We play the scan
        /// If the robot was already scanning, gmapping is launched so we just want to subscribe to get the map
        /// else the robot shut down while scanning and we need to relaunch gmapping to restart the scan
        /// so we ask if we want to relaunch gmapping and start the scan from the beggining or keep the previously scanned map
        if(!scanningOnConnection)
            emit openRestartScanMessageDialog(ip);
        else
            robotsController->sendCommand(ip, QString("e"));
    }
}

void MainController::playPauseExploringSlot(QString ip, bool wasExploring){
    if(wasExploring){
        /// Then we pause the exploration
        robotsController->sendCommand(ip, QString("."));
    } else {
        /// We play the exploration
        robotsController->sendCommand(ip, QString(","));
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
    robotsController->sendCommand(ip, QString("c") + QChar(31) + QString::number(goal_in_robot_coords.x()) + QChar(31) + QString::number(goal_in_robot_coords.y()));
}

void MainController::setMessageTopSlot(int status, QString msg){
    emit setMessageTop(status, msg);
}

void MainController::updateTutoFile(int index, bool visible){

    /// desktop
    QFile tutoFile(Helper::getAppPath() + QDir::separator() + "tutorial.txt");

    /// android
//    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";
//    QFile tutoFile(location + QDir::separator() + "tutorial.txt");

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
    ///save the current points and paths to the attached map before clearing it for new map
    QString oldfilePaths = mapController->getMapFile().mid(0, mapController->getMapFile().length()-4) + "_paths.xml";
    QString oldfilePoints = mapController->getMapFile().mid(0, mapController->getMapFile().length()-4) + "_points.xml";
    qDebug() << "MainController::resetMapConfiguration save current map paths to old path fild:"<<oldfilePaths;
    PathXMLParser::save(pathController,oldfilePaths);
    qDebug() << "MainController::resetMapConfiguration save current map points to old point fild:"<<oldfilePoints;
    XMLParser::save(pointController,oldfilePoints);

    /// clears the map of all paths and points
    pointController->clearPoints();
    pathController->clearPaths();
}

void MainController::sendMapToAllRobots(QString id, QString date, QString metadata, QImage img){
    robotsController->sendMapToAllRobots(id, date, metadata, img);
}

void MainController::saveVelocity(QString ip, double linearVelocity, double angularVelocity) {
    QString cmd = QString("1") + QChar(31) + QString::number(linearVelocity) + QChar(31) + QString::number(angularVelocity);
    robotsController->sendCommand(ip,cmd);
}

void MainController::saveBattery(QString ip, double battery) {
    QString cmd = QString("2") + QChar(31) + QString::number(battery);
    robotsController->sendCommand(ip,cmd);
}

void MainController::updateLinearVelocitySlot(QString ip, double linear){
    emit setLinearVelocity(ip, linear);
}

void MainController::changeLanguage(QString language) {
    qDebug() << "language in maincontroller = " << language;
    robotsController->changeLanguageSlot(language);
}


/**********************************************************************************************************/

void MainController::testSlot(){
    qDebug() << "MainController::testSlot called";
}
