#include "pointcontroller.h"
#include <QApplication>
#include <QDebug>
#include <QImage>
#include <QDir>
#include <QStandardPaths>
#include "Controller/maincontroller.h"
#include "Model/Point/point.h"
#include "Model/Point/pointgroup.h"
#include "Model/Point/points.h"
#include "Model/Point/xmlparser.h"
#include "Helper/helper.h"

PointController::PointController(QObject *applicationWindow, MainController* parent) : QObject(parent){

    points = QPointer<Points>(new Points(this));

    QObject *pointModel = applicationWindow->findChild<QObject*>("pointModel");

    if (pointModel){
        /// Tell the qml point model that we just added a new group
        connect(this, SIGNAL(addGroupQml(QVariant)), pointModel, SLOT(addGroup(QVariant)));
        /// Tell the qml point model that we just added a new point
        connect(this, SIGNAL(addPointQml(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)),
                pointModel, SLOT(addPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(editPointQml(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)),
                pointModel, SLOT(editPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(deleteAllGroupsQml()), pointModel, SLOT(deleteAllGroups()));
        /// Tell the qml point model that we just renamed a group
        connect(this, SIGNAL(renameGroupQml(QVariant, QVariant)), pointModel, SLOT(renameGroup(QVariant, QVariant)));
        connect(pointModel, SIGNAL(hideShow(QString, QString)), this, SLOT(hideShow(QString, QString)));
        connect(pointModel, SIGNAL(deletePointSignal(QString, QString)), this, SLOT(deletePoint(QString, QString)));
        connect(pointModel, SIGNAL(deleteGroupSignal(QString)), this, SLOT(deleteGroup(QString)));
        connect(pointModel, SIGNAL(moveToSignal(QString, QString, QString)), this, SLOT(moveTo(QString, QString, QString)));

    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    QObject *createPointMenuFrame = applicationWindow->findChild<QObject*>("createPointMenuFrame");
    if (createPointMenuFrame){
        /// Tell the menu where we create this that we enable the save button
        connect(this, SIGNAL(enablePointSaveQml(QVariant, QVariant)), createPointMenuFrame, SLOT(enableSave(QVariant, QVariant)));
        /// Got a modification of the name or position of the point we are creating so we check to enable or not the save button
        connect(createPointMenuFrame, SIGNAL(checkPoint(QString, QString, double, double)), parent, SLOT(checkPoint(QString, QString, double, double)));
        /// Clicked on the save button to create the given point
        connect(createPointMenuFrame, SIGNAL(createPoint(QString, QString, double, double, QString, QString, bool, bool, int)), this, SLOT(addPoint(QString, QString, double, double, QString, QString, bool, bool, int)));
    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "PointController::PointController could not find the createPointMenuFrame";
        Q_UNREACHABLE();
    }

    QObject *createPointGroupMenu = applicationWindow->findChild<QObject*>("createPointGroupMenu");
    if (createPointGroupMenu){
        /// Tell the menu where we create groups that we enable the save button
        connect(this, SIGNAL(enableGroupSaveQml(QVariant)), createPointGroupMenu, SLOT(enableSave(QVariant)));
        /// The group name has been modified so we check if it's taken to enable or not the save button
        connect(createPointGroupMenu, SIGNAL(checkGroup(QString)), this, SLOT(checkGroup(QString)));
        /// Clicked on the save button to create the given group
        connect(createPointGroupMenu, SIGNAL(createGroup(QString)), this, SLOT(addGroup(QString)));
        /// Clicked on the save button while editing a group
        connect(createPointGroupMenu, SIGNAL(renameGroup(QString, QString)), this, SLOT(renameGroup(QString, QString)));
    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "PointController::PointController could not find the createPointMenuFrame";
        Q_UNREACHABLE();
    }

    QString location = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation) + QDir::separator() + "Gobot";

    /// desktop
//    currentPointsFile = Helper::getAppPath() + QDir::separator() + "currentPoints.xml";

    /// android
    currentPointsFile = location + QDir::separator() + "currentPoints.xml";
    qDebug() << "PointController::PointController" << currentPointsFile;
    loadPoints(currentPointsFile);
}

void PointController::loadPoints(const QString fileName){
    XMLParser::readPoints(this, fileName);
}


QVector<double> PointController::getHome() {
    QVector<double> home;
    QMapIterator<QString, QPointer<PointGroup>> i(points->getGroups());
    /// For each group except "No Group"
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(NO_GROUP_NAME) != 0){
            /// For each point of the group
            for(int j = 0; j < i.value()->getPointVector().size(); j++){
                if(i.value()->getPointVector().at(j)->isHome()){
                    qDebug() << "group:" <<i.key();
                    home.push_back(i.value()->getPointVector().at(j)->getPos().x());
                    home.push_back(i.value()->getPointVector().at(j)->getPos().y());
                    home.push_back(i.value()->getPointVector().at(j)->getOrientation());
                    return home;
                }
            }
        }
    }
    if(points->getGroups().contains(NO_GROUP_NAME)){
        for(int j = 0; j < points->getGroups().value(NO_GROUP_NAME)->getPointVector().size(); j++){
            if(points->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->isHome()){
                qDebug() << "group:" <<NO_GROUP_NAME;
                home.push_back(points->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->getPos().x());
                home.push_back(points->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->getPos().y());
                home.push_back(points->getGroups().value(NO_GROUP_NAME)->getPointVector().at(j)->getOrientation());
                return home;
            }
        }
    }
    home.push_back(0.0);
    home.push_back(0.0);
    home.push_back(0.0);
    return home;
}

void PointController::addGroup(QString groupName, bool saveXML){
    if(!points->getGroups().contains(groupName)){
        points->addGroup(groupName);
        emit addGroupQml(groupName);
        if(saveXML)
            XMLParser::save(this, currentPointsFile);
    }
}

void PointController::addPoint(const QString name, const QString groupName, const double x, const double y, const QString oldName, const QString oldGroup,
                               const bool displayed, const bool home, const int orientation, bool saveXML)
{
    addGroup(groupName, saveXML);
    points->addPoint(groupName, name, x, y, displayed, home, orientation);

    /// We are creating a new point
    if(oldName.isEmpty())
        emit addPointQml(name, displayed, groupName, x, y, home, orientation);
    else {
        deletePoint(oldGroup, oldName);
        emit editPointQml(oldName, oldGroup, name, displayed, groupName, x, y, home, orientation);
    }
    if(saveXML)
        XMLParser::save(this, currentPointsFile);
}

void PointController::deletePoint(QString groupName, QString name){
    qDebug() << "Delete point";
    /// we remove the point from the c++ side
    points->deletePoint(groupName, name);
    XMLParser::save(this, currentPointsFile);
}

void PointController::deleteGroup(QString groupName){
    points->deleteGroup(groupName);
    XMLParser::save(this, currentPointsFile);
}

void PointController::hideShow(QString groupName, QString name){
    qDebug() << "PointController::hideShow" << name << groupName;
    points->hideShow(groupName, name);
    XMLParser::save(this, currentPointsFile);
}

bool PointController::checkPointName(const QString name){
    QMapIterator<QString, QPointer<PointGroup>> i(points->getGroups());
    while (i.hasNext()) {
        i.next();
        QVector<QPointer<Point>> group = i.value()->getPointVector();
        for(int j = 0; j < group.size(); j++){
            if(group.at(j)->getName().compare(name) == 0)
                return true;
        }
    }
    return false;
}

void PointController::renameGroup(QString newName, QString oldName){
    qDebug() << "PointController::renameGroup from" << oldName << "to" << newName;
    points->renameGroup(newName, oldName);
    emit renameGroupQml(newName, oldName);
    XMLParser::save(this, currentPointsFile);
}

void PointController::moveTo(QString name, QString oldGroup, QString newGroup){
    qDebug() << "PointController::move" << name << "from" << oldGroup << "to" << newGroup;
    points->movePoint(name, oldGroup, newGroup);
    XMLParser::save(this, currentPointsFile);
}

void PointController::checkErrorPoint(const QImage& mapImage, const QString name, const QString oldName, const double x, const double y){
    bool nameError = false;
    bool posError = false;

    /// Name not empty
    nameError = name.isEmpty();

    /// Check if the name is taken by another point
    if(!nameError && name.compare(oldName) != 0)
        nameError = checkPointName(name);

    /// Check if the point is not in a wall, an unknown place or out of the map
    if(x < mapImage.width() && y < mapImage.height())
        posError = (mapImage.pixelColor(x, y).red() < 254);
    else
        posError = true;

    /// Send the result to qml to enable or not the save button
    emit enablePointSaveQml(posError, nameError);
}

void PointController::checkGroup(QString name){
    /// Check if the name of the group is already taken and send the result to enable or not the save button
    emit enableGroupSaveQml(!points->checkGroupName(name));
}

void PointController::clearPoints(){
    qDebug() << "PointController::clearPoints called";
    emit deleteAllGroupsQml();
    points->clearGoups();
}
