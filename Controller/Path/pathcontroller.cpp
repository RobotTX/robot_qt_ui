#include "pathcontroller.h"
#include <QDir>
#include <QDebug>
#include "Controller/maincontroller.h"
#include "Model/Path/pathpoint.h"
#include "Model/Point/point.h"
#include "Model/Path/pathxmlparser.h"

PathController::PathController(QObject *applicationWindow, MainController* parent) : QObject(parent){
    paths = QPointer<Paths>(new Paths(this));



    currentPathsFile = QDir::currentPath() + QDir::separator() + "currentPaths.xml";
    qDebug() << "PathController::PathController" << currentPathsFile;
    loadPaths(currentPathsFile);
    paths->display();
}

void PathController::loadPaths(const QString fileName){
    qDebug() << "PathController::loadPaths loading paths from " << fileName;
    PathXMLParser::readPaths(this, fileName);
}

void PathController::addGroup(const QString groupName, bool saveXML){
    paths->addGroup(groupName);

    if(saveXML)
        PathXMLParser::save(this, currentPathsFile);
}

void PathController::deleteGroup(const QString groupName){
    paths->deleteGroup(groupName);

    PathXMLParser::save(this, currentPathsFile);
}

void PathController::addPath(const QString groupName, const QString name, bool saveXML){
    paths->addPath(groupName, name);

    if(saveXML)
        PathXMLParser::save(this, currentPathsFile);
}

void PathController::deletePath(const QString groupName, const QString name){
    paths->deletePath(groupName, name);

    PathXMLParser::save(this, currentPathsFile);
}

void PathController::addPathPoint(const QString groupName, const QString pathName, const QString name, const double x, const double y, const int waitTime, bool saveXML){
    paths->addPathPoint(groupName, pathName, name, x, y, waitTime);

    if(saveXML)
        PathXMLParser::save(this, currentPathsFile);
}

void PathController::deletePathPoint(const QString groupName, const QString pathName, const QString name){
    paths->deletePathPoint(groupName, pathName, name);

    PathXMLParser::save(this, currentPathsFile);
}

void PathController::renameGroup(const QString newName, const QString oldName){
    paths->renameGroup(newName, oldName);

    PathXMLParser::save(this, currentPathsFile);
}
