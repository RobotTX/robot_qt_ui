#include "pathcontroller.h"
#include "Controller/maincontroller.h"
#include "Model/Path/paths.h"
#include "Model/Path/pathpoint.h"
#include "Model/Point/point.h"

PathController::PathController(QObject *applicationWindow, MainController* parent) : QObject(parent){
    paths = new Paths(this);

    /*
     deserializePaths(QDir::currentPath() + QDir::separator() + "paths.dat");
     */
}

void PathController::serializePaths(const QString fileName){
    QFile pathFile(fileName);
    pathFile.resize(0);
    pathFile.open(QIODevice::ReadWrite);
    QDataStream out(&pathFile);
    out << *paths;
    pathFile.close();
}

void PathController::deserializePaths(const QString fileName){
    QFile pathFile(fileName);
    pathFile.open(QIODevice::ReadWrite);
    QDataStream in(&pathFile);
    Paths tmpPaths;
    in >> tmpPaths;
    pathFile.close();
    paths->setGroups(tmpPaths.getGroups());
}

