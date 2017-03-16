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
