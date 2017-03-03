#include "pointcontroller.h"
#include "Model/Point/points.h"
#include "Model/Point/xmlparser.h"

PointController::PointController(QObject* parent) : QObject(parent){
    points = new Points(this);

    /// TODO create xmlParser with file name for the point => get from map ?
    //XMLParser xmlParser;
}
