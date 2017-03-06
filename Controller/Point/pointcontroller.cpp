#include "pointcontroller.h"
#include <QDebug>
#include "Model/Point/points.h"
#include "Model/Point/xmlparser.h"

PointController::PointController(QObject* parent, QString mapFile) : QObject(parent){
    points = new Points(this);

    /// WARNING check if the file is saved next to the map or next to the config file
    mapFile.remove(mapFile.length() - 4, 4);
    mapFile += "_points.xml";
    loadPoints(mapFile);
}

void PointController::loadPoints(QString fileName){
    qDebug() << "PointController::loadPoints loading points from " << fileName;
    XMLParser::readPoints(points, fileName);
}
