#include "pointcontroller.h"
#include <QDebug>
#include "Model/Point/points.h"
#include "Model/Point/xmlparser.h"

PointController::PointController(QObject *applicationWindow, QString mapFile, QObject* parent) : QObject(parent){
    points = new Points(this);

    QObject *pointModel = applicationWindow->findChild<QObject*>("pointModel");
    if (pointModel){
        connect(points, SIGNAL(addGroupQml(QVariant, QVariant)), pointModel, SLOT(addGroup(QVariant, QVariant)));
        connect(points, SIGNAL(addPointQml(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)), pointModel, SLOT(addPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
    } else {
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    QObject *pointList = applicationWindow->findChild<QObject*>("pointList");
    if (pointList){
        connect(pointList, SIGNAL(deletePointOrGroup(QString, QString)), points, SLOT(deletePointOrGroup(QString, QString)));
        connect(points, SIGNAL(removePointQml(QVariant)), pointList, SLOT(removePoint(QVariant)));
        connect(points, SIGNAL(removeGroupQml(QVariant, QVariant)), pointList, SLOT(removeGroup(QVariant, QVariant)));
    } else {
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    /// WARNING check if the file is saved next to the map or next to the config file
    mapFile.remove(mapFile.length() - 4, 4);
    mapFile += "_points.xml";
    loadPoints(mapFile);
}

void PointController::loadPoints(QString fileName){
    qDebug() << "PointController::loadPoints loading points from " << fileName;
    XMLParser::readPoints(points, fileName);
}
