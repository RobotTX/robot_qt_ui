#include "pointcontroller.h"
#include <QDebug>
#include <QImage>
#include "Controller/maincontroller.h"
#include "Model/Point/points.h"
#include "Model/Point/xmlparser.h"

PointController::PointController(QObject *applicationWindow, QString mapFile, MainController* parent) : QObject(parent){
    points = new Points(this);

    QObject *pointModel = applicationWindow->findChild<QObject*>("pointModel");
    if (pointModel){
        connect(points, SIGNAL(addGroupQml(QVariant, QVariant)), pointModel, SLOT(addGroup(QVariant, QVariant)));
        connect(points, SIGNAL(addPointQml(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)), pointModel, SLOT(addPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(points, SIGNAL(removePointQml(QVariant)), pointModel, SLOT(removePoint(QVariant)));
        connect(points, SIGNAL(removeGroupQml(QVariant, QVariant)), pointModel, SLOT(removeGroup(QVariant, QVariant)));
        connect(points, SIGNAL(hideShowQml(QVariant, QVariant)), pointModel, SLOT(hideShow(QVariant, QVariant)));
    } else {
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    QObject *pointList = applicationWindow->findChild<QObject*>("pointList");
    if (pointList){
        connect(pointList, SIGNAL(deletePointOrGroup(QString, QString)), points, SLOT(deletePointOrGroup(QString, QString)));
        connect(pointList, SIGNAL(hideShow(QString, QString, bool)), points, SLOT(hideShow(QString, QString, bool)));
     } else {
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    QObject *createPointMenuFrame = applicationWindow->findChild<QObject*>("createPointMenuFrame");
    if (createPointMenuFrame){
        connect(this, SIGNAL(enablePointSaveQml(QVariant)), createPointMenuFrame, SLOT(enableSave(QVariant)));
        connect(createPointMenuFrame, SIGNAL(checkPoint(QString, double, double)), parent, SLOT(checkPoint(QString, double, double)));
        connect(createPointMenuFrame, SIGNAL(createPoint(QString, QString, double, double)), points, SLOT(addPoint(QString, QString, double, double)));
    } else {
        qDebug() << "PointController::PointController could not find the createPointMenuFrame";
        Q_UNREACHABLE();
    }

    QObject *createGroupMenuFrame = applicationWindow->findChild<QObject*>("createGroupMenuFrame");
    if (createGroupMenuFrame){
        connect(this, SIGNAL(enableGroupSaveQml(QVariant)), createGroupMenuFrame, SLOT(enableSave(QVariant)));
        connect(createGroupMenuFrame, SIGNAL(checkGroup(QString)), this, SLOT(checkGroup(QString)));
        connect(createGroupMenuFrame, SIGNAL(createGroup(QString)), points, SLOT(addGroup(QString)));
    } else {
        qDebug() << "PointController::PointController could not find the createPointMenuFrame";
        Q_UNREACHABLE();
    }

    /// WARNING check if the file is saved next to the map or next to the config file
    mapFile.remove(mapFile.length() - 4, 4);
    mapFile += "_points.xml";
    loadPoints(mapFile);
}

void PointController::loadPoints(const QString fileName){
    qDebug() << "PointController::loadPoints loading points from " << fileName;
    XMLParser::readPoints(points, fileName);
}

void PointController::checkErrorPoint(const QImage& mapImage, const QString name, const double x, const double y){
    bool error = false;

    error = name.isEmpty();

    if(!error)
        error = points->checkPointName(name);

    if(!error)
        error = (mapImage.pixelColor(x, y).red() != 255);

    emit enablePointSaveQml(QVariant::fromValue(!error));
}

void PointController::checkGroup(QString name){
    emit enableGroupSaveQml(QVariant::fromValue(!points->checkGroupName(name)));
}
