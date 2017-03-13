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
        /// Tell the qml point model that we just added a new group
        connect(points, SIGNAL(addGroupQml(QVariant, QVariant)), pointModel, SLOT(addGroup(QVariant, QVariant)));
        /// Tell the qml point model that we just added a new point
        connect(points, SIGNAL(addPointQml(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)), pointModel, SLOT(addPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
        /// Tell the qml point model that we just removed a point
        connect(points, SIGNAL(removePointQml(QVariant)), pointModel, SLOT(removePoint(QVariant)));
        /// Tell the qml point model that we just removed a group
        connect(points, SIGNAL(removeGroupQml(QVariant, QVariant)), pointModel, SLOT(removeGroup(QVariant, QVariant)));
        /// Tell the qml point model that we just hided or showed a point on the map
        connect(points, SIGNAL(hideShowQml(QVariant, QVariant)), pointModel, SLOT(hideShow(QVariant, QVariant)));
    } else {
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    QObject *pointList = applicationWindow->findChild<QObject*>("pointList");
    if (pointList){
        /// Got a signal from the qml list of point that we just clicked to hide/show the given point
        connect(pointList, SIGNAL(hideShow(QString, QString, bool)), points, SLOT(hideShow(QString, QString, bool)));
     } else {
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    QObject *createPointMenuFrame = applicationWindow->findChild<QObject*>("createPointMenuFrame");
    if (createPointMenuFrame){
        /// Tell the menu where we create points that we enable the save button
        connect(this, SIGNAL(enablePointSaveQml(QVariant)), createPointMenuFrame, SLOT(enableSave(QVariant)));
        /// Got a modification of the name or position of the point we are creating so we check to enable or not the save button
        connect(createPointMenuFrame, SIGNAL(checkPoint(QString, double, double)), parent, SLOT(checkPoint(QString, double, double)));
        /// Clicked on the save button to create the given point
        connect(createPointMenuFrame, SIGNAL(createPoint(QString, QString, double, double)), points, SLOT(addPoint(QString, QString, double, double)));
    } else {
        qDebug() << "PointController::PointController could not find the createPointMenuFrame";
        Q_UNREACHABLE();
    }

    QObject *createGroupMenuFrame = applicationWindow->findChild<QObject*>("createGroupMenuFrame");
    if (createGroupMenuFrame){
        /// Tell the menu where we create groups that we enable the save button
        connect(this, SIGNAL(enableGroupSaveQml(QVariant)), createGroupMenuFrame, SLOT(enableSave(QVariant)));
        /// The group name has been modified so we check if it's taken to enable or not the save button
        connect(createGroupMenuFrame, SIGNAL(checkGroup(QString)), this, SLOT(checkGroup(QString)));
        /// Clicked on the save button to create the given group
        connect(createGroupMenuFrame, SIGNAL(createGroup(QString)), points, SLOT(addGroup(QString)));
        /// Clicked on the save button while editing a group
        connect(createGroupMenuFrame, SIGNAL(renameGroup(QString, QString)), points, SLOT(renameGroup(QString, QString)));
    } else {
        qDebug() << "PointController::PointController could not find the createPointMenuFrame";
        Q_UNREACHABLE();
    }

    QObject *pointMenuFrame = applicationWindow->findChild<QObject*>("pointMenuFrame");
    if (pointMenuFrame){
        /// Clicked on the right popup menu to delete a point
        connect(pointMenuFrame, SIGNAL(deletePoint(QString, QString)), points, SLOT(deletePoint(QString, QString)));
        /// Clicked on the right popup menu to delete a group
        connect(pointMenuFrame, SIGNAL(deleteGroup(QString)), points, SLOT(deleteGroup(QString)));
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

    /// Name not empty
    error = name.isEmpty();

    /// Check if the name is taken by another point
    if(!error)
        error = points->checkPointName(name);

    /// Check if the point is not in a wall or unknown place
    if(!error)
        error = (mapImage.pixelColor(x, y).red() != 255);

    /// Send the result to qml to enable or not the save button
    emit enablePointSaveQml(QVariant::fromValue(!error));
}

void PointController::checkGroup(QString name){
    /// Check if the name of the group is already taken and send the result to enable or not the save button
    emit enableGroupSaveQml(QVariant::fromValue(!points->checkGroupName(name)));
}
