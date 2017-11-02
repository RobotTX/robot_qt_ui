#include "pathcontroller.h"
#include <QApplication>
#include <QDir>
#include <QDebug>
#include <QImage>
#include "Controller/maincontroller.h"
#include "Model/Point/point.h"
#include "Model/Path/pathxmlparser.h"
#include "Helper/helper.h"

PathController::PathController(QObject *applicationWindow, MainController* parent) : QObject(parent){

    paths = QPointer<Paths>(new Paths(this));

    QObject *pathModel = applicationWindow->findChild<QObject*>("pathModel");

    if (pathModel){
        /// Signals from c++ to the qml path model
        connect(this, SIGNAL(addGroupQml(QVariant)), pathModel, SLOT(addGroup(QVariant)));
        connect(this, SIGNAL(addPathQml(QVariant, QVariant)), pathModel, SLOT(addPath(QVariant, QVariant)));
        connect(this, SIGNAL(addPathPointQml(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)),
                pathModel, SLOT(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(renameGroupQml(QVariant, QVariant)), pathModel, SLOT(renameGroup(QVariant, QVariant)));
        connect(this, SIGNAL(deleteAllPathsQml()), pathModel, SLOT(deleteAllPaths()));

        /// Signals from the qml path model to c++
        connect(pathModel, SIGNAL(deletePathSignal(QString, QString)), this, SLOT(deletePath(QString, QString)));
        connect(pathModel, SIGNAL(deleteGroupSignal(QString)), this, SLOT(deleteGroup(QString)));
        connect(pathModel, SIGNAL(moveToSignal(QString, QString, QString)), this, SLOT(moveTo(QString, QString, QString)));

    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "PathController::PathController could not find the qml point model";
        Q_UNREACHABLE();
    }

    /// The model of the tmp path that we use when we create a path
    QObject *tmpPathModel = applicationWindow->findChild<QObject*>("tmpPathModel");
    if (tmpPathModel){
        connect(tmpPathModel, SIGNAL(checkTmpPosition(int, double, double)), parent, SLOT(checkTmpPosition(int, double, double)));
        connect(this, SIGNAL(setTmpValidPositionQml(QVariant, QVariant)), tmpPathModel, SLOT(setTmpValidPosition(QVariant, QVariant)));
    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "PathController::PathController could not find the qml point model";
        Q_UNREACHABLE();
    }

    /// Signals of the page where we create path groups
    QObject *createPathGroupMenu = applicationWindow->findChild<QObject*>("createPathGroupMenu");
    if (createPathGroupMenu){
        /// Tell the menu where we create groups that we enable the save button
        connect(this, SIGNAL(enableGroupSaveQml(QVariant)), createPathGroupMenu, SLOT(enableSave(QVariant)));
        /// The group name has been modified so we check if it's taken to enable or not the save button
        connect(createPathGroupMenu, SIGNAL(checkGroup(QString)), this, SLOT(checkGroup(QString)));
        /// Clicked on the save button to create the given group
        connect(createPathGroupMenu, SIGNAL(createGroup(QString)), this, SLOT(addGroup(QString)));
        /// Clicked on the save button while editing a group
        connect(createPathGroupMenu, SIGNAL(renameGroup(QString, QString)), this, SLOT(renameGroup(QString, QString)));
    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "PathController::PathController could not find the createPathGroupMenu";
        Q_UNREACHABLE();
    }

    /// Signals of the page where we create paths
    QObject *createPathMenuFrame = applicationWindow->findChild<QObject*>("createPathMenuFrame");
    if (createPathMenuFrame){
        /// Clicked on the save button to create the given group
        connect(createPathMenuFrame, SIGNAL(createPath(QString, QString)), this, SLOT(addPath(QString, QString)));
        /// Clicked on the save button while editing a group
        connect(createPathMenuFrame, SIGNAL(createPathPoint(QString, QString, QString, double, double, int, int)), this, SLOT(addPathPoint(QString, QString, QString, double, double, int, int)));
    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "PathController::PathController could not find the createPathMenuFrame";
        Q_UNREACHABLE();
    }

    currentPathsFile = Helper::getAppPath() + QDir::separator() + "currentPaths.xml";
    qDebug() << "PathController::PathController" << currentPathsFile;
    loadPaths(currentPathsFile);
}

void PathController::loadPaths(const QString fileName){
    PathXMLParser::readPaths(this, fileName);
}

void PathController::addGroup(const QString groupName, const bool saveXML){
    /// so we never add the same group twice
    if(!paths->getGroups().contains(groupName)){
        paths->addGroup(groupName);
        emit addGroupQml(groupName);
        if(saveXML)
            PathXMLParser::save(this, currentPathsFile);
    }
}

void PathController::deleteGroup(const QString groupName){
    paths->deleteGroup(groupName);
    PathXMLParser::save(this, currentPathsFile);
}

void PathController::addPath(const QString groupName, const QString name, const bool saveXML){
    paths->addPath(groupName, name);
    emit addPathQml(name, groupName);
    if(saveXML)
        PathXMLParser::save(this, currentPathsFile);
}

void PathController::deletePath(const QString groupName, const QString name){
    paths->deletePath(groupName, name);
    PathXMLParser::save(this, currentPathsFile);
}

void PathController::addPathPoint(const QString groupName, const QString pathName, const QString name, const double x, const double y, const int waitTime, const int orientation, const bool saveXML){
    paths->addPathPoint(groupName, pathName, name, x, y, waitTime, orientation);
    emit addPathPointQml(name, pathName,
                    groupName, x,
                    y, waitTime, orientation);

    if(saveXML)
        PathXMLParser::save(this, currentPathsFile);
}

void PathController::deletePathPoint(const QString groupName, const QString pathName, const QString name){
    paths->deletePathPoint(groupName, pathName, name);
    PathXMLParser::save(this, currentPathsFile);
}

void PathController::renameGroup(const QString newName, const QString oldName){
    paths->renameGroup(newName, oldName);
    emit renameGroupQml(newName, oldName);
    PathXMLParser::save(this, currentPathsFile);
}

void PathController::checkGroup(const QString name){
    /// Check if the name of the group is already taken and send the result to enable or not the save button
    emit enableGroupSaveQml(!paths->checkGroupName(name));
}

void PathController::moveTo(const QString name, const QString oldGroup, const QString newGroup){
    paths->movePath(name, oldGroup, newGroup);
    PathXMLParser::save(this, currentPathsFile);
}

void PathController::checkPosition(const QImage& mapImage, const int index, const double x, const double y){
    qDebug() << "PathController::checkPosition" << x << y << mapImage.pixelColor(x, y).red();
    emit setTmpValidPositionQml(index, mapImage.pixelColor(x, y).red() >= 254);
}

void PathController::clearPaths(){
    paths->clearGroups();
    emit deleteAllPathsQml();
}
