#include "pointcontroller.h"
#include <QDebug>
#include <QImage>
#include <QDir>
#include "Controller/maincontroller.h"
#include "Model/Point/point.h"
#include "Model/Point/points.h"
#include "Model/Point/xmlparser.h"
#include "Helper/helper.h"

PointController::PointController(QObject *applicationWindow, MainController* parent) : QObject(parent){
    points = new Points(this);

    QObject *pointModel = applicationWindow->findChild<QObject*>("pointModel");
    if (pointModel){
        /// Tell the qml point model that we just added a new group
        connect(this, SIGNAL(addGroupQml(QVariant, QVariant)), pointModel, SLOT(addGroup(QVariant, QVariant)));
        /// Tell the qml point model that we just added a new point
        connect(this, SIGNAL(addPointQml(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)), pointModel, SLOT(addPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
        /// Tell the qml point model that we just removed a point
        connect(this, SIGNAL(removePointQml(QVariant)), pointModel, SLOT(removePoint(QVariant)));
        /// Tell the qml point model that we just removed a group
        connect(this, SIGNAL(removeGroupQml(QVariant, QVariant)), pointModel, SLOT(removeGroup(QVariant, QVariant)));
        /// Tell the qml point model that we just hided or showed a point on the map
        connect(this, SIGNAL(hideShowQml(QVariant, QVariant)), pointModel, SLOT(hideShow(QVariant, QVariant)));
        /// Tell the qml point model that we just renamed a group
        connect(this, SIGNAL(renameGroupQml(QVariant, QVariant)), pointModel, SLOT(renameGroup(QVariant, QVariant)));
        /// Tell the qml point model that we moved a point to a new group
        connect(this, SIGNAL(movePointQml(QVariant, QVariant, QVariant)), pointModel, SLOT(movePoint(QVariant, QVariant, QVariant)));
    } else {
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    QObject *pointList = applicationWindow->findChild<QObject*>("pointList");
    if (pointList){
        /// Got a signal from the qml list of point that we just clicked to hide/show the given point
        connect(pointList, SIGNAL(hideShow(QString, QString, bool)), this, SLOT(hideShow(QString, QString, bool)));
     } else {
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    QObject *createPointMenuFrame = applicationWindow->findChild<QObject*>("createPointMenuFrame");
    if (createPointMenuFrame){
        /// Tell the menu where we create this that we enable the save button
        connect(this, SIGNAL(enablePointSaveQml(QVariant)), createPointMenuFrame, SLOT(enableSave(QVariant)));
        /// Got a modification of the name or position of the point we are creating so we check to enable or not the save button
        connect(createPointMenuFrame, SIGNAL(checkPoint(QString, QString, double, double)), parent, SLOT(checkPoint(QString, QString, double, double)));
        /// Clicked on the save button to create the given point
        connect(createPointMenuFrame, SIGNAL(createPoint(QString, QString, double, double, QString, QString)), this, SLOT(addPoint(QString, QString, double, double, QString, QString)));
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
        connect(createGroupMenuFrame, SIGNAL(createGroup(QString)), this, SLOT(addGroup(QString)));
        /// Clicked on the save button while editing a group
        connect(createGroupMenuFrame, SIGNAL(renameGroup(QString, QString)), this, SLOT(renameGroup(QString, QString)));
    } else {
        qDebug() << "PointController::PointController could not find the createPointMenuFrame";
        Q_UNREACHABLE();
    }

    QObject *pointMenuFrame = applicationWindow->findChild<QObject*>("pointMenuFrame");
    if (pointMenuFrame){
        /// Clicked on the right popup menu to delete a point
        connect(pointMenuFrame, SIGNAL(deletePoint(QString, QString)), this, SLOT(deletePoint(QString, QString)));
        /// Clicked on the right popup menu to delete a group
        connect(pointMenuFrame, SIGNAL(deleteGroup(QString)), this, SLOT(deleteGroup(QString)));
        connect(pointMenuFrame, SIGNAL(moveTo(QString, QString, QString)), this, SLOT(moveTo(QString, QString, QString)));
    } else {
        qDebug() << "PointController::PointController could not find the createPointMenuFrame";
        Q_UNREACHABLE();
    }


    currentPointsFile = QDir::currentPath() + QDir::separator() + "currentPoints.xml";
    qDebug() << "PointController::PointController" << currentPointsFile;
    loadPoints(currentPointsFile);
}

void PointController::loadPoints(const QString fileName){
    qDebug() << "PointController::loadPoints loading points from " << fileName;
    XMLParser::readPoints(this, fileName);
}


void PointController::addGroup(QString groupName, bool saveXML){
    groupName = Helper::formatName(groupName);
    if(!points->getGroups()->contains(groupName)){
        //qDebug() << "PointController::addGroup" << groupName;
        points->getGroups()->insert(groupName, new QVector<Point*>());

        emit addGroupQml(QVariant::fromValue(indexOfGroup(groupName)),
                         QVariant::fromValue(groupName));

        if(saveXML)
            XMLParser::save(this, currentPointsFile);
    }
}

void PointController::addPoint(QString name, QString groupName, double x, double y, QString oldName, QString oldGroup, bool displayed, bool saveXML){
    //qDebug() << "PointController::addPoint" << groupName << name << x << y << displayed;
    addGroup(groupName, saveXML);

    name = Helper::formatName(name);
    /// We are creating a new point
    if(oldName.isEmpty()){
        qDebug() << "PointController::addPoint Creating a point";
        QVector<Point*>* group = points->getGroups()->value(groupName);
        group->push_back(new Point(name, x, y, displayed, this));
        emit addPointQml(QVariant::fromValue(indexOfPoint(name, groupName)),
                         QVariant::fromValue(name),
                         QVariant::fromValue(displayed),
                         QVariant::fromValue(groupName),
                         QVariant::fromValue(x),
                         QVariant::fromValue(y));
    } else {
        qDebug() << "PointController::addPoint Editing a point";
        /// We are editing a point
        deletePoint(oldName, oldGroup);

        QVector<Point*>* group = points->getGroups()->value(groupName);
        group->push_back(new Point(name, x, y, displayed, this));
        emit addPointQml(QVariant::fromValue(indexOfPoint(name, groupName)),
                         QVariant::fromValue(name),
                         QVariant::fromValue(displayed),
                         QVariant::fromValue(groupName),
                         QVariant::fromValue(x),
                         QVariant::fromValue(y));
    }

    if(saveXML)
        XMLParser::save(this, currentPointsFile);
}

int PointController::indexOfPoint(QString pointName, QString groupName){
    QVector<Point*>* group = points->getGroups()->value(groupName);
    for(int j = 0; j < group->size(); j++){
        if(group->at(j)->getName().compare(pointName) == 0)
            return j;
    }

    return -1;
}

int PointController::indexOfGroup(QString groupName){
    QMapIterator<QString, QVector<Point*>*> i(*(points->getGroups()));
    int index(0);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(groupName) == 0)
            return index;
        index++;
    }

    return -1;
}

void PointController::deletePoint(QString name, QString groupName){
    /// we want ot delete a point so we remove it from the qml side
    emit removePointQml(QVariant::fromValue(indexOfPoint(name, groupName)));
    /// and we remove it from the c++ side
    QVector<Point*>* group = points->getGroups()->value(groupName);
    for(int i = 0; i < group->size(); i++)
        if(group->at(i)->getName().compare(name) == 0)
            group->remove(i);
    XMLParser::save(this, currentPointsFile);
}

void PointController::deleteGroup(QString name){
    /// if we want to delete a group, we delete all its points from the qml side and the c++ side
    if(points->getGroups()->find(name) != points->getGroups()->end()){
        /// Remove the group from qml side first as we need the index in the c++ model
        emit removeGroupQml(QVariant::fromValue(indexOfGroup(name)),
                            QVariant::fromValue(indexOfPoint(points->getGroups()->value(name)->last()->getName(), name)));
        /// Remove from c++ model
        points->getGroups()->remove(name);
    }
    XMLParser::save(this, currentPointsFile);
}

void PointController::hideShow(QString name, QString groupName, bool show){
    qDebug() << "PointController::hideShow" << name << groupName << show;
    /// If it's a group we just want to open it on the menu
    if(groupName.isEmpty())
        emit hideShowQml(QVariant::fromValue(indexOfGroup(name)), QVariant::fromValue(!show));
     else {
        /// If it's a point, we ant to show/hide it on the map
        QVector<Point*>* group = points->getGroups()->value(groupName);
        for(int i = 0; i < group->size(); i++)
            if(group->at(i)->getName().compare(name) == 0)
                group->at(i)->setVisible(!show);

        emit hideShowQml(QVariant::fromValue(indexOfPoint(name, groupName)), QVariant::fromValue(!show));

        XMLParser::save(this, currentPointsFile);
    }
}

bool PointController::checkPointName(const QString name){
    QMapIterator<QString, QVector<Point*>*> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();

        QVector<Point*>* group = i.value();
        for(int j = 0; j < group->size(); j++){
            if(group->at(j)->getName().compare(Helper::formatName(name)) == 0)
                return true;
        }
    }
    return false;
}

bool PointController::checkGroupName(const QString name){
    return points->getGroups()->find(Helper::formatName(name)) != points->getGroups()->end();
}

void PointController::renameGroup(QString newName, QString oldName){
    newName = Helper::formatName(newName);
    qDebug() << "PointController::renameGroup from" << oldName << "to" << newName;
    points->getGroups()->insert(newName, points->getGroups()->take(oldName));
    emit renameGroupQml(newName, oldName);
    XMLParser::save(this, currentPointsFile);
}

void PointController::moveTo(QString name, QString oldGroup, QString newGroup){
    qDebug() << "PointController::move" << name << "from" << oldGroup << "to" << newGroup;
    /// TODO move to + on rename group
    int oldIndex = indexOfPoint(name, oldGroup);

    QVector<Point*>* group = points->getGroups()->value(oldGroup);
    for(int j = 0; j < group->size(); j++)
        if(group->at(j)->getName().compare(name) == 0)
            points->getGroups()->value(newGroup)->push_back(group->takeAt(j));

    int newIndex = indexOfPoint(name, newGroup);
    emit movePointQml(QVariant::fromValue(oldIndex), QVariant::fromValue(newIndex), QVariant::fromValue(newGroup));

    XMLParser::save(this, currentPointsFile);
}

void PointController::checkErrorPoint(const QImage& mapImage, const QString name, const QString oldName, const double x, const double y){
    bool error = false;

    /// Name not empty
    error = name.isEmpty();

    /// Check if the name is taken by another point
    if(!error && name.compare(oldName) != 0)
        error = checkPointName(name);

    /// Check if the point is not in a wall or unknown place
    if(!error)
        error = (mapImage.pixelColor(x, y).red() != 255);

    /// Send the result to qml to enable or not the save button
    emit enablePointSaveQml(QVariant::fromValue(!error));
}

void PointController::checkGroup(QString name){
    /// Check if the name of the group is already taken and send the result to enable or not the save button
    emit enableGroupSaveQml(QVariant::fromValue(!checkGroupName(name)));
}
