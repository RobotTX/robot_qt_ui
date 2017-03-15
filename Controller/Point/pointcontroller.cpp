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
        connect(this, SIGNAL(editPointQml(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)), pointModel, SLOT(editPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
        /// Tell the qml point model that we just renamed a group
        connect(this, SIGNAL(renameGroupQml(QVariant, QVariant)), pointModel, SLOT(renameGroup(QVariant, QVariant)));
        connect(pointModel, SIGNAL(hideShow(QString, QString)), this, SLOT(hideShow(QString, QString)));
        connect(pointModel, SIGNAL(deletePointSignal(QString, QString)), this, SLOT(deletePoint(QString, QString)));
        connect(pointModel, SIGNAL(deleteGroupSignal(QString)), this, SLOT(deleteGroup(QString)));
        connect(pointModel, SIGNAL(moveToSignal(QString, QString, QString)), this, SLOT(moveTo(QString, QString, QString)));
    } else {
        qDebug() << "PointController::PointController could not find the qml point model";
        Q_UNREACHABLE();
    }

    QObject *pointList = applicationWindow->findChild<QObject*>("pointList");
    if (pointList){
        /// Got a signal from the qml list of point that we just clicked to hide/show the given point
        //connect(pointList, SIGNAL(hideShow(QString, QString, bool)), this, SLOT(hideShow(QString, QString, bool)));
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
        //connect(pointMenuFrame, SIGNAL(moveTo(QString, QString, QString)), this, SLOT(moveTo(QString, QString, QString)));
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
        QVector<Point*>* group = points->getGroups()->value(groupName);
        group->push_back(new Point(name, x, y, displayed, this));
        emit editPointQml(QVariant::fromValue(oldName),
                         QVariant::fromValue(oldGroup),
                         QVariant::fromValue(indexOfPoint(name, groupName)),
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

void PointController::deletePoint(QString groupName, QString name){
    qDebug() << "Delete point";
    /// we remove the point from the c++ side
    QVector<Point*>* group = points->getGroups()->value(groupName);
    for(int i = 0; i < group->size(); i++)
        if(group->at(i)->getName().compare(name) == 0)
            group->remove(i);
    XMLParser::save(this, currentPointsFile);
}

void PointController::deleteGroup(QString name){
    if(points->getGroups()->find(name) != points->getGroups()->end()){
        /// Remove from c++ model
        points->getGroups()->remove(name);
    }
    XMLParser::save(this, currentPointsFile);
}

void PointController::hideShow(QString groupName, QString name){
    qDebug() << "PointController::hideShow" << name << groupName;

    QVector<Point*>* group = points->getGroups()->value(groupName);
    for(int i = 0; i < group->size(); i++)
        if(group->at(i)->getName().compare(name) == 0)
            group->at(i)->setVisible(!group->at(i)->isVisible());

    XMLParser::save(this, currentPointsFile);
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
    QVector<Point*>* group = points->getGroups()->value(oldGroup);
    for(int j = 0; j < group->size(); j++)
        if(group->at(j)->getName().compare(name) == 0)
            points->getGroups()->value(newGroup)->push_back(group->takeAt(j));

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
