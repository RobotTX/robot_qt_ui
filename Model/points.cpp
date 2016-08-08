#include "points.h"
#include <QDataStream>
#include <QDebug>
#include <QMapIterator>
#include "Controller/mainwindow.h"
#include "View/mapview.h"

Points::Points(QObject *parent) : QObject(parent){
    groups = std::shared_ptr<Groups>(new Groups());
}

void Points::addGroup(const QString groupName, std::shared_ptr<QVector<std::shared_ptr<PointView>>> points){
    groups->insert(groupName, points);
}

void Points::display(std::ostream& stream) const {
    std::cout << "This list of points contains " << groups->size() << " groups :" << std::endl;
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        stream << i.key().toStdString() << " : " << std::endl;
        for(int j = 0; j < i.value()->size(); j++){
            stream << i.value()->at(j)->getPoint() << std::endl;
        }
    }
}

std::ostream& operator <<(std::ostream& stream, Points const& points){
    points.display(stream);
    return stream;
}


QDataStream& operator>>(QDataStream& in, Points& points){
    qDebug() << "Points operator >> called";
    /// the size of the vector has to be serialized too in order to deserialize the object correctly
    /*qint32 size;
    in >> size;
    for(int i = 0; i < size; i++){
        Group group;
        in >> group;
        points.addGroup(group);
    }*/
    return in;
}

QDataStream& operator<<(QDataStream& out, const Points& points){
    qDebug() << "Points operator << called";
    /*out << qint32(points.getGroups().size());
    for(int i = 0; i < points.getGroups().size(); i++)
        out << *(points.getGroups().at(i));
    return out;*/
}

void Points::removeGroup(const QString groupName) {
    if(groupName.compare(NO_GROUP_NAME) != 0)
        groups->remove(groupName);
}

void Points::removePoint(const QString pointName) {
    qDebug() << "RemovePoint called" << pointName;
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    QString key = "";
    int index = -1;
    while (i.hasNext()) {
        i.next();
        for(int j = 0; j < i.value()->size(); j++){
            if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0){
                key = i.key();
                index = j;
            }
        }
    }
    if(key.compare("") != 0 && index != -1){
        groups->value(key)->remove(index);
        qDebug() << "Points::removePoint" << pointName << "done";
    } else {
        qDebug() << "Points::removePoint could not find the point to delete";
    }
}

std::shared_ptr<QVector<std::shared_ptr<PointView>>> Points::findGroup(const QString groupName) const {
    return groups->value(groupName);
}

std::shared_ptr<PointView> Points::findPointView(const QString pointName) const{
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0)
                    return i.value()->at(j);
            }
        }
    }
    return NULL;
}

std::shared_ptr<PointView> Points::findPathPointView(const double x, const double y) const{
    if(groups->value(PATH_GROUP_NAME)){
        for(int j = 0; j < groups->value(PATH_GROUP_NAME)->size(); j++){
            if(groups->value(PATH_GROUP_NAME)->at(j)->getPoint()->comparePos(x, y))
                return groups->value(PATH_GROUP_NAME)->at(j);
        }
    }
    return NULL;
}

std::shared_ptr<Point> Points::findPoint(const QString pointName) const {
    qDebug() << "Points::findPoint called" << pointName;
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0)
                    return i.value()->at(j)->getPoint();
            }
        }
    }
    return NULL;
}

std::shared_ptr<Point> Points::findPoint(const QString groupName, const int indexPoint) const {
    return groups->value(groupName)->at(indexPoint)->getPoint();
}

std::pair<QString, int> Points::findPointIndexes(const QString pointName) const {
    std::pair<QString, int> indexes(std::make_pair<QString, int>("", -1));
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0){
                    indexes.first = i.key();
                    indexes.second = j;
                }
            }
        }
    }
    return indexes;
}

/// clears all the groups and add an empty default group
void Points::clear(){
    qDebug() << "Points::clear called";
    groups->clear();
    //addGroup(Group(NO_GROUP_NAME));
}

std::shared_ptr<PointView> Points::createPoint(const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type,
                                               MapView* mapView, MainWindow* mainWindow){
    std::shared_ptr<Point> point = std::shared_ptr<Point>(new Point(pointName, x, y, type));
    std::shared_ptr<PointView> pointView = std::shared_ptr<PointView>(new PointView(point, mapView));
    if(!displayed)
        pointView->hide();

    connect(&(*pointView), SIGNAL(pointLeftClicked(PointView*)), mainWindow, SLOT(displayPointEvent(PointView*)));
    connect(&(*pointView), SIGNAL(editedPointPositionChanged(double, double)), mainWindow, SLOT(updateCoordinates(double, double)));
    connect(&(*pointView), SIGNAL(moveEditedPathPoint()), mainWindow, SLOT(moveEditedPathPointSlot()));
    connect(&(*pointView), SIGNAL(addPointPath(QString, double, double)), mainWindow, SLOT(addPointPathSlot(QString, double, double)));
    connect(&(*pointView), SIGNAL(homeEdited(PointView*)), mainWindow, SLOT(homeEdited(PointView*)));
    connect(&(*pointView), SIGNAL(updatePathPainterPointView()), mainWindow, SLOT(updatePathPainterPointViewSlot()));

    return pointView;
}

void Points::addPoint(const QString groupName, const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type,
                      MapView* mapView, MainWindow* mainWindow){
    qDebug() << "Points::addPoint called";

    std::shared_ptr<PointView> pointView = createPoint(pointName, x, y , displayed, type, mapView, mainWindow);
    addPoint(groupName, pointView);
}

void Points::addPoint(const QString groupName, const std::shared_ptr<PointView> &pointView){
    qDebug() << "Points::addPoint called with pointView";

    if(!groups->empty() && groups->contains(groupName)){
        groups->value(groupName)->push_back(pointView);
    } else {
        std::shared_ptr<QVector<std::shared_ptr<PointView>>> vector = std::shared_ptr<QVector<std::shared_ptr<PointView>>>(new QVector<std::shared_ptr<PointView>>());
        vector->push_back(pointView);
        groups->insert(groupName, vector);
    }
}

void Points::insertPoint(const QString groupName, const int id, const std::shared_ptr<PointView>& pointView){
    qDebug() << "Points::insertPoint called with pointView";

    if(!groups->empty() && groups->contains(groupName)){
        if(groups->value(groupName)->size() > 0)
            groups->value(groupName)->insert(id, pointView);
        else
            groups->value(groupName)->push_back(pointView);
    } else {
        std::shared_ptr<QVector<std::shared_ptr<PointView>>> vector = std::shared_ptr<QVector<std::shared_ptr<PointView>>>(new QVector<std::shared_ptr<PointView>>());
        vector->push_back(pointView);
        groups->insert(groupName, vector);
    }
}

void Points::insertPoint(const QString groupName, const int id, const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type,
                      MapView* mapView, MainWindow* mainWindow){
    qDebug() << "Points::insertPoint called";
    std::shared_ptr<PointView> pointView = createPoint(pointName, x, y , displayed, type, mapView, mainWindow);
    insertPoint(groupName, id, pointView);
}

int Points::count() const {
    int nbPoints = 0;
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        nbPoints += i.value()->size();
    }
    return nbPoints;
}

void Points::displayTmpPoint(const bool display){
    if(groups->value(TMP_GROUP_NAME)->count() > 0 && groups->value(TMP_GROUP_NAME)->at(0) != NULL)
        groups->value(TMP_GROUP_NAME)->at(0)->setVisible(display);
}

void Points::setPointViewsState(const GraphicItemState state){
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        for(int j = 0; j < i.value()->size(); j++){
            i.value()->at(j)->setState(state);
        }
    }
}

std::shared_ptr<PointView> Points::getTmpPointView() const{
    qDebug() << "Points::getTmpPointView called" << this->count();
    if(groups->value(TMP_GROUP_NAME)->count() > 0 && groups->value(TMP_GROUP_NAME)->at(0) != NULL)
        return groups->value(TMP_GROUP_NAME)->at(0);
    return NULL;
}

bool Points::isDisplayed(const QString key) const {
    qDebug() << "Points::isDisplayed called" << key;
    if(!groups->value(key) || groups->value(key)->size() <= 0) {
        return false;
    } else {
        for(int i = 0; i < groups->value(key)->size(); i++){
            if(!groups->value(key)->at(i)->isVisible())
                return false;
        }
    }
    return true;
}

bool Points::isAGroup(const QString groupName) const{
    return groups->contains(groupName);
}

bool Points::isAPoint(const QString pointName) const{
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0)
                    return true;
            }
        }
    }
    return false;
}

QVector<QString> Points::getHomeNameFromGroup(const QString groupName) const{
    QVector<QString> nameVector;
    for(int j = 0; j < groups->value(groupName)->size(); j++){
        if(groups->value(groupName)->at(j)->getPoint()->getType() == Point::PointType::HOME)
            nameVector.push_back(groups->value(groupName)->at(j)->getPoint()->getName());
    }
    return nameVector;
}

QString Points::getGroupNameFromPointName(const QString pointName) const{
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0)
                    return i.key();
            }
        }
    }
    return "";
}

void Points::setPixmapAll(const PointView::PixmapType type){
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    qDebug() << "Points::setPixmapAll called";
    while(i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->count(); j++)
                i.value()->at(j)->setPixmap(type);
        }
    }
}

void Points::setPixmapAll(const QPixmap pixmap){
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*groups);
    qDebug() << "Points::setPixmapAll pixmap version called";
    while (i.hasNext()) {
        i.next();
        if(i.value() && i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->count(); j++){
                i.value()->at(j)->QGraphicsPixmapItem::setPixmap(pixmap);
                i.value()->at(j)->updatePos();
            }
        }
    }
}

void Points::addTmpPoint(MapView *mapView, MainWindow *mainWindow){
    addPoint(TMP_GROUP_NAME, TMP_POINT_NAME, 0, 0, false, Point::PointType::TEMP, mapView, mainWindow);
}
