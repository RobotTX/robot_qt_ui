#include "points.h"
#include <QDataStream>
#include <QDebug>
#include <QMapIterator>
#include "Controller/mainwindow.h"
#include "Controller/Points/pointscontroller.h"
#include "Controller/Paths/pathscontroller.h"
#include "View/Map/mapview.h"
#include "View/Paths/pathcreationwidget.h"

Points::Points(QObject* parent, MainWindow *_mainWindow) : QObject(parent), mainWindow(_mainWindow), groups(QSharedPointer<Groups>(new Groups())) {}

void Points::addGroup(const QString groupName, QSharedPointer<QVector<QSharedPointer<PointView>>> points){
    groups->insert(groupName, points);
}

void Points::display(std::ostream& stream) const {
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
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

/// removes a group from the vector of groups unless this group is the default group
/// in which case the function does not do anything
void Points::removeGroup(const QString groupName) {
    if(groupName.compare(NO_GROUP_NAME) != 0)
        groups->remove(groupName);
}

/// looks in every group but the path group for the pointview whose point's name is <pointName>
/// and delete it if it exists
void Points::removePoint(const QString pointName) {
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    QString key("");
    int index(-1);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0){
                    key = i.key();
                    index = j;
                }
            }
        }
    }
    if(key.compare("") != 0 && index != -1){
        groups->value(key)->remove(index);
        qDebug() << "Points::removePoint" << pointName << "done";
    } else {
        QMessageBox msgBox;
        msgBox.setText(pointName + " could not be found");
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

QSharedPointer<QVector<QSharedPointer<PointView>>> Points::findGroup(const QString groupName) const {
    return groups->value(groupName);
}

/// finds a pointview that does not belong to a path (using the name of its point)
QSharedPointer<PointView> Points::findPointView(const QString pointName) const {
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0)
                    return i.value()->at(j);
            }
        }
    }
    return QSharedPointer<PointView>();
}

/// finds a pointview that belongs to a path (using the position of its point)
QSharedPointer<PointView> Points::findPathPointView(const double x, const double y) const {
    if(groups->value(PATH_GROUP_NAME)){
        for(int j = 0; j < groups->value(PATH_GROUP_NAME)->size(); j++){
            if(groups->value(PATH_GROUP_NAME)->at(j)->getPoint()->comparePos(x, y))
                return groups->value(PATH_GROUP_NAME)->at(j);
        }
    }
    return QSharedPointer<PointView>();
}

/// sames as findPointView but returns the point instead
QSharedPointer<Point> Points::findPoint(const QString pointName) const {
    qDebug() << "Points::findPoint called" << pointName;
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0)
                    return i.value()->at(j)->getPoint();
            }
        }
    }
    return QSharedPointer<Point>();
}

/// returns the point contained in the group <groupName> at index <indexPoint>, assumes that indexPoint is within the range [0, size_group)
QSharedPointer<Point> Points::findPoint(const QString groupName, const int indexPoint) const {
    return groups->value(groupName)->at(indexPoint)->getPoint();
}

/// finds the pair <QString, int> which represents a couple (group_name, index_in_group) of a given point identified by its name
/// the function does not look into the group of path points
QPair<QString, int> Points::findPointIndexes(const QString pointName) const {
    QPair<QString, int> indexes("", -1);
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
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

/// clears all the groups
void Points::clear(){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        i.value()->clear();
    }
    groups->clear();
}

QSharedPointer<PointView> Points::createPoint(const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type){
    QSharedPointer<Point> point = QSharedPointer<Point>(new Point(pointName, x, y, type));
    QSharedPointer<PointView> pointView = QSharedPointer<PointView>(new PointView(point, mainWindow));

    if(!displayed)
        pointView->hide();

    /// to display the information of the point
    connect(&(*pointView), SIGNAL(pointLeftClicked(QString, double, double)), mainWindow->getPointsController(), SLOT(displayPointEvent(QString, double, double)));

    /// to update the coordinates of point that's being edited
    connect(&(*pointView), SIGNAL(editedPointPositionChanged(double, double)), mainWindow->getPointsController(), SLOT(updateCoordinates(double, double)));

    /// to update a path point
    connect(&(*pointView), SIGNAL(moveEditedPathPoint()), mainWindow, SLOT(moveEditedPathPointSlot()));

    /// to add a path point to a path
    connect(&(*pointView), SIGNAL(addPointPath(QString, double, double)), mainWindow->getPathsController()->getPathCreationWidget(), SLOT(addPathPointSlot(QString, double, double)));

    /// to update the path painter
    connect(&(*pointView), SIGNAL(updatePathPainterPointView()), mainWindow, SLOT(updatePathPainterPointViewSlot()));

    return pointView;
}

void Points::addPoint(const QString groupName, const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type){
    QSharedPointer<PointView> pointView = createPoint(pointName, x, y , displayed, type);
    addPoint(groupName, pointView);
}

/// add the pointview <pointview> to the group <groupName>
void Points::addPoint(const QString groupName, QSharedPointer<PointView> pointView){

    if(!groups->empty() && groups->contains(groupName))
        groups->value(groupName)->push_back(pointView);

    else {
        QSharedPointer<QVector<QSharedPointer<PointView>>> vector = QSharedPointer<QVector<QSharedPointer<PointView>>>(new QVector<QSharedPointer<PointView>>());
        vector->push_back(pointView);
        groups->insert(groupName, vector);
    }
}

/// like addPoint but inserts the point at a particular position given by <id>
void Points::insertPoint(const QString groupName, const int id, QSharedPointer<PointView> pointView){
    qDebug() << "Points::insertPoint called with pointView, groupname" << groupName;
    if(!groups->empty() && groups->contains(groupName))

        (groups->value(groupName)->size() > 0) ? groups->value(groupName)->insert(id, pointView) : groups->value(groupName)->push_back(pointView);

    else {
        QSharedPointer<QVector<QSharedPointer<PointView>>> vector = QSharedPointer<QVector<QSharedPointer<PointView>>>(new QVector<QSharedPointer<PointView>>());
        vector->push_back(pointView);
        groups->insert(groupName, vector);
    }
}

/// same but the point is created on the fly
void Points::insertPoint(const QString groupName, const int id, const QString pointName, const double x, const double y, const bool displayed, const Point::PointType type){
    qDebug() << "Points::insertPoint called";
    QSharedPointer<PointView> pointView = createPoint(pointName, x, y , displayed, type);
    insertPoint(groupName, id, pointView);
}

/// replaces the pointView at position given by <id> in the group <groupName> by the pointview <pointView>
void Points::replacePoint(const QString groupName, const int id, const QSharedPointer<PointView>& pointView){
    qDebug() << "Points::replacePoint called with pointview, groupName and id" << groupName << id;
    if(id >= 0 && id < groups->value(groupName)->size()){
        groups->value(groupName)->removeAt(id);
        groups->value(groupName)->insert(id, pointView);
    }
}

/// counts how many points there are but does not include path points
int Points::count() const {
    int nbPoints(0);
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            nbPoints += i.value()->size();
        }
    }
    return nbPoints;
}

/// shows or displays the temporary point on the map
void Points::displayTmpPoint(const bool display){
    if(groups->value(TMP_GROUP_NAME)->count() > 0 && groups->value(TMP_GROUP_NAME)->at(0) != NULL)
        groups->value(TMP_GROUP_NAME)->at(0)->setVisible(display);
}

/// sets the state <state> for all pointViews
void Points::setPointViewsState(const GraphicItemState state){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        for(int j = 0; j < i.value()->size(); j++)
            i.value()->at(j)->setState(state);
    }
}

QSharedPointer<PointView> Points::getTmpPointView(){
    if(!groups->value(TMP_GROUP_NAME) || groups->value(TMP_GROUP_NAME)->count() < 1)
        addTmpPoint();
    return groups->value(TMP_GROUP_NAME)->at(0);
}

bool Points::isDisplayed(const QString key) const {
    if(!groups->value(key) || groups->value(key)->size() <= 0)
        return false;
    else {
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
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
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

bool Points::isAPoint(const QString pointName, const double x, const double y) const{
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0
                        && i.value()->at(j)->getPoint()->comparePos(x, y))
                    return true;
            }
        }
    }
    return false;
}

bool Points::isAHome(const QString pointName, const double x, const double y) const{
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->size(); j++){
                if(i.value()->at(j)->getPoint()->getName().compare(pointName) == 0
                        && i.value()->at(j)->getPoint()->comparePos(x, y))
                    return i.value()->at(j)->getPoint()->isHome();
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
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
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

/// Update all the pixmaps to the given type except for the path points
void Points::setPixmapAll(const PointView::PixmapType type){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while(i.hasNext()) {
        i.next();
        if(i.key().compare(PATH_GROUP_NAME) != 0){
            for(int j = 0; j < i.value()->count(); j++){
                QSharedPointer<PointView> pointView = i.value()->at(j);
                if(pointView->getPoint()->isHome())
                    qDebug() << pointView->getPoint()->getName();
                pointView->setPixmap(type);
            }
        }
    }
}

void Points::addTmpPoint(){
    /// we initialize the tmp point in -1 0 and not 0, 0 to avoid conflicts with coordinates sent by the robot
    if(groups->value(TMP_GROUP_NAME)){
        if(groups->value(TMP_GROUP_NAME)->size() < 1)
            addPoint(TMP_GROUP_NAME, TMP_POINT_NAME, -1, 0, false, Point::PointType::TEMP);
    } else
        addPoint(TMP_GROUP_NAME, TMP_POINT_NAME, -1, 0, false, Point::PointType::TEMP);
}

void Points::updatePointViews(void){
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        for(int j = 0; j < i.value()->count(); j++)
            i.value()->at(j)->updatePos();
    }
}

QSharedPointer<PointView> Points::findPointViewByPos(const Position &pos){
    QSharedPointer<PointView> pv(0);
    QMapIterator<QString, QSharedPointer<QVector<QSharedPointer<PointView>>>> i(*groups);
    while (i.hasNext()) {
        i.next();
        for(int j = 0; j < i.value()->count(); j++){
            if(i.value()->at(j)->getPoint()->comparePos(pos))
                return i.value()->at(j);
        }
    }
    return pv;
}
