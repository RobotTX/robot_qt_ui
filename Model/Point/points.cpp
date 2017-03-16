#include "points.h"
#include <QDataStream>
#include <QDebug>
#include <QString>
#include "Model/Point/point.h"

Points::Points(QObject* parent) : QObject(parent), groups(QMap<QString, QVector<QSharedPointer<Point>>>()) {}

void Points::resetGroups(){
    groups = QMap<QString, QVector<QSharedPointer<Point>>>();
}

QSharedPointer<Point> Points::deletePointFromGroup(const QString group, const QString point_name){
    /// we remove the point from the c++ side
    for(int i = 0; i < groups[group].size(); i++){
        if(groups[group].at(i)->getName().compare(point_name) == 0){
            return groups[group].takeAt(i);
        }
    }
    Q_UNREACHABLE();
    return QSharedPointer<Point>();
}

void Points::display() const {
    QMapIterator<QString, QVector<QSharedPointer<Point> >> it(groups);
    while(it.hasNext()){
        it.next();
        qDebug() << "Group : " << it.key();
        for(int i = 0; i < it.value().size(); i++)
            qDebug() << "\tPoint :" << it.value().at(i)->getName() << it.value().at(i)->getX() << it.value().at(i)->getY();
    }
}
