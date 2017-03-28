#include <QDebug>
#include "mergemapcontroller.h"

MergeMapController::MergeMapController(QObject *applicationWindow, QObject *parent) : QObject(parent)
{
    QObject* mergeMapWindow = applicationWindow->findChild<QObject*>("mergeMapWindow");

    if(mergeMapWindow){
        /// to add new maps
        connect(mergeMapWindow, SIGNAL(importMap(QString)), this, SLOT(importMap(QString)));
    }
}

void MergeMapController::importMap(QString file){
    qDebug() << "MergeMapController importMap called" << file;
}
