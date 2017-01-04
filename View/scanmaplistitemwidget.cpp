#include "scanmaplistitemwidget.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QGraphicsScene>
#include <QLabel>
#include <QPushButton>
#include "stylesettings.h"
#include <QLineEdit>
#include <QIntValidator>
#include <QDebug>
#include "Controller/mainwindow.h"
#include "View/mergemapgraphicsitem.h"

ScanMapListItemWidget::ScanMapListItemWidget(int _id, QString name, QGraphicsScene *scene) : QWidget(), id(_id), robotName(name){
    initializeMenu();
    initializeMap(scene);
}

void ScanMapListItemWidget::initializeMenu(){
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(new QPushButton(robotName, this));
}

void ScanMapListItemWidget::initializeMap(QGraphicsScene* scene){
    qDebug() << "ScanMapListItemWidget::initializeMap robot :" << robotName;
}
