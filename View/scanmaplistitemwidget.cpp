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

ScanMapListItemWidget::ScanMapListItemWidget(int _id, QString fileName, QGraphicsScene *scene, bool _fromRobot) : QWidget(), id(_id), fromRobot(_fromRobot){
    initializeMenu(fileName);
    initializeMap(fileName, scene);
}

void ScanMapListItemWidget::initializeMap(QString fileName, QGraphicsScene* scene){
    qDebug() << "ScanMapListItemWidget::initializeMap received a map or robot :" << fileName;
}

void ScanMapListItemWidget::initializeMenu(QString fileName){
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(new QPushButton(fileName, this));
}
