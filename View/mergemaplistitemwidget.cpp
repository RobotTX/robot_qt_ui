#include "mergemaplistitemwidget.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QGraphicsScene>
#include <QLabel>
#include <QPushButton>
#include "stylesettings.h"
#include <QDir>
#include <QLineEdit>
#include <QIntValidator>
#include <QDebug>
#include <QGraphicsPixmapItem>
#include <fstream>
#include "Controller/mainwindow.h"
#include "View/mergemapgraphicsitem.h"

MergeMapListItemWidget::MergeMapListItemWidget(int _id, QString fileName, QGraphicsScene* scene, bool _fromRobot, QImage image, double _resolution, double _originX, double _originY):
    id(_id), origin(QPointF(_originX, _originY)), resolution(_resolution), originInPixel(QPoint(-1, -1)), fromRobot(_fromRobot), pixmapItem(new MergeMapGraphicsItem()){

    connect(pixmapItem, SIGNAL(pixmapClicked()), this, SLOT(pixmapClickedSlot()));
    initializeMenu(fileName);
    initializeMap(fileName, scene, image);
}

void MergeMapListItemWidget::initializeMenu(QString fileName){
    QVBoxLayout* layout = new QVBoxLayout(this);

    QHBoxLayout* topLayout = new QHBoxLayout();

    if(!fromRobot){
        int index = fileName.lastIndexOf(QDir::separator());
        if(index != -1)
            fileName = fileName.remove(0, index+1);

        QString str = QDir::currentPath() + QDir::separator() + "mapConfigs" + QDir::separator() + fileName.remove(fileName.size()-4, 4) + ".config";
        qDebug() << "MergeMapListItemWidget::initializeMenu Trying to find a map config at" << str;
        std::ifstream mapConfig(str.toStdString(), std::ios::in);
        if(mapConfig.is_open()){
            double originX, originY;
            std::string osef;

            mapConfig >> osef >> osef >> osef >> osef >> osef >> osef >> originX >> originY >> resolution;

            origin = QPointF(originX, originY);

            mapConfig.close();

            qDebug() << "MergeMapListItemWidget::initializeMenu Got a resolution and origin" << resolution << origin;
        } else
            qDebug() << "MergeMapListItemWidget::initializeMenu no config file found for the map" << fileName;
    } else
        qDebug() << "MergeMapListItemWidget::initializeMenu Got a resolution and origin from the robot" << resolution << origin;

    fileNameLabel = new QLabel(fileName, this);
    fileNameLabel->setToolTip(fileName);
    topLayout->addWidget(fileNameLabel, Qt::AlignLeft);

    closeBtn = new QPushButton(QIcon(":/icons/close.png"), "", this);
    closeBtn->setFlat(true);
    closeBtn->setIconSize(xxs_icon_size);
    closeBtn->setStyleSheet(closeBtn->styleSheet() + "QPushButton {padding-top: 15px; padding-bottom: 15px;}");
    closeBtn->setMaximumWidth(xxs_icon_size.width() + 7);
    topLayout->addWidget(closeBtn, Qt::AlignRight);

    layout->addLayout(topLayout);


    QVBoxLayout* bottomLayout = new QVBoxLayout();
    QHBoxLayout* midLayout = new QHBoxLayout();

    QLabel* labelRot = new QLabel("Rotation :", this);
    midLayout->addWidget(labelRot, Qt::AlignLeft);

    rotLineEdit = new QLineEdit("0", this);
    rotLineEdit->setValidator(new QIntValidator(0, 360, this));
    midLayout->addWidget(rotLineEdit, Qt::AlignRight);

    bottomLayout->addLayout(midLayout);
    slider = new QSlider(Qt::Orientation::Horizontal, this);
    slider->setMinimum(0);
    slider->setMaximum(360);
    slider->setTracking(true);
    bottomLayout->addWidget(slider);
    layout->addLayout(bottomLayout);

    topLayout->setContentsMargins(10, 10, 10, 5);
    midLayout->setContentsMargins(0, 0, 0, 0);
    bottomLayout->setContentsMargins(10, 5, 10, 10);
    layout->setContentsMargins(0, 0, 0, 0);

    connect(closeBtn, SIGNAL(clicked()), this, SLOT(closeBtnSlot()));
    connect(rotLineEdit, SIGNAL(textEdited(QString)), this, SLOT(rotLineEditSlot(QString)));
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderSlot(int)));
}

void MergeMapListItemWidget::initializeMap(QString fileName, QGraphicsScene* scene, QImage image){
    if(!fromRobot)
        image = QImage(fileName,"PGM");


    int top = image.height();
    int bottom = 0;
    int left = image.width();
    int right = 0;

    for(int i = 0; i < image.height(); i++){
        for(int j = 0; j < image.width(); j++){
            int color = image.pixelColor(i, j).red();
            if(color == 255 || color == 0){
                if(top > i)
                    top = i;
                if(left > j)
                    left = j;
                if(bottom < i)
                    bottom = i;
                if(right < j)
                    right = j;
            }
        }
    }

    if(resolution != -1){
        Position pos = MainWindow::convertRobotCoordinatesToPixelCoordinates(Position(0, 0), origin.x(), origin.y(), resolution, image.height(), 0);
        originInPixel = QPoint(pos.getX(), pos.getY());
        pixmapItem->setZValue(id+1);
        qDebug() << "MergeMapListItemWidget::initializeMap origin vs originInPixel :" << origin << "vs" << originInPixel;
    } else
        pixmapItem->setZValue(0);


    QImage croppedImage = image.copy(top, left, bottom - top + 1, right - left + 1);
    QImage newImage = QImage(croppedImage.size(), QImage::Format_ARGB32);
    newImage.fill(qRgba(205, 205, 205, 0));

    QRgb wallColor = (id % 2 == 0) ? qRgba(255, 0, 0, 170) : qRgba(0, 255, 0, 170);
    for(int i = 0; i < croppedImage.width(); i++){
        for(int j = 0; j < croppedImage.height(); j++){
            int color = croppedImage.pixelColor(i, j).red();

            if(color < 205)
                newImage.setPixel(i, j, wallColor);
            else if(color > 205)
                newImage.setPixel(i, j, qRgba(255, 255, 255, 170));
        }
    }

    if(resolution != -1)
        newImage.setPixel(originInPixel.x() - top, originInPixel.y() - left, qRgba(0, 0, 255, 170));


    /// Create the graphic item of the map
    QPixmap pixmap = QPixmap::fromImage(newImage);
    pixmapItem->setPixmap(pixmap);

    scene->addItem(pixmapItem);
}

void MergeMapListItemWidget::closeBtnSlot(){
    qDebug() << "MergeMapListItemWidget::closeBtnSlot called";
    emit deleteMap(id);
}

void MergeMapListItemWidget::rotLineEditSlot(QString text){
    /// will call the slot of the slider
    slider->setValue(text.toInt());
}

void MergeMapListItemWidget::sliderSlot(int value){
    rotLineEdit->setText(QString::number(value));
    pixmapItem->setRotation(value);
}


void MergeMapListItemWidget::pixmapClickedSlot(){
    emit pixmapClicked(id);
}
