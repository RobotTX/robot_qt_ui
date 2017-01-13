#include "scanmaplistitemwidget.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QGraphicsScene>
#include <QLabel>
#include <QPushButton>
#include "stylesettings.h"
#include <QLineEdit>
#include <QIntValidator>
#include <QDebug>
#include "Controller/mainwindow.h"
#include "View/scanmapgraphicsitem.h"
#include "View/mergemaplistitemwidget.h"
#include <QFile>

ScanMapListItemWidget::ScanMapListItemWidget(int _id, QString name, QGraphicsScene* _scene)
    : QWidget(), id(_id), robotName(name), scene(_scene), pixmapItem(new ScanMapGraphicsItem(name)),
      oriWidth(0), oriHeight(0), newWidth(0), newHeight(0), top(0), left(0){

    initializeMenu();

    QPixmap pixmap;
    pixmapItem->setPixmap(pixmap);
    connect(pixmapItem, SIGNAL(robotGoTo(double,double)), this, SLOT(robotGoToSlot(double,double)));

    scene->addItem(pixmapItem);
}

void ScanMapListItemWidget::initializeMenu(){
    QVBoxLayout* layout = new QVBoxLayout(this);

    QHBoxLayout* topLayout = new QHBoxLayout();


    /// Warning button to tell the user that the robot disconnected
    discoIcon = new QPushButton(QIcon(":/icons/disconnect.png"), "", this);
    discoIcon->setFlat(true);
    discoIcon->setIconSize(xxs_icon_size);
    discoIcon->setStyleSheet(discoIcon->styleSheet() + "QPushButton {padding-top: 15px; padding-bottom: 15px;}");
    discoIcon->setToolTip("The robot is disconnected.");
    discoIcon->setMaximumWidth(xxs_icon_size.width() + 7);
    discoIcon->hide();
    topLayout->addWidget(discoIcon, Qt::AlignLeft);

    /// Warning button to tell the user that we haven't receive the map yet
    warningIcon = new QPushButton(QIcon(":/icons/warning.png"), "", this);
    warningIcon->setFlat(true);
    warningIcon->setIconSize(xxs_icon_size);
    warningIcon->setStyleSheet(warningIcon->styleSheet() + "QPushButton {padding-top: 15px; padding-bottom: 15px;}");
    warningIcon->setToolTip("Haven't received a map from this robot yet.");
    warningIcon->setMaximumWidth(xxs_icon_size.width() + 7);
    topLayout->addWidget(warningIcon, Qt::AlignLeft);

    /// Label with the name of the map imported
    fileNameLabel = new QLabel(robotName, this);
    fileNameLabel->setToolTip(robotName);
    topLayout->addWidget(fileNameLabel, Qt::AlignLeft);

    /// Btn to remove the map from the list
    closeBtn = new QPushButton(QIcon(":/icons/close.png"), "", this);
    closeBtn->setFlat(true);
    closeBtn->setIconSize(xxs_icon_size);
    closeBtn->setStyleSheet(closeBtn->styleSheet() + "QPushButton {padding-top: 15px; padding-bottom: 15px;}");
    closeBtn->setToolTip("Delete this map");
    closeBtn->setMaximumWidth(xxs_icon_size.width() + 7);
    topLayout->addWidget(closeBtn, Qt::AlignRight);

    layout->addLayout(topLayout);

    QHBoxLayout* scanningLayout = new QHBoxLayout();
    scanningBtn = new QPushButton(QIcon(":/icons/pause.png"),"", this);
    scanningBtn->setFlat(true);
    scanningLayout->addWidget(scanningBtn, Qt::AlignLeft);
    scanningLabel = new QLabel("Scanning");
    scanningLayout->addWidget(scanningLabel, Qt::AlignLeft);

    layout->addLayout(scanningLayout);

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


    topLayout->setContentsMargins(10, 0, 10, 0);
    scanningLayout->setContentsMargins(10, 0, 10, 5);
    midLayout->setContentsMargins(0, 0, 0, 0);
    bottomLayout->setContentsMargins(10, 0, 10, 10);
    layout->setContentsMargins(0, 0, 0, 0);

    connect(closeBtn, SIGNAL(clicked()), this, SLOT(closeBtnSlot()));
    connect(rotLineEdit, SIGNAL(textEdited(QString)), this, SLOT(rotLineEditSlot(QString)));
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderSlot(int)));
    connect(scanningBtn, SIGNAL(clicked()), this, SLOT(scanningBtnSlot()));
}

void ScanMapListItemWidget::robotConnected(bool connected){
    if(connected)
        discoIcon->hide();
    else
        discoIcon->show();
}

void ScanMapListItemWidget::closeBtnSlot(){
    qDebug() << "MergeMapListItemWidget::closeBtnSlot called";
    emit deleteMap(id, robotName);
}

void ScanMapListItemWidget::rotLineEditSlot(QString text){
    /// will call the sliderSlot and rotate the map
    slider->setValue(text.toInt());
}

void ScanMapListItemWidget::sliderSlot(int value){
    rotLineEdit->setText(QString::number(MergeMapListItemWidget::mod(value, 360)));
    /// rotate the map
    if(pixmapItem)
        pixmapItem->setRotation(value);
}

void ScanMapListItemWidget::scanningBtnSlot(){
    bool startingToScan = !scanningLabel->text().compare("Not scanning");
    emit playScan(startingToScan, robotName);
}

void ScanMapListItemWidget::robotScanning(bool scanning){
    if(scanning){
        scanningBtn->setIcon(QIcon(":/icons/pause.png"));
        scanningLabel->setText("Scanning");
    } else {
        scanningBtn->setIcon(QIcon(":/icons/play.png"));
        scanningLabel->setText("Not scanning");
    }
}

void ScanMapListItemWidget::updateMap(QImage map){
    QPixmap pixmap = QPixmap::fromImage(cropImage(map));
    pixmapItem->setPixmap(pixmap);

    if(!warningIcon->isHidden()){
        warningIcon->hide();
        emit centerOn(pixmapItem->getRobotView());
    }
}

void ScanMapListItemWidget::robotGoToSlot(double x, double y){
    emit robotGoTo(robotName, x + left, y + top);
}

void ScanMapListItemWidget::updateRobotPos(double x, double y, double ori){
    pixmapItem->updateRobotPos(x - left, y - top, ori);
}

QImage ScanMapListItemWidget::cropImage(QImage image){
    oriWidth = image.width();
    oriHeight = image.height();
    left = image.width();
    int right = 0;
    top = image.height();
    int bottom = 0;

    /// We want to find the smallest rectangle containing the map (white and black) to crop it and use a small image
    for(int i = 0; i < image.height(); i++){
        for(int j = 0; j < image.width(); j++){
            int color = image.pixelColor(i, j).red();
            if(color == 255 || color == 0){
                if(left > i)
                    left = i;
                if(top > j)
                    top = j;
                if(right < i)
                    right = i;
                if(bottom < j)
                    bottom = j;
            }
        }
    }

    /// We crop the image
    QImage croppedImage = image.copy(left, top, right - left + 1, bottom - top + 1);
    /// Create a new image filled with invisible grey
    QImage newImage = QImage(croppedImage.size(), QImage::Format_ARGB32);
    newImage.fill(qRgba(205, 205, 205, 0));

    /// 1 out of 2 map will have red wall and the other one green wall to better distinguish them
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

    newWidth = newImage.width();
    newHeight = newImage.height();

    return newImage;
}

void ScanMapListItemWidget::mouseDoubleClickEvent(QMouseEvent*){
    qDebug() << "ScanMapListItemWidget::mouseDoubleClickEvent";
    emit centerOn(pixmapItem->getRobotView());
}
