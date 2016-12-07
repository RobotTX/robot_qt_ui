#include "mergemaplistitemwidget.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QGraphicsScene>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include "stylesettings.h"
#include <QDir>
#include <QLineEdit>
#include <QIntValidator>
#include <QDebug>
#include <QGraphicsPixmapItem>

MergeMapListItemWidget::MergeMapListItemWidget(int _id, QString fileName, QGraphicsScene* scene):id(_id){
    initializeMenu(fileName);
    initializeMap(fileName, scene);
}

void MergeMapListItemWidget::initializeMap(QString fileName, QGraphicsScene* scene){
    QImage image = QImage(fileName,"PGM");

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

    QImage croppedImage = image.copy(top, left, bottom - top + 1, right - left + 1);
    QImage newImage = QImage(croppedImage.size(), QImage::Format_ARGB32);

    for(int i = 0; i < croppedImage.width(); i++){
        for(int j = 0; j < croppedImage.height(); j++){
            int color = croppedImage.pixelColor(i, j).red();

            if(color < 205)
                newImage.setPixel(i, j, qRgba(0, 0, 0, 170));
            else if(color > 205)
                newImage.setPixel(i, j, qRgba(255, 255, 255, 170));
            else {
                newImage.setPixel(i, j, qRgba(205, 205, 205, 0));
            }
        }
    }

    /// Create the graphic item of the map
    QPixmap pixmap = QPixmap::fromImage(newImage);
    pixmapItem = new QGraphicsPixmapItem(pixmap);
    /// To drag & drop the map
    pixmapItem->setFlag(QGraphicsItem::ItemIsMovable);
    scene->addItem(pixmapItem);
}

void MergeMapListItemWidget::initializeMenu(QString fileName){
    QVBoxLayout* layout = new QVBoxLayout(this);

    QHBoxLayout* topLayout = new QHBoxLayout();

    int index = fileName.lastIndexOf(QDir::separator());
    QString fileName2 = fileName.remove(0, index+1);

    fileNameLabel = new QLabel(fileName2, this);
    fileNameLabel->setToolTip(fileName2);
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
