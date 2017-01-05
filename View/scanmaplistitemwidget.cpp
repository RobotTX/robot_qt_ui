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
#include "View/mergemaplistitemwidget.h"
#include <QFile>
#include "View/custompushbutton.h"

ScanMapListItemWidget::ScanMapListItemWidget(int _id, QString name, QGraphicsScene* _scene) : QWidget(), id(_id), robotName(name), scene(_scene), pixmapItem(NULL){
    initializeMenu();
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
    scanningBtn = new CustomPushButton(QIcon(":/icons/pause.png"),"", this, CustomPushButton::ButtonType::BOTTOM);
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

    topLayout->setContentsMargins(10, 10, 10, 5);
    midLayout->setContentsMargins(0, 0, 0, 0);
    bottomLayout->setContentsMargins(10, 5, 10, 10);
    layout->setContentsMargins(0, 0, 0, 0);

    connect(closeBtn, SIGNAL(clicked()), this, SLOT(closeBtnSlot()));
    connect(rotLineEdit, SIGNAL(textEdited(QString)), this, SLOT(rotLineEditSlot(QString)));
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderSlot(int)));
    connect(scanningBtn, SIGNAL(clicked(bool)), this, SLOT(scanningBtnSlot(bool)));
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

void ScanMapListItemWidget::scanningBtnSlot(bool checked){
    emit playScan(checked, robotName);
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
