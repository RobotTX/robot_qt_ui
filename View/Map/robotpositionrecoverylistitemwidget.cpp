#include "robotpositionrecoverylistitemwidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QDebug>
#include <QGraphicsScene>
#include <QLabel>
#include "View/Map/recoverpositionmapgraphicsitem.h"
#include "View/Other/stylesettings.h"

RobotPositionRecoveryListItemWidget::RobotPositionRecoveryListItemWidget(const int _id, const QString name, QSharedPointer<Robots> robots, QGraphicsScene *_scene)
    : QWidget(), id(_id), robotName(name), scene(_scene), pixmapItem(new RecoverPositionMapGraphicsItem(name)), top(0), left(0)
{
    QVBoxLayout* layout = new QVBoxLayout(this);

    QHBoxLayout* topLayout = new QHBoxLayout();

    /// Warning button to tell the user that the robot disconnected
    disconnectedIcon = new QPushButton(QIcon(":/icons/disconnect.png"), "", this);
    disconnectedIcon->setFlat(true);
    disconnectedIcon->setIconSize(xxs_icon_size);
    disconnectedIcon->setStyleSheet(disconnectedIcon->styleSheet() + "QPushButton {padding-top: 15px; padding-bottom: 15px;}");
    disconnectedIcon->setToolTip("The robot is disconnected.");
    disconnectedIcon->setMaximumWidth(xxs_icon_size.width() + 7);
    disconnectedIcon->hide();
    topLayout->addWidget(disconnectedIcon, Qt::AlignLeft);

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

    /// Btn to play/pause a scan
    QHBoxLayout* recoverPositionLayout = new QHBoxLayout();
    recoverPositionBtn = new QPushButton(QIcon(":/icons/pause.png"),"", this);
    recoverPositionBtn->setFlat(true);
    recoverPositionLayout->addWidget(recoverPositionBtn, Qt::AlignLeft);
    recoverPositionLabel = new QLabel("Recovering the position");
    recoverPositionLayout->addWidget(recoverPositionLabel, Qt::AlignLeft);

    layout->addLayout(recoverPositionLayout);

    topLayout->setContentsMargins(10, 0, 10, 0);
    recoverPositionLayout->setContentsMargins(10, 0, 10, 5);
    layout->setContentsMargins(0, 0, 0, 0);

    connect(closeBtn, SIGNAL(clicked()), this, SLOT(closeBtnSlot()));
    connect(pixmapItem, SIGNAL(robotGoTo(double, double)), this, SLOT(robotGoToSlot(double, double)));
    connect(recoverPositionBtn, SIGNAL(clicked()), this, SLOT(startRecoverySlot()));

    scene->addItem(pixmapItem);
}

void RobotPositionRecoveryListItemWidget::robotConnected(const bool connected){
    (connected) ? disconnectedIcon->hide() : disconnectedIcon->show();
}

void RobotPositionRecoveryListItemWidget::closeBtnSlot(){
    qDebug() << "RobotPositionRecoveryListItemWidget::closeBtnSlot called";
    emit deleteMap(id, robotName);
}

void RobotPositionRecoveryListItemWidget::startRecoverySlot(){
    bool startingToRecover = !recoverPositionLabel->text().compare("Recovering the position");
    emit playRecovery(startingToRecover, robotName);
}

/// updates the icon
void RobotPositionRecoveryListItemWidget::robotRecovering(const bool recovering){
    if(recovering){
        recoverPositionBtn->setIcon(QIcon(":/icons/pause.png"));
        recoverPositionLabel->setText("Recovering the position");
    } else {
        recoverPositionBtn->setIcon(QIcon(":/icons/play.png"));
        recoverPositionLabel->setText("Not recovering the position");
    }
}

void RobotPositionRecoveryListItemWidget::updateMap(const QImage& map){
    QPixmap pixmap = QPixmap::fromImage(cropImage(map));
    pixmapItem->setPixmap(pixmap);
    /// we have received a map so no reason to show the warning icon anymore
    if(!warningIcon->isHidden()){
        warningIcon->hide();
        /// center on the map that we just received
        emit centerOn(pixmapItem->getRobotView());
    }
}

void RobotPositionRecoveryListItemWidget::robotGoToSlot(double x, double y){
    /// Tell the robot where to go, we add left and top as they are the padding we removed when cropping the map
    emit robotGoTo(robotName, x + left, y + top);
}

void RobotPositionRecoveryListItemWidget::updateRobotPos(double x, double y, double ori){
    /// Got the new position of the robot, minus the padding we removed when cropping the map
    pixmapItem->updateRobotPos(x - left, y - top, ori);
}

QImage RobotPositionRecoveryListItemWidget::cropImage(const QImage &image) {
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
    QRgb wallColor = (id % 2 == 0) ? qRgba(0, 0, 255, 170) : qRgba(0, 255, 0, 170);
    for(int i = 0; i < croppedImage.width(); i++){
        for(int j = 0; j < croppedImage.height(); j++){
            int color = croppedImage.pixelColor(i, j).red();
            if(color < 205)
                newImage.setPixel(i, j, wallColor);
            else if(color > 205)
                newImage.setPixel(i, j, qRgba(255, 255, 255, 170));
        }
    }

    return newImage;
}

/// when the item is clicked the scene centers on the corresponding robot
void RobotPositionRecoveryListItemWidget::mouseDoubleClickEvent(QMouseEvent*){
    qDebug() << "RobotPositionRecoveryListItemWidget::mouseDoubleClickEvent";
    emit centerOn(pixmapItem->getRobotView());
}
