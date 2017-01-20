#include "scanmapgraphicsitem.h"
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QPainter>
#include "Model/robots.h"
#include "Model/robot.h"
#include "View/robotview.h"
#include <QImage>
#include <QtMath>

ScanMapGraphicsItem::ScanMapGraphicsItem(QString _robotName, QSharedPointer<Robots> _robots)
    : QGraphicsPixmapItem(), robotName(_robotName), robots(_robots){

    /// Tell the class which mouse button to accept
    setAcceptedMouseButtons(Qt::LeftButton | Qt::MidButton);
    /// To drag & drop the map
    setFlag(QGraphicsItem::ItemIsMovable);

    /// The view of the robot on the map
    scanRobotView = new QGraphicsPixmapItem(QPixmap(":/icons/final_robot.png"), this);
    scanRobotView->setScale(0.07);
    scanRobotView->setTransformOriginPoint(scanRobotView->pixmap().width()/2, scanRobotView->pixmap().height()/2);
    scanRobotView->setToolTip(robotName);
    scanRobotView->setZValue(2);

    laserView = new QGraphicsPixmapItem(this);
    laserView->setZValue(3);

    QPointer<RobotView> robotView = robots->getRobotViewByName(robotName);
    if(robotView)
        connect(robotView, SIGNAL(updateLaser()), this, SLOT(updateLaserSLot()));
}

void ScanMapGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if(event->button() == Qt::LeftButton){
        /// On mouse press event, we set the drag start position
        dragStartPosition = this->pos();
        QGraphicsPixmapItem::mousePressEvent(event);

    } else if(event->button() == Qt::MidButton)
        emit robotGoTo(event->pos().x(), event->pos().y());
}

void ScanMapGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(event->button() == Qt::LeftButton){
        float x = dragStartPosition.x() - this->pos().x();
        float y = dragStartPosition.y() - this->pos().y();

        /// we compare the start position of the drag event & the drop position
        /// if we have moved for more than 1 pixel, it's a drag, else it's a click
        /// and we create a temporary point
        if (abs(x) <= 1 && abs(y) <= 1)
            emit pixmapClicked();

        /// drag and drop
        QGraphicsPixmapItem::mouseReleaseEvent(event);
    }
}

void ScanMapGraphicsItem::updateRobotPos(double x, double y, double ori){
    scanRobotView->setRotation(ori);
    scanRobotView->setPos(x-scanRobotView->pixmap().width()/2, y-scanRobotView->pixmap().height()/2);
}

void ScanMapGraphicsItem::updateLaserSLot(){

    QPointer<RobotView> robotView = robots->getRobotViewByName(robotName);
    if(robotView){
        QVector<QPointF> obstacles = robotView->getObstacles();
        int minX = 0;
        int maxX = 0;
        int minY = 0;
        int maxY = 0;
        for(int i = 0; i < obstacles.size(); i++){
            QPointF point = obstacles.at(i);
            if(point.x() < 1000 && point.x() > -1000 && point.y() < 1000 && point.y() > -1000){
                if(point.x() > maxX)
                    maxX = point.x();
                if(point.x() < minX)
                    minX = point.x();
                if(point.y() > maxY)
                    maxY = point.y();
                if(point.y() < minY)
                    minY = point.y();
            }
        }

        QImage laserImage = QImage(maxX - minX + 1, maxY - minY + 1, QImage::Format_ARGB32);
        laserImage.fill(qRgba(205, 205, 205, 0));

        for(int j = 0; j < obstacles.size(); j++)
            if(obstacles.at(j).x() < 1000 && obstacles.at(j).x() > -1000 && obstacles.at(j).y() < 1000 && obstacles.at(j).y() > -1000)
                laserImage.setPixel(obstacles.at(j).x() - minX,
                                         obstacles.at(j).y() - minY, qRgba(255, 0, 0, 255));

        double posX = sqrt(37) * cos(atan(-1.0/6.0) + (scanRobotView->rotation() - 90)/180.0*3.14159);
        double posY = sqrt(37) * sin(atan(-1.0/6.0) + (scanRobotView->rotation() - 90)/180.0*3.14159);


        laserImage.setPixel(-minX + posX, -minY + posY, qRgba(0, 255, 0, 255));

        laserView->setPixmap(QPixmap::fromImage(laserImage));
        laserView->setTransformOriginPoint(-minX, -minY);
/*
        qDebug() << "ScanMapGraphicsItem::updateLaserSLot"
                 << "\n1:" << minX << maxX << minY << maxY
                 << "\n2:" << maxX - minX << maxY - minY
                 << "\n3:" << this->boundingRect()
                 << "\n4:" << this->pos()
                 << "\n5:" << scanRobotView->pos()
                 << "\n6:" << laserView->pos()
                 << "\n7:" << laserView->mapFromItem(scanRobotView, 0, 0)
                 << "\n8:" << scanRobotView->rotation()
                 << "\n9:" << atan(-1.0/6.0)
                 << "\n10:" << cos(atan(-1.0/6.0))
                 << "\n11:" << sin(atan(-1.0/6.0))
                 << "\n12:" << sin(-9.46233)
                 << "\n13:" << posX << posY;
*/
        /*laserView->setPos(laserView->pos().x() + laserView->mapFromItem(scanRobotView, 0, 0).x() - (maxX - minX)/2,
                          laserView->pos().y() + laserView->mapFromItem(scanRobotView, 0, 0).y() - (maxY - minY)/2);*/
        laserView->setPos(scanRobotView->pos().x() + scanRobotView->pixmap().size().width()/2 + minX + posX,
                          scanRobotView->pos().y() + scanRobotView->pixmap().size().height()/2 + minY + posY);
    }
}
