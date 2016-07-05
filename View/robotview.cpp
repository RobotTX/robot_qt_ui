#include "robotview.h"
#include "Model/robot.h"
#include <QBrush>
#include <QPen>
#include <QGraphicsPolygonItem>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include "mainwindow.h"
#include <QGraphicsWidget>
#include "mapview.h"

RobotView::RobotView (std::shared_ptr<Robot> const &_robot, QGraphicsItem* parent):state(GraphicItemState::NO_STATE), selected(false), QGraphicsPolygonItem(parent)
{
    robot = _robot;
    setAcceptHoverEvents(true);
    setAcceptedMouseButtons(Qt::LeftButton);
    setToolTip(robot->getName());

    /// We create a polygon of the robot by writting each points of the polygon
    QPolygonF polygon;
    polygon << QPointF(ROBOT_WIDTH/2, ROBOT_HEIGHT_HIGH) << QPointF(0, ROBOT_HEIGHT_LOW)
               << QPointF(ROBOT_WIDTH/2, 0) << QPointF(ROBOT_WIDTH, ROBOT_HEIGHT_LOW);
    setPolygon(polygon);
    /*QLinearGradient linearGrad(QPointF(ROBOT_WIDTH/2, ROBOT_HEIGHT_HIGH), QPointF(ROBOT_WIDTH/2, 0));
    linearGrad.setColorAt(0, Qt::red);
    linearGrad.setColorAt(0.5, Qt::red);
    linearGrad.setColorAt(1, Qt::black);
    setBrush(QBrush(linearGrad));*/
    /// Brush = the inner part of the polygon
    setBrush(QBrush(Qt::red));

    /// Pen = the border of the polygon
    if(selected){
        setPen(QPen(Qt::green));
    } else {
        setPen(QPen(Qt::red));
    }
    shown = true;
    mapView =(MapView*) parent;
}

RobotView::RobotView (QGraphicsItem* parent):selected(false), state(GraphicItemState::NO_STATE),
    QGraphicsPolygonItem(parent){
}

void RobotView::mousePressEvent(QGraphicsSceneMouseEvent *event){

    if(state == GraphicItemState::NO_STATE){
       qDebug() << "map robot clicked";
       // MainWindow* mw = (MainWindow*)(((MapView*)(this ->parentWidget()))->getMainWindow());
        MainWindow* mw = (MainWindow*)(mapView->getMainWindow());
        mw->resetFocus();
        qDebug() << "robot map pressed";
        emit setSelectedSignal(this);


    } else if(state == GraphicItemState::CREATING_PATH){
        qDebug() << "Clicked on a robot while creating a path";
    } else {
        qDebug() << "(RobotView) NO EVENT";
    }
}

void RobotView::hoverEnterEvent(QGraphicsSceneHoverEvent *event){
    setToolTip(robot->getName());
    setPen(QPen(Qt::yellow));
}

void RobotView::hoverLeaveEvent(QGraphicsSceneHoverEvent *event){
    if(selected){
        setPen(QPen(Qt::green));
    } else {
        setPen(QPen(Qt::red));
    }
}

void RobotView::setPosition(const Position _position) {
    setPos(_position.getX(), _position.getY());
    robot->setPosition(_position.getX(), _position.getY());
}

void RobotView::setPosition(const float x, const float y) {
    setPos(x, y);
    robot->setPosition(x, y);
}

void RobotView::setSelected(const bool _selected){
    selected = _selected;
    if(selected){
        setPen(QPen(Qt::green));
    } else {
        setPen(QPen(Qt::red));
    }
}

void RobotView::display(const bool _show){
    shown = _show;
    if(shown){
        show();
    } else {
        hide();
    }
    qDebug() << "Show : " << shown;
}



void RobotView::setOrientation(const float ori){
    setRotation(ori);
    robot->setOrientation(ori);

}
