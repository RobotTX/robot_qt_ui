#include "robotview.h"
#include <QBrush>
#include <QPen>
#include <QGraphicsPolygonItem>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include "mainwindow.h"
#include <QGraphicsWidget>

RobotView::RobotView (QPointer<Robot> _robot, QPointer<MapView> parent):
    QGraphicsPixmapItem(QPixmap(":/icons/final_robot.png"), parent), robot(_robot), selected(false), state(GraphicItemState::NO_STATE), shown(true), lastStage(0)
{
    setScale(0.07);
    /// so that the pixmap rotates around its center and not about its top left corner
    setTransformOriginPoint(pixmap().width()/2, pixmap().height()/2);
    setAcceptHoverEvents(true);
    setAcceptedMouseButtons(Qt::LeftButton);
    setToolTip(robot->getName());

    mapView = parent;
}

RobotView::RobotView (QPointer<MapView> parent): QGraphicsPixmapItem(parent), selected(false), state(GraphicItemState::NO_STATE), shown(true), lastStage(0) {}

void RobotView::mousePressEvent(QGraphicsSceneMouseEvent * /* unused */){

    if(state == GraphicItemState::NO_STATE){
        qDebug() << "map robot clicked";
        MainWindow* mw = static_cast<MainWindow*> (mapView->getMainWindow());
        mw->resetFocus();
        qDebug() << "robot map pressed";
        emit setSelectedSignal(this);
    } else if(state == GraphicItemState::CREATING_PATH)
        qDebug() << "Clicked on a robot while creating a path";
    else
        qDebug() << "(RobotView) NO EVENT";
}

void RobotView::hoverEnterEvent(QGraphicsSceneHoverEvent * /* unused */){

    setToolTip(robot->getName());
    setPixmap(QPixmap(":/icons/final_robot_hover"));
}

void RobotView::hoverLeaveEvent(QGraphicsSceneHoverEvent * /* unused */){
    if(selected)
        setPixmap(QPixmap(":/icons/final_robot_selected.png"));
    else
        setPixmap(QPixmap(":/icons/final_robot.png"));
}

void RobotView::setPosition(const Position _position) {
    /// we have to substract half the size of the pixmap because of setTransformOriginPoint(pixmap().width()/2, pixmap().height()/2);
    /// don't really know why
    setPos(_position.getX()-pixmap().width()/2, _position.getY()-pixmap().height()/2);
    robot->setPosition(_position.getX(), _position.getY());
}

void RobotView::setPosition(const float x, const float y) {
    setPos(x-pixmap().width()/2, y-pixmap().height()/2);
    robot->setPosition(x, y);
}

void RobotView::setSelected(const bool _selected){
    selected = _selected;
    (selected) ? setPixmap(QPixmap(":/icons/final_robot_selected.png")) : setPixmap(QPixmap(":/icons/final_robot.png"));
}

void RobotView::display(const bool _show){
    shown = _show;
    (shown) ? show(): hide();
}

void RobotView::setOrientation(const float ori){
    setRotation(ori);
    robot->setOrientation(ori);
}
