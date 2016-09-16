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

RobotView::RobotView (QSharedPointer<Robot> const &_robot, QGraphicsItem* parent):
    QGraphicsPixmapItem(QPixmap(":/icons/robot_icon.png"), parent), robot(_robot), selected(false), state(GraphicItemState::NO_STATE), shown(true), lastStage(0)
{
    setScale(0.07);
    setAcceptHoverEvents(true);
    setAcceptedMouseButtons(Qt::LeftButton);
    setToolTip(robot->getName());

    mapView = static_cast<MapView*> (parent);
}

RobotView::RobotView (QGraphicsItem* parent): QGraphicsPixmapItem(parent), selected(false), state(GraphicItemState::NO_STATE), shown(true), lastStage(0)
{
}

void RobotView::mousePressEvent(QGraphicsSceneMouseEvent * /* unused */){

    if(state == GraphicItemState::NO_STATE){
        qDebug() << "map robot clicked";
        MainWindow* mw = static_cast<MainWindow*> (mapView->getMainWindow());
        mw->resetFocus();
        qDebug() << "robot map pressed";
        emit setSelectedSignal(this);
    } else if(state == GraphicItemState::ROBOT_CREATING_PATH){
        qDebug() << "Clicked on a robot while creating a path";
    } else {
        qDebug() << "(RobotView) NO EVENT";
    }
}

void RobotView::hoverEnterEvent(QGraphicsSceneHoverEvent * /* unused */){

    setToolTip(robot->getName());
    setPixmap(QPixmap(":/icons/hover_robot_icon.png"));
}

void RobotView::hoverLeaveEvent(QGraphicsSceneHoverEvent * /* unused */){
    if(selected)
        setPixmap(QPixmap(":/icons/selected_robot_icon.png"));
    else
        setPixmap(QPixmap(":/icons/robot_icon.png"));
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
    if(selected)
        setPixmap(QPixmap(":/icons/selected_robot_icon.png"));
    else
        setPixmap(QPixmap(":/icons/robot_icon.png"));
}

void RobotView::display(const bool _show){
    shown = _show;
    (shown) ? show(): hide();
}

void RobotView::setOrientation(const float ori){
    setRotation(ori);
    robot->setOrientation(ori);

}
