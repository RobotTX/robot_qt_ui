#include "leftmenuwidget.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include <QScrollArea>
#include "Model/points.h"
#include "View/custompushbutton.h"
#include "View/stylesettings.h"

LeftMenuWidget::LeftMenuWidget(QWidget *parent, QSharedPointer<Points> const& _points): QWidget(parent), points(_points)
{

    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    /// Robot button
    robotBtn = new CustomPushButton(QIcon(":/icons/robot.png"), "Robots", this);
    //robotBtn->setAutoDefault(true);
    robotBtn->setIconSize(normal_icon_size);

    /// Point button
    pointBtn = new CustomPushButton(QIcon(":/icons/coordinates.png"), "Points", this);
    //pointBtn->setAutoDefault(true);
    pointBtn->setIconSize(normal_icon_size);

    /// Map button
    mapBtn = new CustomPushButton(QIcon(":/icons/map.png"), "Map", this);
    //mapBtn->setAutoDefault(true);
    mapBtn->setIconSize(normal_icon_size);

    /// Path button
    pathBtn = new CustomPushButton(QIcon(":/icons/path.png"), "Paths", this);
    //pathBtn->setAutoDefault(true);
    pathBtn->setIconSize(normal_icon_size);

    layout->addWidget(robotBtn);
    layout->addWidget(pointBtn);
    layout->addWidget(mapBtn);
    layout->addWidget(pathBtn);

    hide();

    /*setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);*/

    layout->setAlignment(Qt::AlignTop);
}

void LeftMenuWidget::showEvent(QShowEvent *event){
    points->setPixmapAll(PointView::PixmapType::NORMAL);
    emit resetPathPointViews();
    QWidget::showEvent(event);
}

