#include "leftmenuwidget.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include <QScrollArea>
#include "Model/points.h"
#include "View/custompushbutton.h"
#include "View/stylesettings.h"

LeftMenuWidget::LeftMenuWidget(QWidget *_parent, QSharedPointer<Points> const& _points): QWidget(_parent), points(_points)
{
    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    /// Point button
    pointBtn = new CustomPushButton(QIcon(":/icons/coordinates.png"), "Points", this);
    pointBtn->setIconSize(s_icon_size);

    /// Path button
    pathBtn = new CustomPushButton(QIcon(":/icons/path.png"), "Paths", this);
    pathBtn->setIconSize(m_icon_size);

    /// Robot button
    robotBtn = new CustomPushButton(QIcon(":/icons/robot.png"), "Robots", this);
    robotBtn->setIconSize(s_icon_size);

    /// Map button
    mapBtn = new CustomPushButton(QIcon(":/icons/map.png"), "Map", this);
    mapBtn->setIconSize(s_icon_size);

    layout->addWidget(pointBtn);
    layout->addWidget(pathBtn);
    layout->addWidget(robotBtn);
    layout->addWidget(mapBtn);

    hide();

    layout->setContentsMargins(0, 0, 10, 0);
    layout->setAlignment(Qt::AlignTop);
}

void LeftMenuWidget::showEvent(QShowEvent *event){
    /// to reset the colors of the points on the map
    points->setPixmapAll(PointView::PixmapType::NORMAL);
    emit resetPathPointViews();
    QWidget::showEvent(event);
}

