#include "leftmenuwidget.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include <QScrollArea>
#include "Model/points.h"
#include "View/custompushbutton.h"

LeftMenuWidget::LeftMenuWidget(QMainWindow* parent, QSharedPointer<Points> const& _points): QWidget(parent), points(_points)
{

    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    /// Robot button
    CustomPushButton* robotBtn = new CustomPushButton(QIcon(":/icons/robot.png"), "Robots", this);
    //robotBtn->setAutoDefault(true);
    robotBtn->setIconSize(parent->size()/10);

    /// Point button
    CustomPushButton* pointBtn = new CustomPushButton(QIcon(":/icons/coordinates.png"), "Points", this);
    //pointBtn->setAutoDefault(true);
    pointBtn->setIconSize(parent->size()/10);

    /// Map button
    CustomPushButton* mapBtn = new CustomPushButton(QIcon(":/icons/map.png"), "Map", this);
    //mapBtn->setAutoDefault(true);
    mapBtn->setIconSize(parent->size()/10);

    /// Path button
    CustomPushButton* pathBtn = new CustomPushButton(QIcon(":/icons/path.png"), "Paths", this);
    //pathBtn->setAutoDefault(true);
    pathBtn->setIconSize(parent->size()/10);

    layout->addWidget(robotBtn);
    layout->addWidget(pointBtn);
    layout->addWidget(mapBtn);
    layout->addWidget(pathBtn);

    connect(robotBtn, SIGNAL(clicked()), parent, SLOT(robotBtnEvent()));
    connect(pointBtn, SIGNAL(clicked()), parent, SLOT(pointBtnEvent()));
    connect(mapBtn, SIGNAL(clicked()), parent, SLOT(mapBtnEvent()));
    connect(pathBtn, SIGNAL(clicked()), parent, SLOT(pathBtnEvent()));

    connect(this, SIGNAL(resetPathPointViews()), parent, SLOT(resetPathPointViewsSlot()));

    hide();

    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);

    layout->setAlignment(Qt::AlignTop);
}

void LeftMenuWidget::showEvent(QShowEvent *event){
    points->setPixmapAll(PointView::PixmapType::NORMAL);
    emit resetPathPointViews();
    QWidget::showEvent(event);
}

