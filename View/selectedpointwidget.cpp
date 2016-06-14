#include "selectedpointwidget.h"
#include "View/pointview.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>

SelectedPointWidget::SelectedPointWidget(QMainWindow* parent){

    /// Button with the name of the robot and which allow the user to return back
    /// to the last menu
    layout = new QVBoxLayout();
    backBtn = new QPushButton(QIcon(":/icons/arrowLeft.png"),"");
    backBtn->setStyleSheet ("text-align: left");
    backBtn->setIconSize(parent->size()/10);
    layout->addWidget(backBtn);

    /// Layout for suppression and edition buttons
    QHBoxLayout* grid = new QHBoxLayout();
    QPushButton* minusBtn = new QPushButton(QIcon(":/icons/minus.png"),"");
    QPushButton* editBtn = new QPushButton(QIcon(":/icons/edit.png"),"");
    editBtn->setIconSize(parent->size()/10);
    minusBtn->setIconSize(parent->size()/10);

    grid->addWidget(minusBtn);
    grid->addWidget(editBtn);

    layout->addLayout(grid);

    /// Labels with the position of the point
    posXLabel = new QLabel("X : ");
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ");
    layout->addWidget(posYLabel);

    connect(backBtn, SIGNAL(clicked()), parent, SLOT(backSelecPointBtnEvent()));
    connect(minusBtn, SIGNAL(clicked()), parent, SLOT(minusSelecPointBtnEvent()));
    connect(editBtn, SIGNAL(clicked()), parent, SLOT(editSelecPointBtnEvent()));

    hide();
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    setLayout(layout);
}

SelectedPointWidget::~SelectedPointWidget(){
    delete layout;
    delete backBtn;
    delete posXLabel;
    delete posYLabel;
}

void SelectedPointWidget::setSelectedPoint(PointView * const &pointView){
    /// Update the name and the position labels of the point
    backBtn->setText(pointView->getPoint()->getName());

    posXLabel->setText("X : " + QString::number(pointView->getPoint()->getPosition().getX()));
    posYLabel->setText("Y : " + QString::number(pointView->getPoint()->getPosition().getY()));

    update();
}
