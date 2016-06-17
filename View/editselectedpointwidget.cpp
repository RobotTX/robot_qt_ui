#include "editselectedpointwidget.h"
#include "View/pointview.h"
#include "View/pointsview.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>
#include <QLineEdit>
#include <QDebug>

EditSelectedPointWidget::EditSelectedPointWidget(QMainWindow* _parent, PointsView* _points){
    parent = _parent;
    points = _points;

    groupMenu = new GroupMenu(points->getPoints(), true);
    groupMenu->displayReverse();

    layout = new QVBoxLayout();
    nameEdit = new QLineEdit(_parent);
    nameEdit->setStyleSheet ("text-align: left");
    layout->addWidget(nameEdit);

    posXLabel = new QLabel("X : ");
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ");
    layout->addWidget(posYLabel);

    QHBoxLayout* grid = new QHBoxLayout();

    saveBtn = new QPushButton("Save");

    //grid->addWidget(cancelBtn);
    grid->addWidget(saveBtn);

    layout->addWidget(groupMenu);
    layout->addLayout(grid);

    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveEditSelecPointBtnEvent()));
    connect(nameEdit, SIGNAL(textEdited(QString)), this, SLOT(checkPointName()));

    hide();
    setMaximumWidth(_parent->width()*4/10);
    setMinimumWidth(_parent->width()*4/10);
    layout->setAlignment(Qt::AlignTop);
    setLayout(layout);
}

EditSelectedPointWidget::~EditSelectedPointWidget(){
    delete layout;
    delete pointView;
    delete nameEdit;
    delete posXLabel;
    delete posYLabel;
    delete points;
    delete saveBtn;
    delete parent;
    delete groupMenu;
}

void EditSelectedPointWidget::setSelectedPoint(PointView * const &_pointView, const bool isTemporary){
    _isTemporary = isTemporary;
    pointView = _pointView;
    nameEdit->setText(pointView->getPoint()->getName());
    posXLabel->setText("X : " + QString::number(pointView->getPoint()->getPosition().getX()));
    posYLabel->setText("Y : " + QString::number(pointView->getPoint()->getPosition().getY()));
}

void EditSelectedPointWidget::saveEditSelecPointBtnEvent(){
    qDebug() << "saveEditSelecPointBtnEvent called";
    if(isTemporary()){
        pointView->getPoint()->setName(nameEdit->text());

    }
    emit pointSaved();
}

void EditSelectedPointWidget::checkPointName(void){
    qDebug() << "checkPointName called";
    /*if((group->existPointName(nameEdit->text()) || nameEdit->text() == "") && nameEdit->text() != pointView->getPoint()->name()){
        saveBtn->setEnabled(false);
        qDebug() << "Save btn not enabled : " << nameEdit->text() << "already exist";
    } else {
        saveBtn->setEnabled(true);
        qDebug() << "Save btn enabled";
    }*/
}

