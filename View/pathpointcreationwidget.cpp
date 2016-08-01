#include "pathpointcreationwidget.h"
#include <QIntValidator>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QDebug>
#include <QLineEdit>
#include <QComboBox>

PathPointCreationWidget::PathPointCreationWidget(const int _id, std::shared_ptr<Points> _points, const Point& _point, QWidget* parent):QWidget(parent){
    layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel("PathPointCreationWidget", this));
    /*point = _point;
    points = _points;
    id = _id;
    name = _point.getName();
    posX = 0;
    posY = 0;

    editLayout = new QHBoxLayout(this);

    /// Label for the name of the point
    pointLabel = new QLabel(this);
    setName(name);
    layout->addWidget(pointLabel);

    /// The widget that contain the layout for the button to select the
    /// action the robot need to do (wait for X sec or wait for human action)
    actionWidget = new QWidget(this);

    QVBoxLayout* actionLayout = new QVBoxLayout(actionWidget);

    actionBtn = new QComboBox(this);
    actionBtn->addItem("Wait for");
    actionBtn->addItem("Human Action");
    actionLayout->addWidget(actionBtn);
    connect(actionBtn, SIGNAL(activated(QString)), this, SLOT(actionClicked(QString)));

    timeWidget = new QWidget(this);
    QHBoxLayout* timeLayout = new QHBoxLayout(timeWidget);
    timeEdit = new QLineEdit(this);
    timeEdit->setAlignment(Qt::AlignCenter);
    timeEdit->setValidator(new QIntValidator(0, 99999, this));
    timeLayout->addWidget(timeEdit);

    QLabel* sLabel = new QLabel("sec", this);
    sLabel->setMaximumWidth(30);
    timeLayout->addWidget(sLabel);

    timeLayout->setContentsMargins(0, 0, 0, 0);
    actionLayout->setContentsMargins(0, 0, 0, 0);
    actionLayout->addWidget(timeWidget);

    layout->addWidget(actionWidget);
    actionWidget->hide();

    cancelBtn = new QPushButton("Cancel", this);
    cancelBtn->hide();
    saveEditBtn = new QPushButton("Save changes", this);
    connect(saveEditBtn, SIGNAL(clicked()), this, SLOT(saveEdit()));

    //editLayout->addWidget(cancelBtn);
    //editLayout->addWidget(saveEditBtn);
    layout->addWidget(saveEditBtn);
    //layout->addLayout(editLayout);
    saveEditBtn->hide();


    layout->setAlignment(Qt::AlignTop);*/
}

void PathPointCreationWidget::setName(const QString _name){
    qDebug() << "PathPointCreationWidget::setName called";
    /*name = _name;
    point.setName(name);
    qDebug() << "New Name :" << name;

    if(name.contains("tmpPoint")){
        posX = point.getPosition().getX();
        posY = point.getPosition().getY();
        setPointLabel(point.getPosition().getX(), point.getPosition().getY());
    } else {
        pointLabel->setText(QString::number(id) + ". " + name);
    }*/
}

void PathPointCreationWidget::setId(const int _id){
    qDebug() << "PathPointCreationWidget::setId called";
    /*id = _id;

    if(name.contains("tmpPoint")){
        setPointLabel(point.getPosition().getX(), point.getPosition().getY());
    } else {
        pointLabel->setText(QString::number(id)+". "+name);
    }*/
}

void PathPointCreationWidget::actionClicked(QString action){
    qDebug() << "PathPointCreationWidget::actionClicked called";
    /*if(action.compare("Wait for") == 0){
        timeWidget->show();
    } else {
        timeWidget->hide();
    }*/
}

void PathPointCreationWidget::displayActionWidget(const bool show){
    qDebug() << "PathPointCreationWidget::displayActionWidget called";
    /*if(show)
        actionWidget->show();
    else
        actionWidget->hide();*/
}

void PathPointCreationWidget::resetAction(){
    qDebug() << "PathPointCreationWidget::resetAction called";
    /*timeEdit->clear();
    actionBtn->setCurrentIndex(0);
    timeWidget->show();*/
}

void PathPointCreationWidget::displaySaveEditBtn(const bool show, const int count){
    qDebug() << "PathPointCreationWidget::displaySaveEditBtn called";
    /*if(show)
        saveEditBtn->show();
    else
        saveEditBtn->hide();

    if(id < count)
        displayActionWidget(!show);*/
}

void PathPointCreationWidget::saveEdit(){
    qDebug() << "PathPointCreationWidget::saveEdit called";

    //emit saveEditSignal(this);
}

void PathPointCreationWidget::setPos(const float _posX, const float _posY){
    qDebug() << "PathPointCreationWidget::setPos called";
    /*posX = _posX;
    posY = _posY;
    point.setPosition(posX, posY);
    setPointLabel(posX, posY);*/
}

void PathPointCreationWidget::updatePointLabel(const float _posX, const float _posY){
    qDebug() << "PathPointCreationWidget::updatePointLabel called";
    //setPointLabel(_posX, _posY);
}

void PathPointCreationWidget::setPointLabel(const float _posX, const float _posY){
    qDebug() << "PathPointCreationWidget::setPointLabel called";
    /*if(name.contains("tmpPoint")){
        pointLabel->setText(QString::number(id)+". "+QString::number(_posX,'f', 1) + "; " + QString::number(_posY,'f', 1));
    }*/
}



