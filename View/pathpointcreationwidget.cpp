#include "pathpointcreationwidget.h"
#include <QIntValidator>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QDebug>
#include <QLineEdit>
#include <QComboBox>

PathPointCreationWidget::PathPointCreationWidget(const int id, const Points& points, QString name){
    initialisation(id, points, name);
}

PathPointCreationWidget::PathPointCreationWidget(const int id, const Points& points, const Point& _point){
    point = _point;
    initialisation(id, points, _point.getName());
}


void PathPointCreationWidget::initialisation(const int _id, const Points _points, QString _name){
    layout = new QVBoxLayout();
    points = _points;
    id = _id;
    name = _name;
    posX = 0;
    posY = 0;

    /// Label for the name of the point
    pointLabel = new QLabel();
    setName(name);
    layout->addWidget(pointLabel);

    /// The widget that contain the layout for the button to select the
    /// action the robot need to do (wait for X sec or wait for human action)
    actionWidget = new QWidget();
    QVBoxLayout* actionLayout = new QVBoxLayout();

    actionBtn = new QComboBox();
    actionBtn->addItem("Wait for");
    actionBtn->addItem("Human Action");
    actionLayout->addWidget(actionBtn);
    connect(actionBtn, SIGNAL(activated(QString)), this, SLOT(actionClicked(QString)));

    timeWidget = new QWidget();
    QHBoxLayout* timeLayout = new QHBoxLayout();
    timeEdit = new QLineEdit();
    timeEdit->setAlignment(Qt::AlignCenter);
    timeEdit->setValidator(new QIntValidator(0, 99999, this));
    timeLayout->addWidget(timeEdit);

    QLabel* sLabel = new QLabel("sec");
    sLabel->setMaximumWidth(30);
    timeLayout->addWidget(sLabel);

    timeLayout->setContentsMargins(0, 0, 0, 0);
    timeWidget->setLayout(timeLayout);
    actionLayout->setContentsMargins(0, 0, 0, 0);
    actionLayout->addWidget(timeWidget);

    actionWidget->setLayout(actionLayout);
    layout->addWidget(actionWidget);
    actionWidget->hide();

    saveEditBtn = new QPushButton("Save changes");
    connect(saveEditBtn, SIGNAL(clicked()), this, SLOT(saveEdit()));
    layout->addWidget(saveEditBtn);
    saveEditBtn->hide();


    layout->setAlignment(Qt::AlignTop);
    setLayout(layout);
}

PathPointCreationWidget::~PathPointCreationWidget(){
    delete layout;
    delete pointLabel;
    delete actionBtn;
    delete saveEditBtn;
    delete timeEdit;
    delete timeWidget;
    delete actionWidget;
}

void PathPointCreationWidget::setName(const QString _name){
    name = _name;
    point.setName(name);
    qDebug() << "New Name :" << name;

    if(name.compare("tmpPoint") == 0){
        posX = point.getPosition().getX();
        posY = point.getPosition().getY();
        setPointLabel(point.getPosition().getX(), point.getPosition().getY());
    } else {
        pointLabel->setText(QString::number(id)+". "+name);
    }
}

void PathPointCreationWidget::setId(const int _id){
    id = _id;

    if(name.compare("tmpPoint") == 0){
        setPointLabel(point.getPosition().getX(), point.getPosition().getY());
    } else {
        pointLabel->setText(QString::number(id)+". "+name);
    }
}

void PathPointCreationWidget::actionClicked(QString action){
    qDebug() << "actionClicked called ";
    if(action.compare("Wait for") == 0){
        timeWidget->show();
    } else {
        timeWidget->hide();
    }
}

void PathPointCreationWidget::displayActionWidget(const bool show){
    if(show)
        actionWidget->show();
    else
        actionWidget->hide();
}

void PathPointCreationWidget::resetAction(){
    qDebug() << "resetAction called";
    timeEdit->clear();
    actionBtn->setCurrentIndex(0);
    timeWidget->show();
}

void PathPointCreationWidget::displaySaveEditBtn(const bool show, const int count){
    if(show)
        saveEditBtn->show();
    else
        saveEditBtn->hide();

    if(id < count)
        displayActionWidget(!show);
}

void PathPointCreationWidget::saveEdit(){
    qDebug() << "saveEdit called ";

    emit saveEditSignal(this);
}

void PathPointCreationWidget::setPos(const float _posX, const float _posY){
    posX = _posX;
    posY = _posY;
    point.setPosition(posX, posY);
    setPointLabel(posX, posY);
}

void PathPointCreationWidget::updatePointLabel(const float _posX, const float _posY){
    setPointLabel(_posX, _posY);
}

void PathPointCreationWidget::setPointLabel(const float _posX, const float _posY){
    if(name.compare("tmpPoint") == 0){
        pointLabel->setText(QString::number(id)+". "+QString::number(_posX,'f', 1) + "; " + QString::number(_posY,'f', 1));
    }
}

