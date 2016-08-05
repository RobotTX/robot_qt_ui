#include "pathpointcreationwidget.h"
#include <QIntValidator>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QDebug>
#include <QLineEdit>
#include <QComboBox>

PathPointCreationWidget::PathPointCreationWidget(const int _id, const QString _name, const double x, const double y, QWidget* parent)
    :QWidget(parent), id(_id), name(_name), posX(x), posY(y){
    layout = new QHBoxLayout(this);

    QVBoxLayout* rightLayout = new QVBoxLayout();

    editLayout = new QHBoxLayout(this);

    /// Label for the name of the point
    pointLabel = new QLabel(this);
    setName(name);
    rightLayout->addWidget(pointLabel);

    /// The widget that contain the layout for the button to select the
    /// action the robot need to do (wait for X sec or wait for human action)
    actionWidget = new QWidget(this);

    QVBoxLayout* actionLayout = new QVBoxLayout(actionWidget);

    actionBtn = new QComboBox(this);
    actionBtn->addItem("Wait for");
    actionBtn->addItem("Human Action");
    actionLayout->addWidget(actionBtn);

    timeWidget = new QWidget(this);
    QHBoxLayout* timeLayout = new QHBoxLayout(timeWidget);
    timeEdit = new QLineEdit("0" ,this);
    timeEdit->setAlignment(Qt::AlignCenter);
    timeEdit->setValidator(new QIntValidator(0, 99999, this));
    timeLayout->addWidget(timeEdit);

    QLabel* sLabel = new QLabel("sec", this);
    sLabel->setMaximumWidth(30);
    timeLayout->addWidget(sLabel);

    timeLayout->setContentsMargins(0, 0, 0, 0);
    actionLayout->setContentsMargins(0, 0, 0, 0);
    actionLayout->addWidget(timeWidget);

    rightLayout->addWidget(actionWidget);
    actionWidget->hide();

    cancelBtn = new QPushButton("Cancel", this);
    cancelBtn->hide();
    saveEditBtn = new QPushButton("Save changes", this);
    saveEditBtn->hide();


    connect(actionBtn, SIGNAL(activated(QString)), this, SLOT(actionClicked(QString)));
    connect(saveEditBtn, SIGNAL(clicked()), this, SLOT(saveEdit()));
    connect(timeEdit, SIGNAL(textChanged(QString)), this, SLOT(timeChanged(QString)));



    rightLayout->addWidget(saveEditBtn);
    rightLayout->setAlignment(Qt::AlignTop);

    QLabel*  moveImage = new QLabel();
    QPixmap mypix (":/icons/cropped_list.png");
    moveImage->setPixmap(mypix);
    moveImage->setScaledContents(true);
    moveImage->setMaximumWidth(10);
    moveImage->setMaximumHeight(10);
    layout->addWidget(moveImage);

    layout->addLayout(rightLayout);
    layout->setContentsMargins(2,0,0,11);
    rightLayout->setContentsMargins(0,11,0,11);
}

void PathPointCreationWidget::setName(const QString _name){
    qDebug() << "PathPointCreationWidget::setName called" << _name;
    name = _name;

    if(name.compare(TMP_POINT_NAME) == 0){
        setPointLabel(posX, posY);
    } else {
        pointLabel->setText(QString::number(id) + ". " + name);
    }
}

void PathPointCreationWidget::setId(const int _id){
    qDebug() << "PathPointCreationWidget::setId called" << _id;
    id = _id;

    if(name.compare(TMP_POINT_NAME) == 0){
        setPointLabel(posX, posY);
    } else {
        pointLabel->setText(QString::number(id)+". "+name);
    }
}

void PathPointCreationWidget::actionClicked(QString action){
    qDebug() << "PathPointCreationWidget::actionClicked called" << action;
    if(action.compare("Wait for") == 0){
        timeWidget->show();
    } else {
        timeWidget->hide();
        timeEdit->setText("");
    }
    emit actionChanged(id, timeEdit->text());
}

void PathPointCreationWidget::timeChanged(QString){
    qDebug() << "PathPointCreationWidget::timeChanged called";
    emit actionChanged(id, timeEdit->text());
}

void PathPointCreationWidget::displayActionWidget(const bool show){
    qDebug() << "PathPointCreationWidget::displayActionWidget called";
    if(show)
        actionWidget->show();
    else
        actionWidget->hide();
}

void PathPointCreationWidget::resetAction(){
    qDebug() << "PathPointCreationWidget::resetAction called";
    timeEdit->clear();
    actionBtn->setCurrentIndex(0);
    timeWidget->show();
}

void PathPointCreationWidget::displaySaveEditBtn(const bool show, const int count){
    qDebug() << "PathPointCreationWidget::displaySaveEditBtn called";
    if(show)
        saveEditBtn->show();
    else
        saveEditBtn->hide();

    if(id < count)
        displayActionWidget(!show);
}

void PathPointCreationWidget::saveEdit(){
    qDebug() << "PathPointCreationWidget::saveEdit called";

    emit saveEditSignal(this);
}

void PathPointCreationWidget::setPos(const float x, const float y){
    qDebug() << "PathPointCreationWidget::setPos called";
    posX = x;
    posY = y;
    setPointLabel(posX, posY);
}

void PathPointCreationWidget::updatePointLabel(const float x, const float y){
    qDebug() << "PathPointCreationWidget::updatePointLabel called";
    setPointLabel(x, y);
}

void PathPointCreationWidget::setPointLabel(const float _posX, const float _posY){
    qDebug() << "PathPointCreationWidget::setPointLabel called";
    if(name.contains(TMP_POINT_NAME)){
        pointLabel->setText(QString::number(id)+". "+QString::number(_posX,'f', 1) + "; " + QString::number(_posY,'f', 1));
    }
}



