#include "pathpointcreationwidget.h"
#include <QIntValidator>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QDebug>
#include <QLineEdit>
#include <QComboBox>
#include "Model/pathpoint.h"
#include "View/custompushbutton.h"
#include "View/customlineedit.h"
#include "View/stylesettings.h"

PathPointCreationWidget::PathPointCreationWidget(const int _id, const QString _name, const double x, const double y, QWidget* parent):
    QWidget(parent), id(_id), name(_name), posX(x), posY(y)
{
    layout = new QHBoxLayout(this);

    pathWidget = new QWidget(this);
    editWidget = new QWidget(this);
    QVBoxLayout* pathLayout = new QVBoxLayout(pathWidget);
    QVBoxLayout* editLayout = new QVBoxLayout(editWidget);

    topLayout = new QGridLayout();
    /// Label for the name of the point
    pointLabel = new QLabel(this);
    setName(name);
    topLayout->addWidget(pointLabel, 0, 0);

    closeBtn = new CustomPushButton(QIcon(":/icons/close.png"), "", this);
    closeBtn->setIconSize(xxs_icon_size);
    topLayout->addWidget(closeBtn, 0, 1);

    topLayout->setColumnStretch(0, 1);
    pathLayout->addLayout(topLayout);

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
    timeEdit = new CustomLineEdit("0" ,this);
    timeEdit->setAlignment(Qt::AlignCenter);
    timeEdit->setValidator(new QIntValidator(0, 99999, this));
    timeLayout->addWidget(timeEdit);

    QLabel* sLabel = new QLabel("sec", this);
    timeLayout->addWidget(sLabel);

    timeLayout->setContentsMargins(0, 0, 0, 0);
    actionLayout->setContentsMargins(0, 0, 0, 0);
    actionLayout->addWidget(timeWidget);

    pathLayout->addWidget(actionWidget);
    actionWidget->hide();

    cancelBtn = new CustomPushButton("Cancel", this);
    saveEditBtn = new CustomPushButton("Save changes", this);
    editLayout->addWidget(saveEditBtn);
    editLayout->addWidget(cancelBtn);
    editWidget->hide();

    connect(actionBtn, SIGNAL(activated(QString)), this, SLOT(actionClicked(QString)));
    connect(saveEditBtn, SIGNAL(clicked()), this, SLOT(saveEdit()));
    connect(cancelBtn, SIGNAL(clicked()), this, SLOT(cancelEdit()));
    connect(timeEdit, SIGNAL(textChanged(QString)), this, SLOT(timeChanged(QString)));
    connect(closeBtn, SIGNAL(clicked()), this, SLOT(removePathPoint()));

    pathLayout->setAlignment(Qt::AlignTop);

    QLabel*  moveImage = new QLabel();
    QPixmap mypix (":/icons/list.png");
    moveImage->setPixmap(mypix);
    moveImage->setScaledContents(true);
    moveImage->setMaximumWidth(10);
    moveImage->setMaximumHeight(10);
    layout->addWidget(moveImage);

    layout->addWidget(pathWidget);
    layout->addWidget(editWidget);
    layout->setContentsMargins(0, 0, 0, 0);
}

void PathPointCreationWidget::setName(const QString _name){
    //qDebug() << "PathPointCreationWidget::setName called" << _name;
    name = _name;
    (name.contains(PATH_POINT_NAME)) ? setPointLabel(posX, posY) : pointLabel->setText(QString::number(id+1) + ". " + name);
}

void PathPointCreationWidget::setId(const int _id){
    //qDebug() << "PathPointCreationWidget::setId called" << _id;
    id = _id;
    (name.contains(PATH_POINT_NAME)) ? setPointLabel(posX, posY) : pointLabel->setText(QString::number(id+1) + ". " + name);
}

void PathPointCreationWidget::actionClicked(QString action){
    //qDebug() << "PathPointCreationWidget::actionClicked called" << action;
    if(action.compare("Wait for") == 0){
        timeWidget->show();
    } else {
        timeWidget->hide();
        timeEdit->setText("-1");
    }
    emit actionChanged(id, timeEdit->text());
}

void PathPointCreationWidget::timeChanged(QString){
    //qDebug() << "PathPointCreationWidget::timeChanged called";
    emit actionChanged(id, timeEdit->text());
}

void PathPointCreationWidget::displayActionWidget(const bool show){
    //qDebug() << "PathPointCreationWidget::displayActionWidget called";
    (show) ? actionWidget->show() : actionWidget->hide();
}

void PathPointCreationWidget::resetAction(){
    //qDebug() << "PathPointCreationWidget::resetAction called";
    timeEdit->setText("0");
    actionBtn->setCurrentIndex(0);
    timeWidget->show();
}

void PathPointCreationWidget::displaySaveEditBtn(const bool show, const int count){
    qDebug() << "PathPointCreationWidget::displaySaveEditBtn called";
    (show) ? saveEditBtn->show() : saveEditBtn->hide();
    if(id < count)
        displayActionWidget(!show);
}

void PathPointCreationWidget::saveEdit(){
    //qDebug() << "PathPointCreationWidget::saveEdit called";
    emit saveEditSignal(this);
}

void PathPointCreationWidget::cancelEdit(){
    qDebug() << "PathPointCreationWidget::cancelEdit called";
    emit cancelEditSignal(this);
}

void PathPointCreationWidget::setPos(const float x, const float y){
    //qDebug() << "PathPointCreationWidget::setPos called";
    posX = x;
    posY = y;
    setPointLabel(posX, posY);
}

void PathPointCreationWidget::updatePointLabel(const float x, const float y){
    //qDebug() << "PathPointCreationWidget::updatePointLabel called";
    setPointLabel(x, y);
}

void PathPointCreationWidget::setPointLabel(const float _posX, const float _posY){
    //qDebug() << "PathPointCreationWidget::setPointLabel called";
    if(name.contains(PATH_POINT_NAME))
        pointLabel->setText(QString::number(id+1) + ". " + QString::number(_posX,'f', 1) + "; " + QString::number(_posY, 'f', 1));
}

void PathPointCreationWidget::removePathPoint(){
    qDebug() << "PathPointCreationWidget::removePathPoint called";
    emit removePathPoint(this);
}

void PathPointCreationWidget::setActionWidget(const int waitTime){
    if(waitTime >= 0){
        timeWidget->show();
        timeEdit->setText(QString::number(waitTime));
    } else {
        timeWidget->hide();
        timeEdit->setText("0");
        actionBtn->setCurrentIndex(1);
    }
    emit actionChanged(id, timeEdit->text());
}
