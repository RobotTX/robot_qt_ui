#include "pathpointcreationwidget.h"
#include "Model/group.h"
#include <QIntValidator>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QDebug>
#include <QMenu>
#include <QLineEdit>

PathPointCreationWidget::PathPointCreationWidget(const int id, const Points points, QString name){
    initialisation(id, points, name);
}

PathPointCreationWidget::PathPointCreationWidget(int id, Points points, Point _point){
    point = _point;
    initialisation(id, points, _point.getName());
}


void PathPointCreationWidget::initialisation(const int _id, const Points _points, QString _name){
    layout = new QVBoxLayout();
    points = _points;
    id = _id;
    name = _name;
    waitHuman = false;
    posX = 0;
    posY = 0;

    /// Label for the name of the point
    pointLabel = new QLabel();
    setName(name);
    layout->addWidget(pointLabel);

    /// The menu which display the list of point to select
    pointsMenu = new QMenu();
    connect(pointsMenu, SIGNAL(triggered(QAction*)), this, SLOT(pointClicked(QAction*)));

    for(int i = 0; i < points.getGroups().size(); i++){
        if(points.getGroups().at(i)->getName().compare("No group") == 0){
            for(int j = 0; j < points.getGroups().at(i)->getPoints().size(); j++){
                QString pointName = points.getGroups().at(i)->getPoints().at(j)->getName();
                QAction *point = pointsMenu->addAction(pointName);

                PointInfo pointInfo;
                pointInfo.name = pointName;
                pointInfo.posX = points.getGroups().at(i)->getPoints().at(j)->getPosition().getX();
                pointInfo.posY = points.getGroups().at(i)->getPoints().at(j)->getPosition().getY();
                pointInfos.push_back(pointInfo);
            }
        } else {
            QMenu *group = pointsMenu->addMenu("&" + points.getGroups().at(i)->getName());
            for(int j = 0; j < points.getGroups().at(i)->getPoints().size(); j++){
                QString pointName = points.getGroups().at(i)->getPoints().at(j)->getName();
                QAction *point = group->addAction(pointName);

                PointInfo pointInfo;
                pointInfo.name = pointName;
                pointInfo.posX = points.getGroups().at(i)->getPoints().at(j)->getPosition().getX();
                pointInfo.posY = points.getGroups().at(i)->getPoints().at(j)->getPosition().getY();
                pointInfos.push_back(pointInfo);
            }
        }
    }

    /// The widget that contain the layout for the button to select the
    /// action the robot need to do (wait for X sec or wait for human action)
    actionWidget = new QWidget();
    QVBoxLayout* actionLayout = new QVBoxLayout();

    actionBtn = new QPushButton("Wait for");
    actionLayout->addWidget(actionBtn);
    connect(actionBtn, SIGNAL(clicked()), this, SLOT(actionClicked()));

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


    //setAutoFillBackground( false );
    layout->setAlignment(Qt::AlignTop);
    setLayout(layout);
}

PathPointCreationWidget::~PathPointCreationWidget(){
    delete layout;
    delete pointLabel;
    delete pointsMenu;
    delete actionBtn;
    delete timeEdit;
    delete timeWidget;
    delete actionWidget;
}

void PathPointCreationWidget::setName(const QString _name){
    name = _name;
    if(name.compare("tmpPoint") == 0){
        posX = point.getPosition().getX();
        posY = point.getPosition().getY();
        pointLabel->setText(QString::number(id)+". "+QString::number(point.getPosition().getX(),'f', 1) + ", " + QString::number(point.getPosition().getY(),'f', 1));
    } else {
        pointLabel->setText(QString::number(id)+". "+name);
    }
}

void PathPointCreationWidget::setId(const int _id){
    id = _id;
    pointLabel->setText(QString::number(id)+". "+name);
}

void PathPointCreationWidget::clicked(void){
    qDebug() << "I have been clicked" << name;
    if(pointsMenu != NULL){
        pointsMenu->exec(QCursor::pos());
    }
}

void PathPointCreationWidget::pointClicked(QAction *action){
    qDebug() << "pointClicked called " << action->text();
    setName(action->text());
    for(int i = 0; i < pointInfos.size(); i++){
        if(pointInfos.at(i).name.compare(action->text()) == 0){
            posX = pointInfos.at(i).posX;
            posY = pointInfos.at(i).posY;
        }
    }
    emit pointSelected(id, name);
}

void PathPointCreationWidget::actionClicked(){
    qDebug() << "actionClicked called ";
    if(waitHuman){
        actionBtn->setText("Wait for");
        waitHuman = false;
        timeWidget->show();
    } else {
        actionBtn->setText("Human Action");
        waitHuman = true;
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
    actionBtn->setText("Wait for");
    waitHuman = false;
    timeWidget->show();
}


