#include "displayselectedpoint.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QMainWindow>
#include <QLineEdit>
#include <QDebug>
#include <QKeyEvent>
#include "Model/xmlparser.h"

DisplaySelectedPoint::DisplaySelectedPoint(QMainWindow *_parent, Points const& _points, std::shared_ptr<Point> const& _point, const Origin origin)
{
    parent = _parent;
    points = _points;
    point = _point;

    layout = new QVBoxLayout();

    nameLayout = new QHBoxLayout();

    backButton = new QPushButton(QIcon(":/icons/arrowLeft.png"), "Groups");
    backButton->setIconSize(_parent->size()/10);
    layout->addWidget(backButton);

    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"");
    plusButton->setIconSize(_parent->size()/10);
    plusButton->setEnabled(false);

    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"");
    minusButton->setIconSize(_parent->size()/10);
    minusButton->setCheckable(true);

    editButton = new QPushButton(QIcon(":/icons/edit.png"),"");
    editButton->setIconSize(_parent->size()/10);
    editButton->setCheckable(true);

    grid = new QHBoxLayout();
    grid->addWidget(plusButton);
    grid->addWidget(minusButton);
    grid->addWidget(editButton);

    eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "");
    eyeButton->setIconSize(_parent->size()/10);
    eyeButton->setEnabled(false);

    mapButton = new QPushButton(QIcon(":/icons/map.png"),"");
    mapButton->setCheckable(true);
    mapButton->setIconSize(_parent->size()/10);

    eyeMapLayout = new QHBoxLayout();
    eyeMapLayout->addWidget(eyeButton);
    eyeMapLayout->addWidget(mapButton);

    layout->addLayout(grid);
    layout->addLayout(eyeMapLayout);


    //nameLabell = new QLabel("Name : ");
    //nameLabell->setAlignment(Qt::AlignCenter);
    ////nameLabell->setWordWrap(true);
    //nameLayout->addWidget(nameLabell);

    nameEdit = new QLineEdit();
    nameEdit->setReadOnly(true);
    nameEdit->setStyleSheet("* { background-color: rgba(255, 0, 0, 0); }");
    connect(this->getNameEdit(), SIGNAL(textChanged(QString)), this, SLOT(updateNameUsingKey(QString)));

    nameLayout->addWidget(nameEdit);

    layout->addLayout(nameLayout);

    //nameLabell->setWordWrap(true);
    //layout->addWidget(nameLabel);

    posXLabel = new QLabel("X : ");
    posXLabel->setWordWrap(true);
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ");
    posYLabel->setWordWrap(true);
    layout->addWidget(posYLabel);



    saveButton = new QPushButton("Save");
    layout->addWidget(saveButton);
    connect(this->getSaveButton(), SIGNAL(clicked(bool)), this, SLOT(updateNameUsingButton()));
    setLayout(layout);
}

DisplaySelectedPoint::~DisplaySelectedPoint(){
    //delete nameLabell;
    delete posXLabel;
    delete posYLabel;
    delete layout;
    delete backButton;
    delete plusButton;
    delete minusButton;
    delete editButton;
    delete mapButton;
    delete eyeButton;
    delete grid;
    delete eyeMapLayout;
    delete nameEdit;
    delete nameLayout;
    delete parent;
    delete saveButton;
}

void DisplaySelectedPoint::displayPointInfo(void){
    ////nameLabell->setText("Name : ");
    posXLabel->setText("X : " + QString::number(point->getPosition().getX()));
    posYLabel->setText("Y : " + QString::number(point->getPosition().getY()));
    nameEdit->setText(point->getName());
}


void DisplaySelectedPoint::updateNameUsingKey(QString newName){
    emit nameChanged(point->getName(), newName);
    point->setName(newName);
    XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
    parserPoints.save(points);
}

void DisplaySelectedPoint::updateNameUsingButton(){
    emit nameChanged(point->getName(), nameEdit->text());
    point->setName(nameEdit->text());
    XMLParser parserPoints("/home/joan/Qt/QtProjects/gobot-software/gobot-software/points.xml");
    parserPoints.save(points);
    /// so that the name cannot be changed anymore unless you click on the edit button again
    nameEdit->setReadOnly(true);
    /// so that you cannot edit a new name unless you click the edit button again
    editButton->setChecked(false);
}

void DisplaySelectedPoint::mousePressEvent(QEvent* event){
    qDebug() << "mouse pressed";
    nameEdit->setReadOnly(true);
}

void DisplaySelectedPoint::keyPressEvent(QKeyEvent* event){
    if(!event->text().compare("\r")){
        qDebug() << "enter pressed";
        editButton->setChecked(false);
        nameEdit->setReadOnly(true);
    }
}

void DisplaySelectedPoint::displayPointInfo(const std::shared_ptr<Point> _point){
    //nameLabell->setText("Name : " + _point->getName());
    posXLabel->setText("X : " + QString::number(_point->getPosition().getX()));
    posYLabel->setText("Y : " + QString::number(_point->getPosition().getY()));
}

void DisplaySelectedPoint::setOrigin(const Origin _origin){
    origin = _origin;
    qDebug() << origin;
    /// if we come from the map there is simply no where
    /// to return so we hide the button
    /// the distinction between when we come from the group menu
    /// and when we come from the points menu is made in the pointBtnEvent
    if(origin == MAP)
        backButton->hide();
    else
        backButton->show();
}

