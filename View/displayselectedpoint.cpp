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

    nameEdit = new QLineEdit();
    nameEdit->setReadOnly(true);
    nameEdit->setStyleSheet("* { background-color: rgba(255, 0, 0, 0); }");
    connect(this->getNameEdit(), SIGNAL(textChanged(QString)), this, SLOT(updatePointUsingKey(QString)));

    nameLayout->addWidget(nameEdit);

    layout->addLayout(nameLayout);

    posXLabel = new QLabel("X : ");
    posXLabel->setWordWrap(true);
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ");
    posYLabel->setWordWrap(true);
    layout->addWidget(posYLabel);

    saveButton = new QPushButton("Save");
    layout->addWidget(saveButton);
    connect(this->getSaveButton(), SIGNAL(clicked(bool)), _parent, SLOT(updatePointUsingButton()));
    setLayout(layout);
}

DisplaySelectedPoint::~DisplaySelectedPoint(){
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
    posXLabel->setText("X : " + QString::number(point->getPosition().getX()));
    posYLabel->setText("Y : " + QString::number(point->getPosition().getY()));
    nameEdit->setText(point->getName());
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

