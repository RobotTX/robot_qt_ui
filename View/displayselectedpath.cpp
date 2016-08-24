#include "displayselectedpath.h"
#include "View/topleftmenu.h"
#include "View/pathwidget.h"
#include "Model/pathpoint.h"
#include "Controller/mainwindow.h"
#include "View/custompushbutton.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QDebug>

DisplaySelectedPath::DisplaySelectedPath(MainWindow *parent):QWidget(parent){
    layout = new QVBoxLayout(this);

    /// Top menu with the 5 buttons
    actionButtons = new TopLeftMenu(this);
    actionButtons->getPlusButton()->setEnabled(false);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMapButton()->setCheckable(true);

    actionButtons->getMinusButton()->setToolTip("You can click this button to remove the path");
    actionButtons->getEditButton()->setToolTip("You can click this button to edit the path");
    actionButtons->getMapButton()->setToolTip("You can click this button to display the path");

    connect(actionButtons->getMinusButton(), SIGNAL(clicked(bool)), this, SLOT(minusBtnSlot(bool)));
    connect(actionButtons->getEditButton(), SIGNAL(clicked(bool)), this, SLOT(editBtnSlot(bool)));
    connect(actionButtons->getMapButton(), SIGNAL(clicked(bool)), this, SLOT(mapBtnSlot(bool)));

    layout->addWidget(actionButtons);

    /// Label with the name of the path
    nameLabel = new QLabel("", this);
    nameLabel->setAlignment(Qt::AlignCenter);
    nameLabel->setStyleSheet("font-weight: bold; text-decoration:underline");
    layout->addWidget(nameLabel);

    /// Widget displaying the path
    pathWidget = new PathWidget(this);
    layout->addWidget(pathWidget);


    connect(this, SIGNAL(deletePath(QString, QString)), parent, SLOT(deletePathSlot(QString, QString)));
    connect(this, SIGNAL(editPath(QString, QString)), parent, SLOT(editPathSlot(QString, QString)));
    connect(this, SIGNAL(displayPath(QString, QString, bool)), parent, SLOT(displayPathSlot(QString, QString, bool)));

    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0,0,0,0);
}

void DisplaySelectedPath::updatePath(QString groupName, QString pathName, QVector<QSharedPointer<PathPoint>> path){
    qDebug() << "DisplaySelectedPath::updatePath called";
    currentPath.groupName = groupName;
    currentPath.pathName = pathName;
    currentPath.path = path;
    nameLabel->setText(pathName);
    pathWidget->setPath(path);
}

void DisplaySelectedPath::minusBtnSlot(bool){
    qDebug() << "DisplaySelectedPath::minusBtnSlot called";
    emit deletePath(currentPath.groupName, currentPath.pathName);
}

void DisplaySelectedPath::editBtnSlot(bool){
    qDebug() << "DisplaySelectedPath::editBtnSlot called";
    emit editPath(currentPath.groupName, currentPath.pathName);
}

void DisplaySelectedPath::mapBtnSlot(bool checked){
    qDebug() << "DisplaySelectedPath::mapBtnSlot called + checked :" << checked;
    emit displayPath(currentPath.groupName, currentPath.pathName, checked);
}
