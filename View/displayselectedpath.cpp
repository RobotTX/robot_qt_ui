#include "displayselectedpath.h"
#include "View/topleftmenu.h"
#include "View/pathwidget.h"
#include "Model/pathpoint.h"
#include "Controller/mainwindow.h"
#include "View/custompushbutton.h"
#include "View/customscrollarea.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QDebug>

DisplaySelectedPath::DisplaySelectedPath(QWidget *parent, MainWindow *mainWindow,  const QSharedPointer<Paths>& _paths):
    QWidget(parent), paths(_paths)
{

    scrollArea = new CustomScrollArea(this);

    layout = new QVBoxLayout(this);

    /// Top menu with the 5 buttons
    actionButtons = new TopLeftMenu(this);
    actionButtons->getPlusButton()->setEnabled(false);
    actionButtons->getGoButton()->setEnabled(false);
    actionButtons->getMapButton()->setCheckable(true);

    actionButtons->getMinusButton()->setToolTip("Click here to remove the path");
    actionButtons->getEditButton()->setToolTip("Click here to edit the path");
    actionButtons->getMapButton()->setToolTip("Click here to display the path");

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
    scrollArea->setWidget(pathWidget);
    layout->addWidget(scrollArea);

    connect(this, SIGNAL(deletePath(QString, QString)), mainWindow, SLOT(deletePathSlot(QString, QString)));
    connect(this, SIGNAL(editPath(QString, QString)), mainWindow, SLOT(editPathSlot(QString, QString)));
    connect(this, SIGNAL(displayPath(QString, QString, bool)), mainWindow, SLOT(displayPathSlot(QString, QString, bool)));

    layout->setAlignment(Qt::AlignTop);
    //layout->setContentsMargins(0,0,0,0);
}

void DisplaySelectedPath::updatePath(QString groupName, QString pathName, QVector<QSharedPointer<PathPoint>> path){
    qDebug() << "DisplaySelectedPath::updatePath called";
    currentPath.groupName = groupName;
    currentPath.pathName = pathName;
    currentPath.path = path;
    nameLabel->setText(pathName);
    pathWidget->setPath(path);
    if(!paths->getVisiblePath().compare(currentPath.pathName))
        actionButtons->getMapButton()->setChecked(true);
    else
        actionButtons->getMapButton()->setChecked(false);
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
