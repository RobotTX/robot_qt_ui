#include "displayselectedpath.h"
#include "View/topleftmenu.h"
#include "View/pathwidget.h"
#include "Model/pathpoint.h"
#include "Controller/mainwindow.h"
#include "View/custompushbutton.h"
#include "View/customscrollarea.h"
#include <QKeyEvent>
#include "View/customlabel.h"
#include <QVBoxLayout>
#include <QDebug>

DisplaySelectedPath::DisplaySelectedPath(MainWindow *mainWindow,  const QSharedPointer<Paths>& _paths):
    QWidget(mainWindow), paths(_paths)
{
    scrollArea = new CustomScrollArea(this, true);

    layout = new QVBoxLayout(this);
    QVBoxLayout* topLayout = new QVBoxLayout();

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

    topLayout->addWidget(actionButtons);

    /// Label with the name of the path
    nameLabel = new CustomLabel("", this, true);
    topLayout->addWidget(nameLabel);
    layout->addLayout(topLayout);

    /// Widget displaying the path
    pathWidget = new PathWidget(this);

    /// increases space between name of path and description of the path
    pathWidget->setContentsMargins(0, 10, 0, 0);
    scrollArea->setWidget(pathWidget);
    layout->addWidget(scrollArea);

    connect(this, SIGNAL(deletePath(QString, QString)), mainWindow, SLOT(deletePathSlot(QString, QString)));
    connect(this, SIGNAL(editPath(QString, QString)), mainWindow, SLOT(editPathSlot(QString, QString)));
    layout->setAlignment(Qt::AlignTop);
    topLayout->setContentsMargins(0, 0, 10, 0);
    layout->setContentsMargins(0, 0, 0, 0);
}

void DisplaySelectedPath::updatePath(const QString groupName, const QString pathName, const QVector<QSharedPointer<PathPoint>>& path, const QString visiblePath){
    qDebug() << "DisplaySelectedPath::updatePath called";
    currentPath.groupName = groupName;
    currentPath.pathName = pathName;
    currentPath.path = path;
    nameLabel->setText(pathName);
    pathWidget->setPath(path);
    actionButtons->getMapButton()->setChecked((!visiblePath.compare(currentPath.pathName)) ? true : false);
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

void DisplaySelectedPath::keyPressEvent(QKeyEvent *event){
    if(event->key() == Qt::Key_Delete)
        emit deletePath(currentPath.groupName, currentPath.pathName);
}
