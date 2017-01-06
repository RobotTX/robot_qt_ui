#include "scanmapwidget.h"
#include <QHBoxLayout>
#include <QDebug>
#include "View/customqgraphicsview.h"
#include "View/custompushbutton.h"
#include "View/spacewidget.h"
#include <QButtonGroup>
#include "stylesettings.h"
#include "View/mergemaplistwidget.h"
#include "View/scanmaplistitemwidget.h"
#include "View/scanmapgraphicsitem.h"
#include <QMenu>
#include "View/robotview.h"


ScanMapWidget::ScanMapWidget(QSharedPointer<Robots> _robots, QWidget* parent) : QWidget(parent), robots(_robots){
    setAttribute(Qt::WA_DeleteOnClose);
    setMouseTracking(true);
    layout = new QHBoxLayout(this);

    initializeMap();
    initializeMenu();

    layout->addWidget(graphicsView);

    resize(800, 600);
    show();

    /// We center the window on the desktop
    QRect screenGeometry = QApplication::desktop()->screenGeometry();
    int x = (screenGeometry.width() - width()) / 2;
    int y = (screenGeometry.height() - height()) / 2;
    move(x, y);
}

void ScanMapWidget::initializeMenu(){
    QWidget* menuWidget = new QWidget(this);
    QVBoxLayout* menuLayout = new QVBoxLayout(menuWidget);
    QVBoxLayout* topMenuLayout = new QVBoxLayout();

    QLabel* titleLabel = new QLabel("Scan a new map", this);
    QFont tmpFont = font();
    tmpFont.setPointSize(13);
    setFont(tmpFont);
    titleLabel->setFont(tmpFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    topMenuLayout->addWidget(titleLabel);

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    topMenuLayout->addWidget(spaceWidget);

    CustomPushButton* addImageRobotBtn = new CustomPushButton("Start a scan", this);
    addImageRobotBtn->setToolTip("Start to scan with a robot");
    connect(addImageRobotBtn, SIGNAL(clicked()), this, SLOT(addImageRobotSlot()));
    topMenuLayout->addWidget(addImageRobotBtn);


    listWidget = new MergeMapListWidget(this);
    //connect(listWidget, SIGNAL(dirKeyPressed(int)), this, SLOT(dirKeyEventSlot(int)));
    topMenuLayout->addWidget(listWidget);

    menuLayout->addLayout(topMenuLayout);


    QGridLayout* manualLayout = new QGridLayout();

    QPushButton* upBtn = new QPushButton(QIcon(":/icons/up.png"),"", this);
    upBtn->setFlat(true);
    QPushButton* rightBtn = new QPushButton(QIcon(":/icons/right.png"),"", this);
    rightBtn->setFlat(true);
    QPushButton* downBtn = new QPushButton(QIcon(":/icons/down.png"),"", this);
    downBtn->setFlat(true);
    QPushButton* leftBtn = new QPushButton(QIcon(":/icons/left.png"),"", this);
    leftBtn->setFlat(true);

    manualLayout->addWidget(upBtn, 0, 1);
    manualLayout->addWidget(leftBtn, 1, 0);
    manualLayout->addWidget(downBtn, 2, 1);
    manualLayout->addWidget(rightBtn, 1, 2);
    menuLayout->addLayout(manualLayout);


    QHBoxLayout* cancelSaveLayout = new QHBoxLayout();
    CustomPushButton* cancelBtn = new CustomPushButton("Cancel", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(cancelBtn);
    connect(cancelBtn, SIGNAL(clicked()), this, SLOT(cancelSlot()));

    CustomPushButton* saveBtn = new CustomPushButton("Save", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(saveBtn);
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveSlot()));
    menuLayout->addLayout(cancelSaveLayout);

    layout->addWidget(menuWidget);

    menuWidget->setFixedWidth(150);
    topMenuLayout->setContentsMargins(0, 0, 0, 0);
    cancelSaveLayout->setContentsMargins(0, 0, 0, 0);
    menuLayout->setContentsMargins(0, 0, 5, 0);

    topMenuLayout->setAlignment(Qt::AlignTop);
    manualLayout->setAlignment(Qt::AlignBottom);
    cancelSaveLayout->setAlignment(Qt::AlignBottom);
}


void ScanMapWidget::initializeMap(){
    scene = new QGraphicsScene(this);

    /// Set the background of the scene as the same grey used in the map
    scene->setBackgroundBrush(QBrush(QColor(205, 205, 205)));

    graphicsView = new CustomQGraphicsView(scene, this);
    graphicsView->setCatchKeyEvent(true);

    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

}

void ScanMapWidget::addImageRobotSlot(){
    qDebug() << "ScanMapWidget::addImageRobotSlot called";

    /// If we have robots, open a menu to select from which robot we want the map
    if(robots->getRobotsVector().size() > 0){
        QMenu menu(this);
        QStringList list;
        for(int i = 0; i < listWidget->count(); i++)
            list.push_back(static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)))->getRobotName());

        for(int i = 0; i < robots->getRobotsVector().size(); i++){
            menu.addAction(robots->getRobotsVector().at(i)->getRobot()->getName());
            if(list.contains(robots->getRobotsVector().at(i)->getRobot()->getName())){
                menu.actions().last()->setEnabled(false);
                menu.actions().last()->setToolTip(menu.actions().last()->text() + " is already scanning");
            }
        }

        connect(&menu, SIGNAL(triggered(QAction*)), this, SLOT(robotMenuSlot(QAction*)));
        menu.exec(QCursor::pos());

    } else {
        QMessageBox msgBox;
        msgBox.setText("No robots connected.");
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

void ScanMapWidget::robotMenuSlot(QAction* action){
    qDebug() << "ScanMapWidget::robotMenuSlot called" << action->text();
    emit startScanning(action->text());
}

void ScanMapWidget::startedScanningSlot(QString robotName, bool scanning){
    qDebug() << "ScanMapWidget::startedScanningSlot called" << robotName << scanning;
    if(scanning)
        addMap(robotName);
    else {
        QMessageBox msgBox;
        msgBox.setText(robotName + "could not start scanning, please try again");
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

void ScanMapWidget::addMap(QString name){
    ScanMapListItemWidget* listItem = new ScanMapListItemWidget(listWidget->count(), name, scene);

    /*
    if(fromRobot){
        if(listWidget->count() == 0)
            originalSize = image.size();
    } else {
        if(listWidget->count() == 0)
            originalSize = QImage(fileName,"PGM").size();
    }

    connect(listItem, SIGNAL(deleteMap(int)), this, SLOT(deleteMapSlot(int)));
    connect(listItem, SIGNAL(pixmapClicked(int)), this, SLOT(selectPixmap(int)));*/
    connect(listItem, SIGNAL(deleteMap(int, QString)), this, SLOT(deleteMapSlot(int, QString)));
    connect(listItem, SIGNAL(playScan(bool, QString)), this, SLOT(playScanSlot(bool, QString)));
    connect(listItem, SIGNAL(robotGoTo(QString, double, double)), this, SLOT(robotGoToSlot(QString, double, double)));


    /// We add the path point widget to the list
    QListWidgetItem* listWidgetItem = new QListWidgetItem(listWidget);
    listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), LIST_WIDGET_HEIGHT));
    listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));

    listWidget->addItem(listWidgetItem);
    listWidget->setItemWidget(listWidgetItem, listItem);
}

void ScanMapWidget::cancelSlot(){
    qDebug() << "ScanMapWidget::cancelSlot Closing the edit widget";
    close();
}

void ScanMapWidget::saveSlot(){
    qDebug() << "ScanMapWidget::saveSlot called";
}

void ScanMapWidget::closeEvent(QCloseEvent *event){
    qDebug() << "ScanMapWidget::closeEvent";
    QStringList list;

    for(int i = 0; i < listWidget->count(); i++){
        ScanMapListItemWidget* item = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        list.push_back(item->getRobotName());
    }

    qDebug() << "ScanMapWidget::closeEvent" << list.count() << "robot(s) to stop scanning :" << list;
    if(list.count() > 0)
        emit stopScanning(list);
    QWidget::closeEvent(event);
}

void ScanMapWidget::robotDisconnectedSlot(QString robotName){
    for(int i = 0; i < listWidget->count(); i++){
        ScanMapListItemWidget* item = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName() == robotName)
            item->robotConnected(false);
    }
}

void ScanMapWidget::deleteMapSlot(int id, QString robotName){
    qDebug() << "MergeMapWidget::deleteMapSlot Removing map" << id << "coming from robot" << robotName;
    /// Tell the robot to stop scanning
    QStringList list;
    list.push_back(robotName);
    emit stopScanning(list);


    /// Remove the QGraphicsPixmapItem from the scene
    QListWidgetItem* listWidgetItem = listWidget->item(id);
    QGraphicsPixmapItem* pixmap = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidgetItem))->getPixmapItem();
    if(pixmap)
        scene->removeItem(pixmap);

    /// Delete the widget in the QListWidgetItem
    delete listWidget->itemWidget(listWidgetItem);

    /// Delete the QListWidgetItem
    delete listWidget->takeItem(id);

    refreshIds();
}

void ScanMapWidget::refreshIds(){
    for(int i = 0; i < listWidget->count(); i++)
        static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)))->setId(i);
}

void ScanMapWidget::robotReconnectedSlot(QString robotName){
    for(int i = 0; i < listWidget->count(); i++){
        ScanMapListItemWidget* item = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName() == robotName)
            item->robotConnected(true);
    }
}

void ScanMapWidget::playScanSlot(bool scan, QString robotName){
    /// Emit to give it to the mainWindow
    emit playScan(scan, robotName);
}

void ScanMapWidget::robotScanningSlot(bool scan, QString robotName, bool success){
    for(int i = 0; i < listWidget->count(); i++){
        ScanMapListItemWidget* item = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName() == robotName){
            item->robotScanning(scan == success);
        }
    }

    if(!success){
        QString msg;
        if(scan)
            msg = "Failed to launch the scan for the robot : " + robotName + "\nPlease try again.";
        else
            msg = "Failed to stop the scan for the robot : " + robotName + "\nPlease try again.";

        QMessageBox msgBox;
        msgBox.setText(msg);
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

void ScanMapWidget::receivedScanMapSlot(QString robotName, QImage map){
    for(int i = 0; i < listWidget->count(); i++){
        ScanMapListItemWidget* item = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName() == robotName)
            item->updateMap(map);
    }
}

void ScanMapWidget::robotGoToSlot(QString robotName, double x, double y){
    /// Emit to give it to the mainWindow
    emit robotGoTo(robotName, x, y);
}

void ScanMapWidget::scanRobotPosSlot(QString robotName, double x, double y, double ori){
    for(int i = 0; i < listWidget->count(); i++){
        ScanMapListItemWidget* item = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName() == robotName)
            item->updateRobotPos(x, y, ori);
    }
}
