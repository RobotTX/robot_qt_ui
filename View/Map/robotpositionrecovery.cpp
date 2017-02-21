#include "robotpositionrecovery.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QButtonGroup>
#include <QDebug>
#include <QApplication>
#include <QDesktopWidget>
#include <QGraphicsScene>
#include <QMenu>
#include "Model/Robots/robots.h"
#include "View/Map/customqgraphicsview.h"
#include "View/Robots/robotview.h"
#include "View/Other/spacewidget.h"
#include "View/Other/custompushbutton.h"
#include "View/Map/mergemaplistwidget.h"
#include "View/Map/teleopwidget.h"
#include "View/Map/robotpositionrecoverylistitemwidget.h"
#include "View/Map/recoverpositionmapgraphicsitem.h"

RobotPositionRecovery::RobotPositionRecovery(QSharedPointer<Robots> _robots, QWidget *parent) : QWidget(parent), robots(_robots), mapSize(QSize())
{
    setAttribute(Qt::WA_DeleteOnClose);

    /// If mouse tracking is enabled, the widget receives mouse move events even if no buttons are pressed.
    setMouseTracking(true);

    mainLayout = new QHBoxLayout(this);

    initializeMap();

    initializeMenu();

    /// to add the maps received from the robot
    mainLayout->addWidget(graphicsView);

    resize(800, 600);
    show();

    /// We center the window on the desktop
    QRect screenGeometry = QApplication::desktop()->screenGeometry();
    int x = (screenGeometry.width() - width()) / 2;
    int y = (screenGeometry.height() - height()) / 2;
    move(x, y);
}

void RobotPositionRecovery::initializeMenu(){
    /// to create a widget that can contain all the left part of the whole widget
    /// and later on be added to the main layout
    QWidget* menuWidget = new QWidget(this);

    /// the layout containing the left part of the whole widget
    /// including the title, button, list of robots and teleoperation widget
    QVBoxLayout* leftLayout = new QVBoxLayout(menuWidget);

    QVBoxLayout* topMenuLayout = new QVBoxLayout();

    /// Title
    QLabel* titleLabel = new QLabel("Recover robots's positions");
    QFont tmpFont = font();
    tmpFont.setPointSize(13);
    setFont(tmpFont);
    titleLabel->setFont(tmpFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    topMenuLayout->addWidget(titleLabel);

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    topMenuLayout->addWidget(spaceWidget);

    /// Button to start the recovery
    CustomPushButton* startRecoveryButton = new CustomPushButton("Recover the position of a robot", this);
    startRecoveryButton->setToolTip("If your robot is lost you can try to use this feature to recover its position");
    connect(startRecoveryButton, SIGNAL(clicked()), this, SLOT(addImageRobotSlot()));
    topMenuLayout->addWidget(startRecoveryButton);

    /// The widget that lists every robot trying to recover its position
    listWidget = new MergeMapListWidget(this);
    connect(listWidget, SIGNAL(dirKeyPressed(int)), this, SLOT(dirKeyEventSlot(int)));
    topMenuLayout->addWidget(listWidget);

    leftLayout->addLayout(topMenuLayout);

    /// Teleoperation widget
    QVBoxLayout* teleopLayout = new QVBoxLayout();
    TeleopWidget* teleopWidget = new TeleopWidget(this);
    connect(teleopWidget->getBtnGroup(), SIGNAL(buttonClicked(int)), this, SLOT(teleopCmdSlot(int)));
    teleopLayout->addWidget(teleopWidget);
    leftLayout->addLayout(teleopLayout);

    mainLayout->addWidget(menuWidget);

    /// ensures that the menu on the left is not taking too much space
    menuWidget->setFixedWidth(150);
    teleopLayout->setContentsMargins(0, 0, 0, 0);
    topMenuLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setContentsMargins(0, 0, 5, 0);

    topMenuLayout->setAlignment(Qt::AlignTop);
    teleopLayout->setAlignment(Qt::AlignBottom);
}

void RobotPositionRecovery::initializeMap(){
    scene = new QGraphicsScene(this);

    /// Set the background of the scene as the same grey used in the map
    /// so that it looks better when a map is received with a lot of grey
    scene->setBackgroundBrush(QBrush(QColor(205, 205, 205)));

    graphicsView = new CustomQGraphicsView(scene, this);
    graphicsView->setCatchKeyEvent(true);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
}

void RobotPositionRecovery::dirKeyEventSlot(int key){
    if(listWidget->currentItem() != NULL){
        RobotPositionRecoveryListItemWidget* widget = static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->currentItem()));
        switch(key){
            /// the arrows are used to move the manipulate the map
            case Qt::Key_Up:
                widget->getPixmapItem()->moveBy(0, -0.1);
            break;
            case Qt::Key_Down:
                widget->getPixmapItem()->moveBy(0, 0.1);
            break;
            case Qt::Key_Left:
                widget->getPixmapItem()->moveBy(-0.1, 0);
            break;
            case Qt::Key_Right:
                widget->getPixmapItem()->moveBy(0.1, 0);
            break;
            case Qt::Key_U:
                teleopCmdSlot(0);
            break;
            case Qt::Key_I:
                teleopCmdSlot(1);
            break;
            case Qt::Key_O:
                teleopCmdSlot(2);
            break;
            case Qt::Key_J:
                teleopCmdSlot(3);
            break;
            case Qt::Key_L:
                teleopCmdSlot(5);
            break;
            case Qt::Key_M:
                teleopCmdSlot(6);
            break;
            case Qt::Key_Comma:
                teleopCmdSlot(7);
            break;
            case Qt::Key_Period:
                teleopCmdSlot(8);
            break;
            default:
                teleopCmdSlot(4);
            break;
        }
    }
}

void RobotPositionRecovery::teleopCmdSlot(int id){
    qDebug() << "RobotPositionRecovery::teleopCmd" << id;
    if(listWidget->currentItem() != NULL){
        QString robotName = static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->currentItem()))->getRobotName();
        qDebug() << "RobotPositionRecovery::teleopCmd" << robotName;
        emit teleopCmd(robotName, id);
    }
}

void RobotPositionRecovery::addImageRobotSlot(){
    qDebug() << "RobotPositionRecovery::addImageRobotSlot called";

    /// If we have robots, open a menu to select from which robot we want the map
    if(robots->getRobotsVector().size() > 0){
        QMenu menu(this);
        QStringList list;

        /// Create a list of already recovering robots so we can't add them twice
        for(int i = 0; i < listWidget->count(); i++)
            list.push_back(static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->item(i)))->getRobotName());

        /// Add the available robots to the list + disable the one already recovering
        for(int i = 0; i < robots->getRobotsVector().size(); i++){
            menu.addAction(robots->getRobotsVector().at(i)->getRobot()->getName());
            if(list.contains(robots->getRobotsVector().at(i)->getRobot()->getName())){
                menu.actions().last()->setEnabled(false);
                menu.actions().last()->setToolTip(menu.actions().last()->text() + " is already recovering its position");
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


void RobotPositionRecovery::receivedMapSlot(QString robotName, QImage map, double _resolution){
    qDebug() << "ReceivedMapSlot called in robotpositionrecovery";
    Q_UNUSED(_resolution)
    mapSize = map.size();
    /// looks for the right item in the list to update the map of the corresponding robot
    for(int i = 0; i < listWidget->count(); i++){
        RobotPositionRecoveryListItemWidget* item = static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName().compare(robotName) == 0)
            item->updateMap(map);
    }
}

void RobotPositionRecovery::cancelSlot(){
    qDebug() << "RobotPositionRecovery::cancelSlot Closing the edit widget";
    close();
}

/// to be able to process any key
void RobotPositionRecovery::keyPressEvent(QKeyEvent *event){
    dirKeyEventSlot(event->key());
}

/// to center on the map of a particular robot
void RobotPositionRecovery::centerOnSlot(QGraphicsItem* pixmap){
    graphicsView->centerOn(pixmap);
}

void RobotPositionRecovery::updateRobotPosSlot(QString robotName, double x, double y, double ori){
    for(int i = 0; i < listWidget->count(); i++){
        RobotPositionRecoveryListItemWidget* item = static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName().compare(robotName) == 0)
            item->updateRobotPos(x, y, ori);
    }
}

/// sets a goal for the robot
void RobotPositionRecovery::robotGoToSlot(QString robotName, double x, double y){
    /// Emit to give it to the mainWindow
    emit robotGoTo(robotName, x, y);
}

/// if the widget is closed we make sure to stop the recovery of the robots
void RobotPositionRecovery::closeEvent(QCloseEvent *event){
    qDebug() << "RobotPositionRecovery::close event";
    QStringList list = getAllRecoveringRobots();
    if(list.count() > 0)
        emit stopRecoveringRobots(list);
    QWidget::closeEvent(event);
}

/// get the names of the robots recovering their position as a list
QStringList RobotPositionRecovery::getAllRecoveringRobots() const {
    QStringList list;
    for(int i = 0; i < listWidget->count(); i++){
        RobotPositionRecoveryListItemWidget* item = static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        list.push_back(item->getRobotName());
    }
    return list;
}

void RobotPositionRecovery::robotMenuSlot(QAction* action){
    qDebug() << "RobotPositionRecovery::robotMenuSlot called" << action->text();
    emit startRecovering(action->text());
}

void RobotPositionRecovery::startedRecoveringSlot(QString robotName, bool recovering){
    qDebug() << "RobotPositionRecovery::startedRecoveringSlot called" << robotName << recovering;
    if(recovering)
        addMapWidget(robotName);
    else {
        QMessageBox msgBox;
        msgBox.setText(robotName + "could not start recovering, please try again");
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

void RobotPositionRecovery::addMapWidget(QString name){
    qDebug() << "RobotPositionRecovery::addMapWidget" << name;
    RobotPositionRecoveryListItemWidget* listItem = new RobotPositionRecoveryListItemWidget(listWidget->count(), name, robots, scene);

    connect(listItem, SIGNAL(deleteMap(int, QString)), this, SLOT(deleteMapSlot(int, QString)));
    connect(listItem, SIGNAL(playRecovery(bool, QString)), this, SLOT(playRecoverySlot(bool,QString)));
    connect(listItem, SIGNAL(robotGoTo(QString, double, double)), this, SLOT(robotGoToSlot(QString, double, double)));
    connect(listItem, SIGNAL(centerOn(QGraphicsItem*)), this, SLOT(centerOnSlot(QGraphicsItem*)));

    /// We add the path point widget to the list
    QListWidgetItem* listWidgetItem = new QListWidgetItem(listWidget);
    listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), LIST_WIDGET_HEIGHT));
    listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));

    listWidget->addItem(listWidgetItem);
    listWidget->setItemWidget(listWidgetItem, listItem);
}

void RobotPositionRecovery::robotDisconnectedSlot(QString robotName){
    for(int i = 0; i < listWidget->count(); i++){
        RobotPositionRecoveryListItemWidget* item = static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName() == robotName)
            item->robotConnected(false);
    }
}

void RobotPositionRecovery::robotReconnectedSlot(QString robotName){
    for(int i = 0; i < listWidget->count(); i++){
        RobotPositionRecoveryListItemWidget* item = static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName().compare(robotName) == 0)
            item->robotConnected(true);
    }
}

void RobotPositionRecovery::robotRecoveringSlot(bool recover, QString robotName, bool success){
    for(int i = 0; i < listWidget->count(); i++){
        RobotPositionRecoveryListItemWidget* item = static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getRobotName().compare(robotName) == 0)
            item->robotRecovering(recover == success);
    }

    if(!success){
        QString msg = (recover) ? "Failed to launch the scan for the robot : " + robotName + "\nPlease try again." :
                               "Failed to stop the scan for the robot : " + robotName + "\nPlease try again.";
        QMessageBox msgBox;
        msgBox.setText(msg);
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

void RobotPositionRecovery::playRecoverySlot(bool recover, QString robotName){
    /// emit to give it to the mainWindow
    emit playRecovery(recover, robotName);
}

void RobotPositionRecovery::deleteMapSlot(int id, QString robotName){
    qDebug() << "MergeMapWidget::deleteMapSlot Removing map" << id << "coming from robot" << robotName;
    /// Tell the robot to stop scanning
    QStringList list;
    list.push_back(robotName);

    emit stopRecoveringRobots(list);

    /// Remove the QGraphicsPixmapItem from the scene
    QListWidgetItem* listWidgetItem = listWidget->item(id);
    QGraphicsPixmapItem* pixmap = static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidgetItem))->getPixmapItem();
    if(pixmap)
        scene->removeItem(pixmap);

    /// Delete the widget in the QListWidgetItem
    delete listWidget->itemWidget(listWidgetItem);

    /// Delete the QListWidgetItem
    delete listWidget->takeItem(id);

    refreshIds();
}

void RobotPositionRecovery::refreshIds(){
    for(int i = 0; i < listWidget->count(); i++)
        static_cast<RobotPositionRecoveryListItemWidget*>(listWidget->itemWidget(listWidget->item(i)))->setId(i);
}
