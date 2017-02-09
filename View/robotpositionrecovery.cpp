#include "robotpositionrecovery.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include "View/spacewidget.h"
#include "custompushbutton.h"
#include "mergemaplistwidget.h"
#include "View/teleopwidget.h"
#include <QButtonGroup>
#include <QDebug>
#include <QApplication>
#include <QDesktopWidget>
#include <QGraphicsScene>
#include "View/customqgraphicsview.h"
#include <QMenu>
#include "Model/robots.h"
#include "View/robotview.h"

RobotPositionRecovery::RobotPositionRecovery(QSharedPointer<Robots> _robots, QWidget *parent) : QWidget(parent), robots(_robots) {
    setAttribute(Qt::WA_DeleteOnClose);
    setMouseTracking(true);

    mainLayout = new QHBoxLayout(this);

    initializeMap();
    initializeMenu();

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
    connect(startRecoveryButton, SIGNAL(clicked()), this, SLOT(recoverRobotPosSlot()));
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

    topMenuLayout->setAlignment(Qt::AlignTop);
    teleopLayout->setAlignment(Qt::AlignBottom);
}

void RobotPositionRecovery::initializeMap(){
    scene = new QGraphicsScene(this);

    graphicsView = new CustomQGraphicsView(scene, this);
    graphicsView->setCatchKeyEvent(true);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

}

void RobotPositionRecovery::dirKeyEventSlot(int key){
    /*
    if(listWidget->currentItem() != NULL){
        ScanMapListItemWidget* widget = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->currentItem()));
        switch(key){
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
    */
}

void RobotPositionRecovery::teleopCmdSlot(int id){
    /*
    qDebug() << "RobotPositionRecovery::teleopCmd" << id;
    if(listWidget->currentItem() != NULL){
        QString robotName = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->currentItem()))->getRobotName();
        qDebug() << "RobotPositionRecovery::teleopCmd" << robotName;
        emit teleopCmd(robotName, id);
    }
    */
}

void RobotPositionRecovery::recoverRobotPosSlot(){
    qDebug() << "RobotPositionRecovery::recoverRobotPosSlot called";

    /// If we have robots, open a menu to select from which robot we want the map
    if(robots->getRobotsVector().size() > 0){
        QMenu menu(this);
        QStringList list;

        /// right now only one recovery at the time
        /*
        /// Create a list of already scanning robots so we can't add them twice
        for(int i = 0; i < listWidget->count(); i++)
            list.push_back(static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)))->getRobotName());
    */
        /// Add the available robots to the list + disable the one already scanning
        for(int i = 0; i < robots->getRobotsVector().size(); i++){
            menu.addAction(robots->getRobotsVector().at(i)->getRobot()->getName());
            /*
            if(list.contains(robots->getRobotsVector().at(i)->getRobot()->getName())){
                menu.actions().last()->setEnabled(false);
                menu.actions().last()->setToolTip(menu.actions().last()->text() + " is already scanning");
            }*/
        }

        connect(&menu, SIGNAL(triggered(QAction*)), this, SLOT(startRecoveringSlot(QAction*)));
        menu.exec(QCursor::pos());

    } else {
        QMessageBox msgBox;
        msgBox.setText("No robots connected.");
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

void RobotPositionRecovery::startRecoveringSlot(){

}
