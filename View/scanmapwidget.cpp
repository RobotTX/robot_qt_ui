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
    connect(listWidget, SIGNAL(dirKeyPressed(int)), this, SLOT(dirKeyEventSlot(int)));
    topMenuLayout->addWidget(listWidget);


    menuLayout->addLayout(topMenuLayout);

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
        for(int i = 0; i < robots->getRobotsVector().size(); i++)
            menu.addAction(robots->getRobotsVector().at(i)->getRobot()->getName());

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
        /// TODO msg in QMessageBox
        qDebug() << "ScanMapWidget::startedScanningSlot" << robotName << "could not start scanning, please try again";
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
    }*/

    connect(listItem, SIGNAL(deleteMap(int)), this, SLOT(deleteMapSlot(int)));
    connect(listItem, SIGNAL(pixmapClicked(int)), this, SLOT(selectPixmap(int)));

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

/// TODO on close, stop scanning
