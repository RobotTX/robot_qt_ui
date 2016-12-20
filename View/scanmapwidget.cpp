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

    QLabel* titleLabel = new QLabel("Merge Maps", this);
    QFont tmpFont = font();
    tmpFont.setPointSize(13);
    setFont(tmpFont);
    titleLabel->setFont(tmpFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    topMenuLayout->addWidget(titleLabel);

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    topMenuLayout->addWidget(spaceWidget);

    CustomPushButton* addImageFileBtn = new CustomPushButton("Add map from file", this);
    addImageFileBtn->setToolTip("Add a map from a pre-existing file");
    connect(addImageFileBtn, SIGNAL(clicked()), this, SLOT(addImageFileSlot()));
    topMenuLayout->addWidget(addImageFileBtn);

    CustomPushButton* addImageRobotBtn = new CustomPushButton("Start a scan with another robot", this);
    addImageRobotBtn->setToolTip("Start to scan with another robot");
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

void ScanMapWidget::addImageFileSlot(){
    qDebug() << "ScanMapWidget::addImageFileSlot called";

    /// Get the file name of the map we want to use
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Image"), "", tr("Image Files (*.pgm)"));

    if(!fileName.isEmpty())
        addMap(fileName, false);
}

void ScanMapWidget::addImageRobotSlot(){
    qDebug() << "MergeMapWidget::addImageRobotSlot called";
}

void ScanMapWidget::addMap(QString fileName, bool fromRobot){
    ScanMapListItemWidget* listItem;

    listItem = new ScanMapListItemWidget(listWidget->count(), fileName, scene, fromRobot);

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
