#include "mergemapwidget.h"
#include <QHBoxLayout>
#include <QDebug>
#include "View/customqgraphicsview.h"
#include "View/custompushbutton.h"
#include "View/spacewidget.h"
#include <QButtonGroup>
#include "View/mergemaplistitemwidget.h"
#include <QListWidgetItem>
#include "stylesettings.h"

MergeMapWidget::MergeMapWidget(QWidget *parent) : QWidget(parent){
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

MergeMapWidget::~MergeMapWidget(){
}

void MergeMapWidget::initializeMenu(){
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

    CustomPushButton* resetBtn = new CustomPushButton(QIcon(":/icons/empty.png"), "Reset", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center");
    topMenuLayout->addWidget(resetBtn);
    connect(resetBtn, SIGNAL(clicked()), this, SLOT(resetSlot()));

    /// Undo redo layout
    QHBoxLayout* undoRedoLayout = new QHBoxLayout();
    CustomPushButton* undoBtn = new CustomPushButton(QIcon(":/icons/undo.png"), "", this);
    undoRedoLayout->addWidget(undoBtn);
    connect(undoBtn, SIGNAL(clicked()), this, SLOT(undoSlot()));

    CustomPushButton* redoBtn = new CustomPushButton(QIcon(":/icons/redo.png"), "", this);
    undoRedoLayout->addWidget(redoBtn);
    connect(redoBtn, SIGNAL(clicked()), this, SLOT(redoSlot()));
    topMenuLayout->addLayout(undoRedoLayout);

    CustomPushButton* addImageFileBtn = new CustomPushButton("Add image from file", this);
    connect(addImageFileBtn, SIGNAL(clicked()), this, SLOT(addImageFileSlot()));
    topMenuLayout->addWidget(addImageFileBtn);

    CustomPushButton* addImageRobotBtn = new CustomPushButton("Add image from robot", this);
    connect(addImageRobotBtn, SIGNAL(clicked()), this, SLOT(addImageRobotSlot()));
    topMenuLayout->addWidget(addImageRobotBtn);


    listWidget = new QListWidget(this);

    //listWidget->setDragDropMode(QAbstractItemView::InternalMove);
    listWidget->setFrameShape(QFrame::NoFrame);
    listWidget->viewport()->setAutoFillBackground(false);
    listWidget->setAttribute(Qt::WA_MacShowFocusRect, false);
    listWidget->setStyleSheet(" QListWidget {color: red;}\
                  QListWidget::item {border-bottom: 1px solid; border-bottom-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #949494, stop: 1 transparent); }\
                      QListWidget::item:hover {background-color:"+button_hover_color+";}\
                      QListWidget::item:selected {background-color:"+button_checked_color+";}");

    topMenuLayout->addWidget(listWidget);

    menuLayout->addLayout(topMenuLayout);

    /// Save cancel layout
    QHBoxLayout* cancelSaveLayout = new QHBoxLayout();
    CustomPushButton* cancelBtn = new CustomPushButton("Cancel", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(cancelBtn);
    connect(cancelBtn, SIGNAL(clicked()), this, SLOT(cancelSlot()));

    CustomPushButton* saveBtn = new CustomPushButton("Save", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(saveBtn);
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveSlot()));
    menuLayout->addLayout(cancelSaveLayout);

    /// Some shortcuts to undo and redo
    QShortcut* undoShortcut = new QShortcut(QKeySequence(tr("Ctrl+Z", "Undo")), this);
    connect(undoShortcut, SIGNAL(activated()), this, SLOT(undoSlot()));

    QShortcut* redoShortcut = new QShortcut(QKeySequence(tr("Ctrl+Y", "Redo")), this);
    connect(redoShortcut, SIGNAL(activated()), this, SLOT(redoSlot()));

    layout->addWidget(menuWidget);

    menuWidget->setFixedWidth(150);
    undoRedoLayout->setContentsMargins(0, 0, 0, 0);
    topMenuLayout->setContentsMargins(0, 0, 0, 0);
    cancelSaveLayout->setContentsMargins(0, 0, 0, 0);
    menuLayout->setContentsMargins(0, 0, 5, 0);

    topMenuLayout->setAlignment(Qt::AlignTop);
    cancelSaveLayout->setAlignment(Qt::AlignBottom);
}

void MergeMapWidget::initializeMap(){
    scene = new QGraphicsScene(this);
    scene->setBackgroundBrush(QBrush(QColor(205, 205, 205)));
    graphicsView = new CustomQGraphicsView(scene, this);

    graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
}


void MergeMapWidget::resetSlot(){
    qDebug() << "MergeMapWidget::resetSlot called";
}

void MergeMapWidget::undoSlot(){
    qDebug() << "MergeMapWidget::undoSlot called";
}

void MergeMapWidget::redoSlot(){
    qDebug() << "MergeMapWidget::redoSlot called";
}

void MergeMapWidget::addImageFileSlot(){
    qDebug() << "MergeMapWidget::addImageFileSlot called";

    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Image"), "", tr("Image Files (*.pgm)"));

    if(!fileName.isEmpty()){
        MergeMapListItemWidget* listItem = new MergeMapListItemWidget(listWidget->count(), fileName, scene);
        connect(listItem, SIGNAL(deleteMap(int)), this, SLOT(deleteMapSlot(int)));

        /// We add the path point widget to the list
        QListWidgetItem* listWidgetItem = new QListWidgetItem(listWidget);
        listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), MERGE_WIDGET_HEIGHT));
        listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));

        listWidget->addItem(listWidgetItem);
        listWidget->setItemWidget(listWidgetItem, listItem);
    }
}

void MergeMapWidget::addImageRobotSlot(){
    qDebug() << "MergeMapWidget::addImageRobotSlot called";
}

void MergeMapWidget::cancelSlot(){
    qDebug() << "MergeMapWidget::cancelSlot Closing the edit widget";
    close();
}

void MergeMapWidget::saveSlot(){
    qDebug() << "MergeMapWidget::saveSlot called";
}

void MergeMapWidget::deleteMapSlot(int itemId){
    qDebug() << "MergeMapWidget::deleteMapSlot Removing item" << itemId;
    QListWidgetItem* listWidgetItem = listWidget->item(itemId);

    /// Remove the QGraphicsPixmapItem from the scene
    scene->removeItem(static_cast<MergeMapListItemWidget*>(listWidget->itemWidget(listWidgetItem))->getPixmapItem());

    /// Delete the widget in the QListWidgetItem
    delete listWidget->itemWidget(listWidgetItem);

    /// Delete the QListWidgetItem
    delete listWidget->takeItem(itemId);

    refreshIds();
}

void MergeMapWidget::refreshIds(){
    for(int i = 0; i < listWidget->count(); i++)
        static_cast<MergeMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)))->setId(i);
}
