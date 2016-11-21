#include "editmapwidget.h"
#include <QDebug>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include "Model/map.h"
#include "View/customqgraphicsview.h"
#include "View/editmapview.h"
#include "View/custompushbutton.h"
#include <QLabel>
#include "View/stylesettings.h"
#include "View/spacewidget.h"
#include <QButtonGroup>

EditMapWidget::EditMapWidget(QImage _mapImage, int _width, int _height, QWidget* parent):
    QWidget(parent), mapImage(_mapImage), mapWidth(_width), mapHeight(_height){
    qDebug() << "EditMapWidget::EditMapWidget constructor";
    setAttribute(Qt::WA_DeleteOnClose);
    layout = new QHBoxLayout(this);

    initializeMenu();
    initializeMap();

    show();

    /// We center the window on the desktop
    QRect screenGeometry = QApplication::desktop()->screenGeometry();
    int x = (screenGeometry.width() - width()) / 2;
    int y = (screenGeometry.height() - height()) / 2;
    move(x, y);
    centerMap();
}

void EditMapWidget::initializeMenu(){
    QWidget* menuWidget = new QWidget(this);
    QVBoxLayout* menuLayout = new QVBoxLayout(menuWidget);
    QVBoxLayout* topMenuLayout = new QVBoxLayout();


    QLabel* titleLabel = new QLabel("Edit a Map", this);
    QFont tmpFont = font();
    tmpFont.setPointSize(13);
    setFont(tmpFont);
    titleLabel->setFont(tmpFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    topMenuLayout->addWidget(titleLabel);

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    topMenuLayout->addWidget(spaceWidget);

    /// Undo redo layout
    QHBoxLayout* undoRedoLayout = new QHBoxLayout();
    CustomPushButton* undoBtn = new CustomPushButton(QIcon(":/icons/undo.png"), "", this);
    undoRedoLayout->addWidget(undoBtn);
    connect(undoBtn, SIGNAL(clicked()), this, SLOT(undoSlot()));

    CustomPushButton* redoBtn = new CustomPushButton(QIcon(":/icons/redo.png"), "", this);
    undoRedoLayout->addWidget(redoBtn);
    connect(redoBtn, SIGNAL(clicked()), this, SLOT(redoSlot()));
    topMenuLayout->addLayout(undoRedoLayout);


    /// Color layout
    /// black grey white
    QHBoxLayout* colorLayout = new QHBoxLayout();
    QLabel* colorLabel = new QLabel("Select a color :", this);
    topMenuLayout->addWidget(colorLabel);

    QButtonGroup* colorGroup = new QButtonGroup(this);
    CustomPushButton* blackBtn = new CustomPushButton(QIcon(":/icons/black_square.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    blackBtn->setChecked(true);
    colorGroup->addButton(blackBtn, 0);
    colorLayout->addWidget(blackBtn);

    CustomPushButton* greyBtn = new CustomPushButton(QIcon(":/icons/grey_square.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    colorGroup->addButton(greyBtn, 1);
    colorLayout->addWidget(greyBtn);

    CustomPushButton* whiteBtn = new CustomPushButton(QIcon(":/icons/white_square.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    colorGroup->addButton(whiteBtn, 2);
    colorLayout->addWidget(whiteBtn);

    connect(colorGroup, SIGNAL(buttonClicked(int)), this, SLOT(changeColorSlot(int)));
    topMenuLayout->addLayout(colorLayout);


    /// Shape layout
    /// hand bucket point line rectangle filled rectangle
    QGridLayout* shapeLayout = new QGridLayout();
    QLabel* shapeLabel = new QLabel("Select a shape :", this);
    topMenuLayout->addWidget(shapeLabel);

    QButtonGroup* shapeGroup = new QButtonGroup(this);
    CustomPushButton* handBtn = new CustomPushButton(QIcon(":/icons/hand.png"), "", this, false, CustomPushButton::ButtonType::TOP, "center", true);
    handBtn->setChecked(true);
    shapeGroup->addButton(handBtn, 0);
    shapeLayout->addWidget(handBtn, 0, 0);

    CustomPushButton* bucketBtn = new CustomPushButton(QIcon(":/icons/bucket.png"), "", this, false, CustomPushButton::ButtonType::TOP, "center", true);
    shapeGroup->addButton(bucketBtn, 1);
    shapeLayout->addWidget(bucketBtn, 0, 1);

    CustomPushButton* dotBtn = new CustomPushButton(QIcon(":/icons/dot.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    shapeGroup->addButton(dotBtn, 2);
    shapeLayout->addWidget(dotBtn, 1, 0);

    CustomPushButton* lineBtn = new CustomPushButton(QIcon(":/icons/line.png"), "", this, false, CustomPushButton::ButtonType::TOP, "center", true);
    shapeGroup->addButton(lineBtn, 3);
    shapeLayout->addWidget(lineBtn, 1, 1);

    CustomPushButton* rectangleBtn = new CustomPushButton(QIcon(":/icons/rectangle.png"), "", this, false, CustomPushButton::ButtonType::TOP, "center", true);
    shapeGroup->addButton(rectangleBtn, 4);
    shapeLayout->addWidget(rectangleBtn, 2, 0);

    CustomPushButton* filledRectangleBtn = new CustomPushButton(QIcon(":/icons/filled_rectangle.png"), "", this, false, CustomPushButton::ButtonType::TOP, "center", true);
    shapeGroup->addButton(filledRectangleBtn, 5);
    shapeLayout->addWidget(filledRectangleBtn, 2, 1);

    connect(shapeGroup, SIGNAL(buttonClicked(int)), this, SLOT(changeShapeSlot(int)));
    topMenuLayout->addLayout(shapeLayout);


    /// Size layout
    /// small medium big + lineedit



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


    layout->addWidget(menuWidget);
    menuWidget->setFixedWidth(150);

    colorLayout->setContentsMargins(0, 0, 0, 0);
    shapeLayout->setContentsMargins(0, 0, 0, 0);
    undoRedoLayout->setContentsMargins(0, 0, 0, 0);
    topMenuLayout->setContentsMargins(0, 0, 0, 0);
    cancelSaveLayout->setContentsMargins(0, 0, 0, 0);
    menuLayout->setContentsMargins(0, 0, 5, 0);
    //layout->setContentsMargins(0, 0, 0, 0);

    topMenuLayout->setAlignment(Qt::AlignTop);
    cancelSaveLayout->setAlignment(Qt::AlignBottom);
}

void EditMapWidget::initializeMap(){
    scene = new QGraphicsScene(this);
    graphicsView = new CustomQGraphicsView(scene, this);
    scene->setSceneRect(0, 0, mapWidth, mapHeight);

    /*graphicsView->scale(std::max(graphicsView->parentWidget()->width()/scene->width(), graphicsView->parentWidget()->height()/scene->height()),
                        std::max(graphicsView->parentWidget()->width()/scene->width(), graphicsView->parentWidget()->height()/scene->height()));
*/
    graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

    /// Create the graphic item of the map
    QPixmap pixmap = QPixmap::fromImage(mapImage);
    pixmapItem = new EditMapView(pixmap);

    scene->addItem(pixmapItem);

    layout->addWidget(graphicsView);
}

void EditMapWidget::centerMap(){
    /// centers the map on the point chosen by the user and not on the center of the map
    //pixmapItem->setPos(mapState.first);
    graphicsView->centerOn(pixmapItem->boundingRect().width()/2, pixmapItem->boundingRect().height()/2);
}

void EditMapWidget::cancelSlot(){
    qDebug() << "EditMapWidget::cancelSlot Closing the edit widget";
    close();
}

void EditMapWidget::saveSlot(){
    qDebug() << "EditMapWidget::saveSlot called";
}

void EditMapWidget::undoSlot(){
    qDebug() << "EditMapWidget::undoSlot called";
}

void EditMapWidget::redoSlot(){
    qDebug() << "EditMapWidget::redoSlot called";
}

void EditMapWidget::changeColorSlot(int color){
    qDebug() << "EditMapWidget::changeColorSlot called" << color;
}

void EditMapWidget::changeShapeSlot(int shape){
    qDebug() << "EditMapWidget::changeShapeSlot called" << shape;
}
