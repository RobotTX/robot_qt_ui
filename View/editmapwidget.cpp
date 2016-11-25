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
#include "View/customlineedit.h"
#include "Model/map.h"

EditMapWidget::EditMapWidget(QImage _mapImage, int _width, int _height, float _mapResolution, Position _mapOrigin, QWidget* parent):
    QWidget(parent), mapImage(_mapImage), mapWidth(_width), mapHeight(_height), mapResolution(_mapResolution), mapOrigin(_mapOrigin){
    qDebug() << "EditMapWidget::EditMapWidget constructor";
    setAttribute(Qt::WA_DeleteOnClose);
    setMouseTracking(true);
    layout = new QHBoxLayout(this);

    initializeMap();
    initializeMenu();

    layout->addWidget(graphicsView);

    show();

    /// We center the window on the desktop
    QRect screenGeometry = QApplication::desktop()->screenGeometry();
    int x = (screenGeometry.width() - width()) / 2;
    int y = (screenGeometry.height() - height()) / 2;
    move(x, y);
    centerMap();
}


EditMapWidget::~EditMapWidget(){
    delete canvas;
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

    CustomPushButton* resetBtn = new CustomPushButton(QIcon(":/icons/empty.png"), "Reset", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center");
    topMenuLayout->addWidget(resetBtn);
    connect(resetBtn, SIGNAL(clicked()), canvas, SLOT(resetSlot()));

    /// Undo redo layout
    QHBoxLayout* undoRedoLayout = new QHBoxLayout();
    CustomPushButton* undoBtn = new CustomPushButton(QIcon(":/icons/undo.png"), "", this);
    undoRedoLayout->addWidget(undoBtn);
    connect(undoBtn, SIGNAL(clicked()), canvas, SLOT(undoSlot()));

    CustomPushButton* redoBtn = new CustomPushButton(QIcon(":/icons/redo.png"), "", this);
    undoRedoLayout->addWidget(redoBtn);
    connect(redoBtn, SIGNAL(clicked()), canvas, SLOT(redoSlot()));
    topMenuLayout->addLayout(undoRedoLayout);


    /// Color layout
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

    connect(colorGroup, SIGNAL(buttonClicked(int)), canvas, SLOT(changeColorSlot(int)));
    topMenuLayout->addLayout(colorLayout);


    /// Shape layout
    QGridLayout* shapeLayout = new QGridLayout();
    QLabel* shapeLabel = new QLabel("Select a shape :", this);
    topMenuLayout->addWidget(shapeLabel);

    QButtonGroup* shapeGroup = new QButtonGroup(this);
    CustomPushButton* handBtn = new CustomPushButton(QIcon(":/icons/hand.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    handBtn->setChecked(true);
    shapeGroup->addButton(handBtn, 0);
    topMenuLayout->addWidget(handBtn);

    CustomPushButton* dotBtn = new CustomPushButton(QIcon(":/icons/dot_l.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    shapeGroup->addButton(dotBtn, 1);
    shapeLayout->addWidget(dotBtn, 0, 0);

    CustomPushButton* lineBtn = new CustomPushButton(QIcon(":/icons/line.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    shapeGroup->addButton(lineBtn, 2);
    shapeLayout->addWidget(lineBtn, 0, 1);

    CustomPushButton* rectangleBtn = new CustomPushButton(QIcon(":/icons/rectangle.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    shapeGroup->addButton(rectangleBtn, 3);
    shapeLayout->addWidget(rectangleBtn, 1, 0);

    CustomPushButton* filledRectangleBtn = new CustomPushButton(QIcon(":/icons/filled_rectangle.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    shapeGroup->addButton(filledRectangleBtn, 4);
    shapeLayout->addWidget(filledRectangleBtn, 1, 1);

    connect(shapeGroup, SIGNAL(buttonClicked(int)), canvas, SLOT(changeShapeSlot(int)));
    topMenuLayout->addLayout(shapeLayout);


    /// Size layout
    QLabel* sizeLabel = new QLabel("Select a size :", this);
    topMenuLayout->addWidget(sizeLabel);


    QHBoxLayout* sizeBtnLayout = new QHBoxLayout();

    sizeGroup = new QButtonGroup(this);
    CustomPushButton* sizeSBtn = new CustomPushButton(QIcon(":/icons/dot_s.png"), "", this, false, CustomPushButton::ButtonType::TOP, "center", true);
    sizeSBtn->setChecked(true);
    sizeGroup->addButton(sizeSBtn, 1);
    sizeBtnLayout->addWidget(sizeSBtn);

    CustomPushButton* sizeMBtn = new CustomPushButton(QIcon(":/icons/dot_m.png"), "", this, false, CustomPushButton::ButtonType::TOP, "center", true);
    sizeGroup->addButton(sizeMBtn, 3);
    sizeBtnLayout->addWidget(sizeMBtn);

    CustomPushButton* sizeLBtn = new CustomPushButton(QIcon(":/icons/dot_l.png"), "", this, false, CustomPushButton::ButtonType::LEFT_MENU, "center", true);
    sizeGroup->addButton(sizeLBtn, 7);
    sizeBtnLayout->addWidget(sizeLBtn);

    connect(sizeGroup, SIGNAL(buttonClicked(int)), canvas, SLOT(changeSizeSlot(int)));
    connect(sizeGroup, SIGNAL(buttonClicked(int)), this, SLOT(changeLineEditSlot(int)));
    topMenuLayout->addLayout(sizeBtnLayout);


    QHBoxLayout* sizeEditLayout = new QHBoxLayout();
    sizeLineEdit = new CustomLineEdit("1", this);
    sizeLineEdit->setAlignment(Qt::AlignCenter);
    sizeLineEdit->setValidator(new QIntValidator(1, 200, this));
    sizeEditLayout->addWidget(sizeLineEdit);
    connect(sizeLineEdit, SIGNAL(editingFinished()), this, SLOT(changeSizeEditSlot()));
    connect(this, SIGNAL(changeSizeEdit(int)), canvas, SLOT(changeSizeSlot(int)));


    QLabel* pxLabel = new QLabel("px", this);
    sizeEditLayout->addWidget(pxLabel);
    topMenuLayout->addLayout(sizeEditLayout);



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
    connect(undoShortcut, SIGNAL(activated()), canvas, SLOT(undoSlot()));

    QShortcut* redoShortcut = new QShortcut(QKeySequence(tr("Ctrl+Y", "Redo")), this);
    connect(redoShortcut, SIGNAL(activated()), canvas, SLOT(redoSlot()));


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


    canvas = new EditMapView(mapImage.width(), mapImage.height());
    scene->addItem(canvas);


    /// Create the graphic item of the map
    QPixmap pixmap = QPixmap::fromImage(mapImage);
    pixmapItem = new QGraphicsPixmapItem(pixmap, canvas);
    pixmapItem->setZValue(0);
    pixmapItem->setFlag(QGraphicsItem::ItemStacksBehindParent);
    //scene->addItem(pixmapItem);
}

void EditMapWidget::centerMap(){
    /// centers the map on the point chosen by the user and not on the center of the map
    graphicsView->centerOn(pixmapItem->boundingRect().width()/2, pixmapItem->boundingRect().height()/2);
}

void EditMapWidget::cancelSlot(){
    qDebug() << "EditMapWidget::cancelSlot Closing the edit widget";
    close();
}

void EditMapWidget::saveSlot(){
    qDebug() << "EditMapWidget::saveSlot called";

    QVector<QPair<QVector<int>, QVector<QPointF>>> items = canvas->getItems();

    QPainter painter;
    painter.begin(&mapImage);

    for(int i = 0; i < items.size(); i++){
        if(items.at(i).first.size() > 2){
            int _color = items.at(i).first.at(0);
            int _shape = items.at(i).first.at(1);
            int _size = items.at(i).first.at(2);
            QVector<QPointF> points = items.at(i).second;

            switch(_shape){
                case 0:
                    qDebug() << "EditMapWidget::saveSlot should not be here (case 0)";
                break;
                case 1:
                    //qDebug() << "EditMapView::paint Drawing points";
                    painter.setPen(QPen(QColor(_color, _color, _color), _size));
                    for(int j = 0; j < points.size(); j++)
                        painter.drawPoint(points.at(j));

                break;
                case 2:{
                    //qDebug() << "EditMapWidget::saveSlot Drawing a line";
                    painter.setPen(QPen(QColor(_color, _color, _color), _size));
                    if(points.size() > 1){
                        QVector<QPointF> newPoints = canvas->getLine(points.at(0), points.at(1));
                        for(int j = 0; j < newPoints.size(); j++)
                            painter.drawPoint(newPoints.at(j));
                    }
                }
                break;
                case 3:
                    //qDebug() << "EditMapWidget::saveSlot Drawing an empty rectangle";
                    painter.setPen(QPen(QColor(_color, _color, _color), _size, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
                    if(points.size() > 1)
                        painter.drawRect(QRect(QPoint(static_cast<int>(points.at(0).x()), static_cast<int>(points.at(0).y())),
                                                QPoint(static_cast<int>(points.at(1).x()), static_cast<int>(points.at(1).y()))));
                break;
                case 4:
                    //qDebug() << "EditMapWidget::saveSlot Drawing a filled rectangle";
                    if(points.size() > 1){
                        QRect rect = QRect(QPoint(static_cast<int>(points.at(0).x()), static_cast<int>(points.at(0).y())),
                                           QPoint(static_cast<int>(points.at(1).x()), static_cast<int>(points.at(1).y())));
                        painter.setPen(QPen(QColor(_color, _color, _color), qMax(rect.width(), rect.height()), Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
                        painter.fillRect(rect, QColor(_color, _color, _color));
                    }
                break;
                default:
                    qDebug() << "EditMapWidget::saveSlot should not be here (default)";
                break;
            }
        } else {
            qDebug() << "EditMapWidget::saveSlot should not be here (items.first < 3)";
        }
    }

    painter.end();

    emit saveEditMap();
}

void EditMapWidget::changeSizeEditSlot(){
    qDebug() << "EditMapWidget::changeSizeEditSlot called" << sizeLineEdit->text();
    int size = sizeLineEdit->text().toInt();

    if(size == 1 || size == 3 || size == 7)
        sizeGroup->button(size)->setChecked(true);

    emit changeSizeEdit(size);
}

void EditMapWidget::changeLineEditSlot(int size){
    qDebug() << "EditMapWidget::changeLineEditSlot called" << size;
    sizeLineEdit->setText(QString::number(size));
}
