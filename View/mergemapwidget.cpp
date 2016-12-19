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
#include "View/mergemaplistwidget.h"
#include <fstream>
#include "Controller/mainwindow.h"
#include "View/mergemapgraphicsitem.h"
#include <QMenu>

MergeMapWidget::MergeMapWidget(QSharedPointer<Robots> _robots, QWidget *parent) : QWidget(parent), robots(_robots){
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

    CustomPushButton* resetBtn = new CustomPushButton(QIcon(":/icons/empty.png"), "Reset", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    resetBtn->setToolTip("Remove all maps");
    topMenuLayout->addWidget(resetBtn);
    connect(resetBtn, SIGNAL(clicked()), this, SLOT(resetSlot()));

    CustomPushButton* addImageFileBtn = new CustomPushButton("Add map from file", this);
    addImageFileBtn->setToolTip("Add a map from a pgm file");
    connect(addImageFileBtn, SIGNAL(clicked()), this, SLOT(addImageFileSlot()));
    topMenuLayout->addWidget(addImageFileBtn);

    CustomPushButton* addImageRobotBtn = new CustomPushButton("Add map from robot", this);
    addImageRobotBtn->setToolTip("Add a robot is currently using");
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

void MergeMapWidget::initializeMap(){
    scene = new QGraphicsScene(this);

    /// Set the background of the scene as the same grey used in the map
    scene->setBackgroundBrush(QBrush(QColor(205, 205, 205)));

    graphicsView = new CustomQGraphicsView(scene, this);
    graphicsView->setCatchKeyEvent(true);
    graphicsView->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    connect(graphicsView, SIGNAL(dirKeyPressed(int)), this, SLOT(dirKeyEventSlot(int)));
}

void MergeMapWidget::resetSlot(){
    qDebug() << "MergeMapWidget::resetSlot called";
    while(listWidget->count() != 0)
        deleteMapSlot(0);
}

void MergeMapWidget::addImageFileSlot(){
    qDebug() << "MergeMapWidget::addImageFileSlot called";

    /// Get the file name of the map we want to use
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Image"), "", tr("Image Files (*.pgm)"));

    if(!fileName.isEmpty())
        addMap(fileName, false);
}

void MergeMapWidget::addImageRobotSlot(){
    qDebug() << "MergeMapWidget::addImageRobotSlot called";

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

void MergeMapWidget::receivedMapToMergeSlot(QString robotName, QImage image, double _resolution, double _originX, double _originY){
    addMap(robotName, true, image, _resolution, _originX, _originY);
}

void MergeMapWidget::addMap(QString fileName, bool fromRobot, QImage image, double _resolution, double _originX, double _originY){
    MergeMapListItemWidget* listItem;

    /// If we got the map from a robot, we already have an image, a resolution and an origin
    if(fromRobot){
        listItem = new MergeMapListItemWidget(listWidget->count(), fileName, scene, fromRobot, image, _resolution, _originX, _originY);
        if(listWidget->count() == 0)
            originalSize = image.size();
    } else {
        listItem = new MergeMapListItemWidget(listWidget->count(), fileName, scene, fromRobot);
        if(listWidget->count() == 0)
            originalSize = QImage(fileName,"PGM").size();
    }

    connect(listItem, SIGNAL(deleteMap(int)), this, SLOT(deleteMapSlot(int)));
    connect(listItem, SIGNAL(pixmapClicked(int)), this, SLOT(selectPixmap(int)));

    /// We add the path point widget to the list
    QListWidgetItem* listWidgetItem = new QListWidgetItem(listWidget);
    listWidgetItem->setSizeHint(QSize(listWidgetItem->sizeHint().width(), MERGE_WIDGET_HEIGHT));
    listWidgetItem->setBackgroundColor(QColor(255, 255, 255, 10));

    listWidget->addItem(listWidgetItem);
    listWidget->setItemWidget(listWidgetItem, listItem);
}

void MergeMapWidget::robotMenuSlot(QAction* action){
    qDebug() << "MergeMapWidget::robotMenuSlot called" << action->text();
    emit getMapForMerging(action->text());
}

void MergeMapWidget::cancelSlot(){
    qDebug() << "MergeMapWidget::cancelSlot Closing the edit widget";
    close();
}

void MergeMapWidget::saveSlot(){
    qDebug() << "\nMergeMapWidget::saveSlot called";
    /// We want the origin to be reset when saving
    originInPixel = QPoint(-1, -1);
    croppedOriginInPixel = QPoint(-1, -1);
    if(listWidget->count() > 1){

        /// Get an image from the scene
        QImage image = sceneToImage();
        //image.save("/home/m-a/Desktop/1.pgm");

        /// Check if the size of the image is bigger than expected, and if so, alert the user that we might lose data if using it
        if(checkImageSize(image.size())){
            image = croppedImageToMapImage(image);
            //image.save("/home/m-a/Desktop/2.pgm");

            getResolution();

            /// If we got a resolution proceed, else we don't save
            if(resolution != -1){
                qDebug() << "MergeMapWidget::saveSlot final origin in pixel :" << originInPixel << resolution << -image.width()*resolution/2;

                /// Reconvert the new origin from pixel coordinates to the system used by the robot
                Position pos1 = MainWindow::convertPixelCoordinatesToRobotCoordinates(Position(originInPixel.x(), originInPixel.y()), 0, 0, resolution, image.height(), 0);
                pos1.setX(-pos1.getX());
                pos1.setY(-pos1.getY());

                /// Where to save the new map
                QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("Images (*.pgm)"));

                if(!fileName.isEmpty()){
                    image.save(fileName);
                    emit saveMergeMap(resolution, pos1, image, fileName);
                    close();
                }
            }
        }
    } else {
        qDebug() << "MergeMapWidget::saveSlot You need to merge at least 2 maps";

        QMessageBox msgBox;
        msgBox.setText("You need at least 2 maps to merge.");
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

QImage MergeMapWidget::sceneToImage(){
    qDebug() << "MergeMapWidget::sceneToImage called";

    /// Center the scene on the images
    scene->clearSelection();
    scene->setSceneRect(scene->itemsBoundingRect());

    /// Create an image with the size the images are taking in the scene
    QImage image(scene->sceneRect().size().toSize(), QImage::Format_ARGB32);
    /// Fill the image with grey
    image.fill(QColor(205, 205, 205));

    /// We use a painter to copy the scene into the image
    QPainter painter(&image);
    scene->render(&painter);
    //image.save("/home/m-a/Desktop/0.png");


    /// The image is still in green and red color so we set the pixel to white and black
    for(int i = 0; i < image.width(); i++){
        for(int j = 0; j < image.height(); j++){
            QColor color = image.pixelColor(i, j);
            if(!(color.red() == 205 && color.green() == 205 && color.blue() == 205)){
                /// If we find a pixel with more blue than the rest, it is our origin pixel
                if(color.red() == color.green() && color.blue() > color.red()){
                    image.setPixelColor(i, j, Qt::white);
                    croppedOriginInPixel = QPoint(i, j);
                    qDebug() << "Got an origin :" << croppedOriginInPixel;
                } else if(color.red() == color.green() && color.green() == color.blue())
                    image.setPixelColor(i, j, Qt::white);
                else
                    image.setPixelColor(i, j, Qt::black);
            }
        }
    }

    return image;
}

QImage MergeMapWidget::croppedImageToMapImage(QImage croppedImage){
    qDebug() << "MergeMapWidget::croppedImageToMapImage called";

    /// If we don't have an origin, we set the origin in the middle of the map
    if(croppedOriginInPixel.x() == -1){
        croppedOriginInPixel.setX(croppedImage.width()/2);
        croppedOriginInPixel.setY(croppedImage.height()/2);
    }

    /// Create the new image on the desired size
    QImage image(originalSize, QImage::Format_ARGB32);
    /// Fille the image with grey
    image.fill(QColor(205, 205, 205));

    /// Calculate the size difference between the given image and the output image
    int leftDiff = qAbs((originalSize.width() - croppedImage.width())/2);
    int topDiff = qAbs((originalSize.height() - croppedImage.height())/2);

    int leftSign = 1;
    int topSign = 1;

    /// If the input image is bigger on any side, we crop the image
    if(croppedImage.width() > originalSize.width() && croppedImage.height() > originalSize.height()){
        qDebug() << "MergeMapWidget::croppedImageToMapImage width & height are bigger";
        QImage tmpImage = croppedImage.copy(leftDiff, topDiff, originalSize.width(), originalSize.height());

        QPainter painter(&image);
        painter.drawImage(0, 0, tmpImage);
        painter.end();

        leftSign = -1;
        topSign = -1;

    } else if(croppedImage.width() > originalSize.width()){
        qDebug() << "MergeMapWidget::croppedImageToMapImage width is bigger";
        QImage tmpImage = croppedImage.copy(leftDiff, 0, originalSize.width(), originalSize.height());

        QPainter painter(&image);
        painter.drawImage(0, 0, tmpImage);
        painter.end();

        leftSign = -1;

    } else if(croppedImage.height() > originalSize.height()){
        qDebug() << "MergeMapWidget::croppedImageToMapImage height is bigger";
        QImage tmpImage = croppedImage.copy(0, topDiff, originalSize.width(), originalSize.height());

        QPainter painter(&image);
        painter.drawImage(0, 0, tmpImage);
        painter.end();

        topSign = -1;

    } else {
        qDebug() << "MergeMapWidget::croppedImageToMapImage everything is fine";
        for(int i = 0; i < croppedImage.width(); i++)
            for(int j = 0; j < croppedImage.height(); j++)
                image.setPixelColor(i + leftDiff, j + topDiff, croppedImage.pixelColor(i, j));
    }

    /// Calculate the position of the origin in the output image
    originInPixel.setX(croppedOriginInPixel.x() + leftDiff * leftSign);
    originInPixel.setY(croppedOriginInPixel.y() + topDiff * topSign);

    /// If the origin is out of the map (because we cropped it), we set it in the middle of the map
    if(originInPixel.x() < 0 || originInPixel.y() < 0 || originInPixel.x() >= image.width() || originInPixel.y() >= image.height()){
        originInPixel.setX(image.width()/2);
        originInPixel.setY(image.height()/2);
    }

    return image;
}

bool MergeMapWidget::checkImageSize(QSize sizeCropped){
    qDebug() << "MergeMapWidget::checkImageSize called original size :" << originalSize << "compared to new size :" << sizeCropped;

    /// If the cropped image is bigger than the ouput one, we display a message to the user
    if(sizeCropped.width() > originalSize.width() || sizeCropped.height() > originalSize.height()){
        QMessageBox msgBox;
        msgBox.setText("The new image is bigger than the input one, some data may be lost.");
        msgBox.setInformativeText("Do you wish to proceed ?");
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        int ret = msgBox.exec();

        switch(ret){
            case QMessageBox::Cancel :
                return false;
            break;
            case QMessageBox::Ok :
                return true;
            break;
            default:
                Q_UNREACHABLE();
                /// should never be here
                qDebug() << "MergeMapWidget::saveSlot should not be here";
            break;
        }
        return false;
    }
    return true;
}

void MergeMapWidget::getResolution(){
    qDebug() << "MergeMapWidget::getResolution called";

    resolution = -1;

    /// We try to get the resolution from the widgets containing the maps
    for(int i = 0; i < listWidget->count(); i++){
        MergeMapListItemWidget* item = static_cast<MergeMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        if(item->getResolution() != -1){
            resolution = item->getResolution();
            break;
        }
    }

    /// If we failed getting it from the maps, we try to get it from the currentMap used in the mainWindow
    if(resolution == -1){
        qDebug() << "MergeMapWidget::getResolution Trying to get a config from the current map" << resolution;
        std::ifstream currentMapConfig((QDir::currentPath() + QDir::separator() + "currentMap.txt").toStdString(), std::ios::in);

        if(currentMapConfig.is_open()){
            std::string osef;
            currentMapConfig >> osef >> osef >> osef >> osef >> osef >> osef >> osef >> osef >> resolution;
            currentMapConfig.close();

            qDebug() << "MergeMapWidget::getMapConfig Got a resolution and origin from the current map" << resolution;
        } else {
            qDebug() << "MergeMapWidget::getMapConfig no config file found for the current map";
            QMessageBox msgBox;
            msgBox.setText("A config file for the maps you selected or for the current map could not be found, please select a map with a valid config file or import one from a robot.");
            msgBox.setStandardButtons(QMessageBox::Cancel);
            msgBox.setDefaultButton(QMessageBox::Cancel);
            msgBox.exec();
        }
    }
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

void MergeMapWidget::dirKeyEventSlot(int key){
    if(listWidget->currentItem() != NULL){
        MergeMapListItemWidget* widget = static_cast<MergeMapListItemWidget*>(listWidget->itemWidget(listWidget->currentItem()));
        switch (key) {
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
        }
    }
}

void MergeMapWidget::selectPixmap(int id){
    qDebug() << "MergeMapWidget::selectPixmap" << id;
    listWidget->setCurrentRow(id);
}
