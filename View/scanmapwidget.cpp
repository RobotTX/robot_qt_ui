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
#include "View/teleopwidget.h"


ScanMapWidget::ScanMapWidget(QSharedPointer<Robots> _robots, QWidget* parent)
    : QWidget(parent), robots(_robots), mapSize(QSize()), resolution(-1){
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

    /// Title
    QLabel* titleLabel = new QLabel("Scan a new map", this);
    QFont tmpFont = font();
    tmpFont.setPointSize(13);
    setFont(tmpFont);
    titleLabel->setFont(tmpFont);
    titleLabel->setAlignment(Qt::AlignCenter);
    topMenuLayout->addWidget(titleLabel);

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    topMenuLayout->addWidget(spaceWidget);

    /// Button to start to scan
    CustomPushButton* addImageRobotBtn = new CustomPushButton("Start a scan", this);
    addImageRobotBtn->setToolTip("Start to scan with a robot");
    connect(addImageRobotBtn, SIGNAL(clicked()), this, SLOT(addImageRobotSlot()));
    topMenuLayout->addWidget(addImageRobotBtn);


    /// The widget that lists every scanning robot
    listWidget = new MergeMapListWidget(this);
    connect(listWidget, SIGNAL(dirKeyPressed(int)), this, SLOT(dirKeyEventSlot(int)));
    topMenuLayout->addWidget(listWidget);

    menuLayout->addLayout(topMenuLayout);


    /// Teleoperation widget
    QVBoxLayout* teleopLayout = new QVBoxLayout();
    TeleopWidget* teleopWidget = new TeleopWidget(this);
    connect(teleopWidget->getBtnGroup(), SIGNAL(buttonClicked(int)), this, SLOT(teleopCmdSlot(int)));
    teleopLayout->addWidget(teleopWidget);
    menuLayout->addLayout(teleopLayout);


    /// Cancel + Svae button
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
    teleopLayout->setContentsMargins(0, 0, 0, 0);
    topMenuLayout->setContentsMargins(0, 0, 0, 0);
    cancelSaveLayout->setContentsMargins(0, 0, 0, 0);
    menuLayout->setContentsMargins(0, 0, 5, 0);

    topMenuLayout->setAlignment(Qt::AlignTop);
    cancelSaveLayout->setAlignment(Qt::AlignBottom);
    teleopLayout->setAlignment(Qt::AlignBottom);
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
        /// Create a list of already scanning robots so we can't add them twice
        for(int i = 0; i < listWidget->count(); i++)
            list.push_back(static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)))->getRobotName());

        /// Add the available robots to the list + disable the one already scanning
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
        addMapWidget(robotName);
    else {
        QMessageBox msgBox;
        msgBox.setText(robotName + "could not start scanning, please try again");
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

void ScanMapWidget::addMapWidget(QString name){
    ScanMapListItemWidget* listItem = new ScanMapListItemWidget(listWidget->count(), name, scene);

    connect(listItem, SIGNAL(deleteMap(int, QString)), this, SLOT(deleteMapSlot(int, QString)));
    connect(listItem, SIGNAL(playScan(bool, QString)), this, SLOT(playScanSlot(bool, QString)));
    connect(listItem, SIGNAL(robotGoTo(QString, double, double)), this, SLOT(robotGoToSlot(QString, double, double)));
    connect(listItem, SIGNAL(centerOn(QGraphicsItem*)), this, SLOT(centerOnSlot(QGraphicsItem*)));


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
    if(listWidget->count() > 0){

        /// We hide the robotViews
        for(int i = 0; i < listWidget->count(); i++){
            ScanMapListItemWidget* item = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
            item->getPixmapItem()->getRobotView()->hide();
        }

        /// Get an image from the scene
        QImage image = sceneToImage();
        image.save("/home/m-a/Desktop/1.pgm");

        /// Check if the size of the image is bigger than expected, and if so, alert the user that we might lose data if using it
        if(checkImageSize(image.size())){
            image = croppedImageToMapImage(image);
            image.save("/home/m-a/Desktop/2.pgm");


            /// If we got a resolution proceed, else we don't save
            if(resolution != -1){
                qDebug() << "ScanMapWidget::saveSlot final origin in pixel :" << resolution << -image.width()*resolution/2;

                /// Reconvert the new origin from pixel coordinates to the system used by the robot
                Position newOrigin = MainWindow::convertPixelCoordinatesToRobotCoordinates(Position(image.width()/2, image.height()/2), 0, 0, resolution, image.height());
                newOrigin.setX(-newOrigin.getX());
                newOrigin.setY(-newOrigin.getY());

                /// Where to save the new map
                QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("Images (*.pgm)"));

                if(!fileName.isEmpty()){
                    image.save(fileName);
                    emit saveScanMap(resolution, newOrigin, image, fileName);
                    close();
                }
            }
        }
    } else {
        QMessageBox msgBox;
        msgBox.setText("You have no map to save.");
        msgBox.setStandardButtons(QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        msgBox.exec();
    }
}

void ScanMapWidget::closeEvent(QCloseEvent *event){
    qDebug() << "ScanMapWidget::closeEvent";
    QStringList list = getAllScanningRobots();

    qDebug() << "ScanMapWidget::closeEvent" << list.count() << "robot(s) to stop scanning :" << list;
    if(list.count() > 0)
        emit stopScanning(list);
    QWidget::closeEvent(event);
}

QStringList ScanMapWidget::getAllScanningRobots(){
    QStringList list;

    for(int i = 0; i < listWidget->count(); i++){
        ScanMapListItemWidget* item = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->item(i)));
        list.push_back(item->getRobotName());
    }

    return list;
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
        if(item->getRobotName() == robotName)
            item->robotScanning(scan == success);
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

void ScanMapWidget::receivedScanMapSlot(QString robotName, QImage map, double _resolution){
    mapSize = map.size();
    if(_resolution != -1)
        resolution = _resolution;

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

void ScanMapWidget::centerOnSlot(QGraphicsItem* pixmap){
    graphicsView->centerOn(pixmap);
}

QImage ScanMapWidget::sceneToImage(){
    qDebug() << "ScanMapWidget::sceneToImage called";

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
    image.save("/home/m-a/Desktop/0.png");


    /// The image is still in green and red color so we set the pixel to white and black
    for(int i = 0; i < image.width(); i++){
        for(int j = 0; j < image.height(); j++){
            QColor color = image.pixelColor(i, j);
            if(!(color.red() == 205 && color.green() == 205 && color.blue() == 205)){
                if(color.red() == color.green() && color.green() == color.blue())
                    image.setPixelColor(i, j, Qt::white);
                else
                    image.setPixelColor(i, j, Qt::black);
            }
        }
    }

    return image;
}

QImage ScanMapWidget::croppedImageToMapImage(QImage croppedImage){
    qDebug() << "ScanMapWidget::croppedImageToMapImage called";


    /// Create the new image on the desired size
    QImage image(mapSize, QImage::Format_ARGB32);
    /// Fille the image with grey
    image.fill(QColor(205, 205, 205));

    /// Calculate the size difference between the given image and the output image
    int leftDiff = qAbs((mapSize.width() - croppedImage.width())/2);
    int topDiff = qAbs((mapSize.height() - croppedImage.height())/2);

    /// If the input image is bigger on any side, we crop the image
    if(croppedImage.width() > mapSize.width() && croppedImage.height() > mapSize.height()){
        qDebug() << "ScanMapWidget::croppedImageToMapImage width & height are bigger";
        QImage tmpImage = croppedImage.copy(leftDiff, topDiff, mapSize.width(), mapSize.height());

        QPainter painter(&image);
        painter.drawImage(0, 0, tmpImage);
        painter.end();

    } else if(croppedImage.width() > mapSize.width()){
        qDebug() << "ScanMapWidget::croppedImageToMapImage width is bigger";
        QImage tmpImage = croppedImage.copy(leftDiff, 0, mapSize.width(), mapSize.height());

        QPainter painter(&image);
        painter.drawImage(0, 0, tmpImage);
        painter.end();

    } else if(croppedImage.height() > mapSize.height()){
        qDebug() << "ScanMapWidget::croppedImageToMapImage height is bigger";
        QImage tmpImage = croppedImage.copy(0, topDiff, mapSize.width(), mapSize.height());

        QPainter painter(&image);
        painter.drawImage(0, 0, tmpImage);
        painter.end();

    } else {
        qDebug() << "ScanMapWidget::croppedImageToMapImage everything is fine";
        for(int i = 0; i < croppedImage.width(); i++)
            for(int j = 0; j < croppedImage.height(); j++)
                image.setPixelColor(i + leftDiff, j + topDiff, croppedImage.pixelColor(i, j));
    }

    return image;
}

bool ScanMapWidget::checkImageSize(QSize sizeCropped){
    qDebug() << "ScanMapWidget::checkImageSize called original size :" << mapSize << "compared to new size :" << sizeCropped;

    /// If the cropped image is bigger than the ouput one, we display a message to the user
    if(sizeCropped.width() > mapSize.width() || sizeCropped.height() > mapSize.height()){
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
                /// should never be here
                qDebug() << "ScanMapWidget::saveSlot should not be here";
                Q_UNREACHABLE();
            break;
        }
        return false;
    }
    return true;
}

void ScanMapWidget::teleopCmdSlot(int id){
    qDebug() << "ScanMapWidget::teleopCmd" << id;
    if(listWidget->currentItem() != NULL){
        QString robotName = static_cast<ScanMapListItemWidget*>(listWidget->itemWidget(listWidget->currentItem()))->getRobotName();
        qDebug() << "ScanMapWidget::teleopCmd" << robotName;
        emit teleopCmd(robotName, id);
    }
}

void ScanMapWidget::keyPressEvent(QKeyEvent *event){
    dirKeyEventSlot(event->key());
}

void ScanMapWidget::dirKeyEventSlot(int key){
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
}
