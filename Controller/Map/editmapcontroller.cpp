#include "editmapcontroller.h"
#include "View/editmappainteditem.h"
#include <QDebug>
#include <QQmlApplicationEngine>
#include <QKeySequence>
#include <QQuickWindow>

EditMapController::EditMapController(QQmlApplicationEngine* engine, QObject *applicationWindow, QObject* parent) : QObject(parent)
{
    QObject *editMapWindow = applicationWindow->findChild<QObject*>("editMapWindow");

    if(editMapWindow){

        QQmlComponent component(engine, QUrl("qrc:/View/Map/EditMapPaintedItem.qml"));
        paintedItem = qobject_cast<EditMapPaintedItem*>(component.create());
        QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

        // that is where we actually tell the paintemItem to paint itself
        QQuickItem* mapView = applicationWindow->findChild<QQuickItem*> ("editMapImage");
        paintedItem->setParentItem(mapView);
        paintedItem->setParent(engine);

        /// to add new items to the scene
        connect(editMapWindow, SIGNAL(clicked(int, QColor, int, int, int, bool)), this, SLOT(add_item(int, QColor, int, int, int, bool)));
        connect(editMapWindow, SIGNAL(resetMap()), this, SLOT(clearMapItems()));
        connect(editMapWindow, SIGNAL(undo()), paintedItem, SLOT(undo()));
        connect(editMapWindow, SIGNAL(redo()), paintedItem, SLOT(redo()));
        connect(editMapWindow, SIGNAL(saveImage(QString)), parent, SLOT(saveEditedImage(QString)));

    } else {
        Q_UNREACHABLE();
        qDebug() << "could not find the mouse area within the edit map widget";
    }
}

/// adds a new item to draw
void EditMapController::add_item(int shape, QColor color, int thickness, int x, int y, bool _update){
    qDebug() << "painting " << shape << color << thickness << x << y ;
    paintedItem->addItem(static_cast<EditMapPaintedItem::SHAPE> (shape), color, thickness, x, y, _update);
}


void EditMapController::clearMapItems(){
    qDebug() << "EditMapController::clearMapItems called";
    paintedItem->clearMapItems();
}

