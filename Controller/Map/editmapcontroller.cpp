#include "editmapcontroller.h"
#include "View/EditMap/editmappainteditem.h"
#include <QDebug>
#include <QQmlApplicationEngine>
#include <QKeySequence>
#include <QQuickWindow>

EditMapController::EditMapController(QQmlApplicationEngine* engine, QObject *applicationWindow, QObject* parent) : QObject(parent)
{
    QObject *editMapWindow = applicationWindow->findChild<QObject*>("editMapWindow");

    if(editMapWindow){
        QQmlComponent component(engine, QUrl("qrc:/View/EditMap/EditMapPaintedItem.qml"));
        paintedItem = qobject_cast<EditMapPaintedItem*>(component.create());
        QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

        /// that is where we actually tell the paintemItem to paint itself
        QQuickItem* mapView = applicationWindow->findChild<QQuickItem*> ("editMapImage");
        paintedItem->setParentItem(mapView);
        paintedItem->setParent(engine);

        /// to add new items to the scene
        connect(editMapWindow, SIGNAL(clicked(int, QColor, int, int, int, bool)), this, SLOT(add_item(int, QColor, int, int, int, bool)));
        /// to clear the map of all the items
        connect(editMapWindow, SIGNAL(resetMap()), this, SLOT(clearMapItems()));
        /// to undo the last action
        connect(editMapWindow, SIGNAL(undo()), paintedItem, SLOT(undo()));
        /// to redo the last action
        connect(editMapWindow, SIGNAL(redo()), paintedItem, SLOT(redo()));
        /// to save the result
        connect(editMapWindow, SIGNAL(saveImage(QString, int)), parent, SLOT(saveEditedImage(QString, int)));

    } else {
        /// NOTE prob can remove that when testing phase over
        Q_UNREACHABLE();
//        qDebug() << "could not find the mouse area within the edit map widget";
    }
}

/// adds a new item to draw
void EditMapController::add_item(int shape, QColor color, int thickness, int x, int y, bool _update){
    paintedItem->addItem(static_cast<EditMapPaintedItem::SHAPE> (shape), color, thickness, x, y, _update);
}

/// remove all the items and clear the undo vector as well
void EditMapController::clearMapItems(){
    paintedItem->clearMapItems();
}

