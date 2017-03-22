#include "editmapcontroller.h"
#include <QDebug>
#include "View/editmappainteditem.h"
#include <QQmlApplicationEngine>

EditMapController::EditMapController(QQmlApplicationEngine* engine, QObject *applicationWindow, QObject* parent) : QObject(parent), _engine(engine)
{
    QObject *editMapWindow = applicationWindow->findChild<QObject*>("editMapWindow");

    if(editMapWindow){
        connect(editMapWindow, SIGNAL(clicked(int, QColor, int, int, int)), this, SLOT(add_item(int, QColor, int, int, int)));
    } else
        qDebug() << "could not find the mouse area within the edit map widget";
}

void EditMapController::add_item(int shape, QColor color, int thickness, int x, int y){
    qDebug() << "painting " << shape << color << thickness << x << y ;
    if(shape == EditMapPaintedItem::SHAPE::POINT){
        QObject *applicationWindow = _engine->rootObjects().at(0);
        QQmlComponent component(_engine, QUrl("qrc:/View/Map/EditMapPaintedItem.qml"));
        QQuickItem *object = qobject_cast<QQuickItem*>(component.create());
        QQmlEngine::setObjectOwnership(object, QQmlEngine::CppOwnership);
        QQuickItem* mapView = applicationWindow->findChild<QQuickItem*> ("editMapImage");

        object->setParentItem(mapView);
        object->setParent(_engine);

        if(object){
            object->setProperty("shape", shape);
            object->setProperty("color", color);
            object->setProperty("thickness", thickness);
            object->setProperty("x", x);
            object->setProperty("y", y);
            QVariant returnedValue;
            QVariant msg = "Hello from C++";
            QMetaObject::invokeMethod(object, "test",
                    Q_RETURN_ARG(QVariant, returnedValue),
                    Q_ARG(QVariant, msg));
        }
        else
            qDebug() << "no painted item";
    }
}

/*
applicationWindow->show();
QQuickItem *root = window->contentItem();

QQmlComponent component(&engine, QUrl("qrc:/View/Map/EditMapPaintedItem.qml"));
QQuickItem *object = qobject_cast<QQuickItem*>(component.create());
QQuickItem *object2 = qobject_cast<QQuickItem*>(component.create());
QQuickItem *object3 = qobject_cast<QQuickItem*>(component.create());

QQuickItem* mapView = window->findChild<QQuickItem*> ("mapImage");

if(mapView){
    qDebug() << "found mapImage";
    object->setParentItem(mapView);
    object->setParent(&engine);
    object2->setParentItem(mapView);
    object2->setParent(&engine);
    object3->setParentItem(mapView);
    object3->setParent(mapView);
}

QObject *item = object->findChild<QObject*>("paintedItem");
if(item){
    item->setProperty("shape", 1);
    item->setProperty("color", "red");
    item->setProperty("width", window->width());
    item->setProperty("height", window->height());

}
else
    qDebug() << "no painted item";

QObject *item2 = object2->findChild<QObject*>("paintedItem");
if(item2){
    item2->setProperty("shape", 0);
    item2->setProperty("color", "blue");
    item2->setProperty("thickness", 5);
    item2->setProperty("width", window->width());
    item2->setProperty("height", window->height());
}
else
    qDebug() << "no painted item 2";

QObject *item3 = object3->findChild<QObject*>("paintedItem");
if(item3){
    item3->setProperty("shape", 2);
    item3->setProperty("color", "green");
    item3->setProperty("width", window->width());
    item3->setProperty("height", window->height());
}
else
    qDebug() << "no painted item 3";

QQmlEngine::setObjectOwnership(object, QQmlEngine::CppOwnership);
QQmlEngine::setObjectOwnership(object2, QQmlEngine::CppOwnership);
QQmlEngine::setObjectOwnership(object3, QQmlEngine::CppOwnership);

MainController controller(&engine);

//item->setProperty("opacity", 0);

*/
