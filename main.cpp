#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "Controller/maincontroller.h"
#include "View/editmappainteditem.h"
#include <QtQuick/QQuickView>

int main(int argc, char *argv[]) {
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication app(argc, argv);

    qmlRegisterType<EditMapPaintedItem>("EditMapItems", 1, 0, "EditMapPaintedItem");
/*


    QQuickView view;
    view.setResizeMode(QQuickView::SizeRootObjectToView);
    view.setSource(QUrl("qrc:/View/Map/EditMapPaintedItem.qml"));
    view.show();
*/


    QQmlApplicationEngine engine;
    engine.load(QUrl("qrc:/main.qml"));

    QQuickWindow *window = qobject_cast<QQuickWindow*>(engine.rootObjects().at(0));
    if (!window) {
        qFatal("Error: Your root item has to be a window.");
        return -1;
    }
    window->show();
    QQuickItem *root = window->contentItem();

    QQmlComponent component(&engine, QUrl("qrc:/View/Map/EditMapPaintedItem.qml"));
    QQuickItem *object = qobject_cast<QQuickItem*>(component.create());
    QQuickItem *object2 = qobject_cast<QQuickItem*>(component.create());
    QQuickItem *object3 = qobject_cast<QQuickItem*>(component.create());
/*
    QObject *item = object->findChild<QObject*>("paintedItem");
    if(item){
        item->setProperty("shape", 1);
        item->setProperty("color", "red");
    }
    else
        qDebug() << "no painted item";

    QObject *item2 = object2->findChild<QObject*>("paintedItem");
    if(item2){
        item2->setProperty("shape", 0);
        item2->setProperty("color", "blue");
        item2->setProperty("thickness", 5);
    }
    else
        qDebug() << "no painted item 2";

    QObject *item3 = object3->findChild<QObject*>("paintedItem");
    if(item3){
        item3->setProperty("shape", 2);
        item3->setProperty("color", "green");
    }
    else
        qDebug() << "no painted item 3";

    QQmlEngine::setObjectOwnership(object, QQmlEngine::CppOwnership);
    QQmlEngine::setObjectOwnership(object2, QQmlEngine::CppOwnership);
    QQmlEngine::setObjectOwnership(object3, QQmlEngine::CppOwnership);

    object->setParentItem(root);
    object->setParent(&engine);
    object2->setParentItem(root);
    object2->setParent(&engine);
    object3->setParentItem(root);
    object3->setParent(&engine);
*/
    MainController controller(&engine);

    return app.exec();
}
