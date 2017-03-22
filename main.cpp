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
    engine.load(QUrl(QLatin1String("qrc:/main.qml")));

    MainController controller(&engine);

    return app.exec();
}
