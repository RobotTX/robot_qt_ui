#include <QApplication>
#include "Controller/mainwindow.h"

int main(int argc, char *argv[]){

    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontShowIconsInMenus, false);

    MainWindow w;
    w.setMinimumSize(800, 600);
    w.show();

    return app.exec();
}
