#include <QApplication>
#include "Controller/mainwindow.h"
#include <QStyle>
#include <QDesktopWidget>
#include <QSettings>
#include <QDir>

int main(int argc, char *argv[]){

    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontShowIconsInMenus, false);

    QSettings settings("GTDollar", "gobot-software");
    settings.setPath(QSettings::IniFormat, QSettings::UserScope, QDir::currentPath());

    MainWindow w(&settings);
    w.setMinimumSize(1000, 800);
    /// to center the application on the screen
    w.setGeometry(
        QStyle::alignedRect(
            Qt::LeftToRight,
            Qt::AlignCenter,
            w.size(),
            qApp->desktop()->availableGeometry()
        )
    );
    w.show();

    return app.exec();
}
