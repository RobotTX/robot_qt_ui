#ifndef MAINMENUCONTROLLER_H
#define MAINMENUCONTROLLER_H

#include <QObject>
#include <QVariant>

class MainMenuController : public QObject {
    Q_OBJECT
public:
    MainMenuController(QObject *applicationWindow, QObject* parent = Q_NULLPTR);

private slots:
    void menuClicked(QString txt, bool checked);
    void closeMenuClicked(QString txt);

signals:
    void showMenu(QVariant);
    void closeMenu(QVariant);

};

#endif // MAINMENUCONTROLLER_H
