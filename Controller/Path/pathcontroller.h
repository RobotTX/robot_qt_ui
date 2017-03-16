#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

class Paths;
class MainController;

#include <QObject>

class PathController : public QObject {
    Q_OBJECT
public:
    PathController(QObject *applicationWindow, MainController* parent);

private:
    Paths* paths;
};

#endif // PATHCONTROLLER_H
