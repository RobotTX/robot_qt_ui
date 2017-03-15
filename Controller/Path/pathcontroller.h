#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

class Paths;
class MainController;

#include <QObject>

class PathController : public QObject {
    Q_OBJECT
public:
    PathController(QObject *applicationWindow, MainController* parent);

    void serializePaths(const QString fileName);
    void deserializePaths(const QString fileName);

private:
    Paths* paths;
};

#endif // PATHCONTROLLER_H
