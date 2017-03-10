#ifndef POINTCONTROLLER_H
#define POINTCONTROLLER_H

class Points;
class MainController;

#include <QObject>
#include <QVariant>

class PointController : public QObject {
    Q_OBJECT
public:
    PointController(QObject *applicationWindow, QString mapFile, MainController *parent);

    void checkErrorPoint(const QImage &mapImage, const QString name, const double x, const double y);

private:
    void loadPoints(const QString fileName);
    bool checkPointName(const QString name);

private slots:
    void checkGroup(QString name);

signals:
    void enablePointSaveQml(QVariant enable);
    void enableGroupSaveQml(QVariant enable);

private:
    Points* points;
};

#endif // POINTCONTROLLER_H
