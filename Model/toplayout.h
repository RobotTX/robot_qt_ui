#ifndef TOPLAYOUT_H
#define TOPLAYOUT_H

#include <QVector>
#include <QObject>
#include <QDebug>

class TopLayout: public QObject
{
    Q_OBJECT

public:
    TopLayout();

    void addRobotWithoutHome(const QString name) { robotsWithoutHome.push_back(name); }

    void removeRobotWithoutHome(const QString name);

    QString getRobotsString(void) const;

private:
    QVector<QString> robotsWithoutHome;
};

#endif /// TOPLAYOUT_H
