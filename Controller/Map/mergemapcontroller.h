#ifndef MERGEMAPCONTROLLER_H
#define MERGEMAPCONTROLLER_H

#include <QObject>
class QQmlApplicationEngine;

class MergeMapController : public QObject {

    Q_OBJECT

public:
    MergeMapController(QObject *applicationWindow, QObject* parent);

private slots:
    void importMap(QString file);
};

#endif /// MERGEMAPCONTROLLER_H
