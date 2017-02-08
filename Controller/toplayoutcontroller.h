#ifndef TOPLAYOUTCONTROLLER_H
#define TOPLAYOUTCONTROLLER_H

#include <QObject>
#include <QPointer>
#include <QWidget>
#include "View/toplayoutwidget.h"

class TopLayout;

class TopLayoutController: public QObject {
    Q_OBJECT

public:

    TopLayoutController(QWidget* parent = 0);

    TopLayoutWidget* getTopLayout(void) const { return view; }

    void createTopLayoutView(QWidget *parent);

    void removeRobotWithoutHome(const QString name);
    void addRobotWithoutHome(const QString name);

    void setLabelDelay(const QString msgType, const QString label, int delay);

    QString getLabelText(void) const { return view->getLabelText(); }

    void enableLayout(const bool enable) { view->setEnable(enable); }

public slots:
    void setLabel(const QString msgType, const QString label);

private:
    QPointer<TopLayout> topLayout;
    TopLayoutWidget* view;

};

#endif /// TOPLAYOUTCONTROLLER_H
