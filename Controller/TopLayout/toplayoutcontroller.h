#ifndef TOPLAYOUTCONTROLLER_H
#define TOPLAYOUTCONTROLLER_H

class MainWindow;

#include <QObject>
#include <QPointer>
#include <QWidget>
#include "View/TopLayout/toplayoutwidget.h"

/**
 * @brief The TopLayoutController class
 * This class's purpose is to control what is going on at the top of the application
 * the buttons and the label
 */
class TopLayoutController : public QObject {

    Q_OBJECT

public:

    TopLayoutController(MainWindow *parent = 0);

    TopLayoutWidget* getTopLayout(void) const { return view; }

    void createTopLayoutView(MainWindow *parent);

    void removeRobotWithoutHome(const QString name);
    void addRobotWithoutHome(const QString name);

    QString getLabelText(void) const { return view->getLabelText(); }

    QString getRobotsString(void) const;

private slots:
    void setLabelDelay(const QString msgType, const QString label, int delay);
    void setLabel(const QString msgType, const QString label);
    void enableLayout(const bool enable) { view->setEnable(enable); }

private:

    TopLayoutWidget* view;

    QVector<QString> robotsWithoutHome;

};

#endif /// TOPLAYOUTCONTROLLER_H
