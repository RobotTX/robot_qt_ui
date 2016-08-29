#ifndef DISPLAYSELECTEDPATH_H
#define DISPLAYSELECTEDPATH_H

class TopLeftMenu;
class PathWidget;
class QVBoxLayout;
class QLabel;
class PathPoint;
class MainWindow;
class CustomScrollArea;

#include "Model/paths.h"
#include <QWidget>
#include <QSharedPointer>
#include <QMap>

class DisplaySelectedPath: public QWidget{
    Q_OBJECT

public:
    struct PathInfos {
      QString groupName;
      QString pathName;
      QVector<QSharedPointer<PathPoint>> path;
    };

    DisplaySelectedPath(QWidget* parent, MainWindow* mainWindow, const QSharedPointer<Paths>& _paths);
    void updatePath(QString groupName, QString pathName, QVector<QSharedPointer<PathPoint>> path);

signals:
    void deletePath(QString, QString);
    void editPath(QString, QString);
    void displayPath(QString, QString, bool);

private slots:
    void minusBtnSlot(bool checked);
    void editBtnSlot(bool checked);
    void mapBtnSlot(bool checked);

private:
    QSharedPointer<Paths> paths;
    QVBoxLayout* layout;
    TopLeftMenu* actionButtons;
    QLabel* nameLabel;
    PathWidget* pathWidget;
    PathInfos currentPath;
    CustomScrollArea* scrollArea;


};

#endif // DISPLAYSELECTEDPATH_H
