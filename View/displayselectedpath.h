#ifndef DISPLAYSELECTEDPATH_H
#define DISPLAYSELECTEDPATH_H

class TopLeftMenu;
class PathWidget;
class QVBoxLayout;
class CustomLabel;
class PathPoint;
class MainWindow;
class CustomScrollArea;

#include "Model/paths.h"
#include <QWidget>
#include <QSharedPointer>
#include <QMap>

/**
 * @brief The DisplaySelectedPath class
 * class that provides a widget to display the information of a given path
 * also allows to edit the path
 */
class DisplaySelectedPath: public QWidget{
    Q_OBJECT

public:
    struct PathInfos {
      QString groupName;
      QString pathName;
      QVector<QSharedPointer<PathPoint>> path;
    };

    DisplaySelectedPath(QWidget* parent, const MainWindow* mainWindow, const QSharedPointer<Paths>& _paths);
    void updatePath(const QString groupName, const QString pathName, const QVector<QSharedPointer<PathPoint> > &path, const QString visiblePath);

protected:
    void keyPressEvent(QKeyEvent* event);

signals:
    void deletePath(QString, QString);
    void editPath(QString, QString);
    void displayPath(QString, QString, bool);

private slots:/**
     * @brief deletePath
     * emits the delete path signal when the minus button is clicked
     */
    void minusBtnSlot(bool checked);
    /**
     * @brief deletePath
     * emits the edit path signal when the edit button is clicked
     */
    void editBtnSlot(bool checked);
    /**
     * @brief deletePath
     * emits the display path signal when the edit button is clicked
     */
    void mapBtnSlot(bool checked);

private:
    QSharedPointer<Paths> paths;
    QVBoxLayout* layout;
    TopLeftMenu* actionButtons;
    CustomLabel* nameLabel;
    PathWidget* pathWidget;
    PathInfos currentPath;
    CustomScrollArea* scrollArea;


};

#endif // DISPLAYSELECTEDPATH_H
