#ifndef PATHCREATIONWIDGET_H
#define PATHCREATIONWIDGET_H

class MainWindow;
class Points;
class Robot;
class TopLeftMenu;
class QMenu;
class PathPointList;
class QListWidgetItem;
class PathPointCreationWidget;
class PathPoint;

#include <QWidget>
#include <QSharedPointer>
#include <QVBoxLayout>

#define WIDGET_HEIGHT 100

/**
 * @brief The PathCreationWidget class
 * The widget which display the left menu of the creation of a path
 * Display a few buttons and the pathpointlist
 */
class PathCreationWidget: public QWidget{
    Q_OBJECT

enum CheckState { NO_STATE, CREATE, EDIT };

public:
    struct PointInfo {
        QString name;
        float posX;
        float posY;
    };

    PathCreationWidget(MainWindow* parent, const QSharedPointer<Points>& points);
    void updatePath(const QVector<QSharedPointer<PathPoint> >& _currentPath);
    void updatePointsList(void);
    void deleteItem(QListWidgetItem* item);
    void editPathPoint(const QString name, const double x, const double y);
    PathPointList* getPathPointList(void) const { return pathPointsList; }

protected:
    void showEvent(QShowEvent* event);

signals:
    /// emitted when the waiting times of points have not been set properly so that a message is displayed to the user
    void setMessage(QString, QString );
    /// emitted when a new path point is added to the list
    void addPathPoint(QString, double, double);
    /// emitted when a path point is deleted
    void deletePathPoint(int);
    /// emitted when the order of the path has changed
    void orderPathPointChanged(int, int);
    /// emitted when the widget is reset
    void resetPath();
    /// emitted when a waiting time is changed
    void actionChanged(int, int, QString);
    /// emitted when a path point is edited
    void editPathPoint(int, QString, double, double);
    void editTmpPathPoint(int, QString, double, double);
    void saveEditPathPoint();
    void cancelEditPathPoint();
    void savePath();

private slots:
    void resetWidget(void);
    void addPathPointByMenuSlot(void);
    void deletePathPointSlot(void);
    void editPathPointSlot(void);
    void itemClicked(QListWidgetItem* item);
    void itemMovedSlot(const QModelIndex& , int start, int , const QModelIndex& , int row);
    void savePathClicked(void);
    void clicked(void);
    void pointClicked(QAction *action);
    void addPathPointSlot(QString name, double x, double y);
    void saveEditSlot(PathPointCreationWidget* pathPointCreationWidget);
    void cancelEditSlot(PathPointCreationWidget* pathPointCreationWidget);
    void actionChangedSlot(int id, int action, QString waitTime);

private:
    QSharedPointer<Points> points;
    QVBoxLayout* layout;
    TopLeftMenu* actionButtons;
    QMenu* pointsMenu;
    PathPointList* pathPointsList;
    CheckState state;
};

#endif // PATHCREATIONWIDGET_H

