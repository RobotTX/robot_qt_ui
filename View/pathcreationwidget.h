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
class QLineEdit;
class QLabel;
class CustomPushButton;

#include <QWidget>
#include <QSharedPointer>
#include <QVBoxLayout>
#include "Model/paths.h"

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

    PathCreationWidget(QWidget *parent, const QSharedPointer<Points>& points, const QSharedPointer<Paths>& _paths, const bool associatedToRobot);
    void updatePath(const QVector<QSharedPointer<PathPoint> >& _currentPath);
    void updatePointsList(void);
    void deleteItem(QListWidgetItem* item);
    void editPathPoint(const QString name, const double x, const double y);
    PathPointList* getPathPointList(void) const { return pathPointsList; }
    void setCurrentGroupName(const QString name) { currentGroupName = name; }
    QString getCurrentGroupName(void) const { return currentGroupName; }
    CustomPushButton* getCancelButton(void) const { return cancelBtn; }
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    CustomPushButton* getSaveButton(void) const { return saveBtn; }

protected:
    void showEvent(QShowEvent* event);
    void keyPressEvent(QKeyEvent* event);

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
    void codeEditPath(int codeError);

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
    void checkPathName(const QString name);

private:
    QSharedPointer<Points> points;
    QSharedPointer<Paths> paths;
    QVBoxLayout* layout;
    TopLeftMenu* actionButtons;
    QMenu* pointsMenu;
    PathPointList* pathPointsList;
    CheckState state;
    QLabel* nameLabel;
    QLineEdit* nameEdit;
    QString currentGroupName;
    CustomPushButton* cleanBtn;
    CustomPushButton* cancelBtn;
    CustomPushButton* saveBtn;
};

#endif // PATHCREATIONWIDGET_H

