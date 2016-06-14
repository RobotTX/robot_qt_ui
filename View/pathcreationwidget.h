#ifndef PATHCREATIONWIDGET_H
#define PATHCREATIONWIDGET_H

class PathPointList;
class QMainWindow;
class QVBoxLayout;
class QListWidgetItem;
class Robot;

#include "Model/points.h"
#include <QWidget>
#include <QPushButton>

#define WIDGET_HEIGHT 100

/**
 * @brief The PathCreationWidget class
 * The widget which display the left menu of the creation of a path
 * Display a few buttons and the pathpointlist
 */
class PathCreationWidget: public QWidget{
    Q_OBJECT

enum CheckState { NO_STATE, SUPPR, EDIT };

public:
    PathCreationWidget(QMainWindow* parent, const Points& point);
    ~PathCreationWidget();
    void setSelectedRobot(Robot* const _selectedRobot){ selectedRobot = _selectedRobot; }
    void resetWidget(void);
    void supprItem(QListWidgetItem* item);
    void editItem(QListWidgetItem* item);
    /**
     * @brief addPathPoint
     * add a path point to the list from a given point
     */
    void addPathPoint(Point* const point);
    void hideEvent(QHideEvent *event);

private slots:
    /**
     * @brief itemClicked
     * @param item
     * Manage when the user click on the path list
     */
    void itemClicked(QListWidgetItem* item);

    void supprPathPoint(void);
    void editPathPoint(void);
    void savePath(void);
    void pointSelected(const int id, const QString name);

    /**
     * @brief addPathPoint
     * add a path point to the list
     */
    void addPathPoint(void);

    /**
     * @brief updatePointPainter
     * Function which emit the updatePathPointToPainter signal to update the painter
     */
    void updatePointPainter(void);
    void itemMovedSlot(const int from, const int to);

signals:
    void pathSaved();
    void updatePathPointToPainter(QVector<Point>* pointVector);
    void hidePathCreationWidget();
    void editTmpPathPoint(int id, Point* point, int nbWidget);


private:
    QVBoxLayout* layout;
    int idPoint;
    PathPointList* pathPointsList;
    Points points;
    Robot* selectedRobot;
    CheckState state;
    QPushButton* supprBtn;
    QPushButton* editBtn;
    QListWidgetItem* previousItem;
    QVector<Point> pointList;
};

#endif // PATHCREATIONWIDGET_H
