#ifndef POINTLIST_H
#define POINTLIST_H

class Points;

#include <QVBoxLayout>
#include <QListWidget>
#include <QDebug>
#include <memory>

/**
 * @brief The PointList class
 * This class purpose is to display a group of knwow points to the Model of our application
 */
class PointList: public QWidget
{
public:
    PointList(Points& _points);
    ~PointList();

    QListWidget* getPointList(void) const { return widgetsList; }

    /**
     * @brief setNewGroupToDisplay
     * @param index
     * Sets the next group to display in a scrollable bar
     */
    void setNewGroupToDisplay(const int index);
    void display(void) const ;

private:
    QVBoxLayout* layout;
    QListWidget* widgetsList;
    std::shared_ptr<Points> points ;
};

#endif // POINTLIST_H
