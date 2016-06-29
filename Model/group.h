#ifndef GROUP_H
#define GROUP_H

class Points;
class Point;

#include <QVector>
#include <memory>

/**
 * @brief The Group class
 * This class provides a model for a group of points
 * A group is identified by a name that is unique and a vector of pointors on Point objets
 * All points are unique in that two points cannot share the same name
 */


class Group
{
public:
    Group(void);
    Group(const QString _name);
    Group(const std::shared_ptr<Points>& _groupPoints, const QString _name);

    QString getName(void) const { return name; }
    void setName(const QString _name) { name = _name; }
    QVector<std::shared_ptr<Point>> getPoints(void) const { return points; }
    /// to count the number of points in the group
    int count(void) const { return points.size(); }

    /// a simple helper function to overload the << operator
    void display(std::ostream&) const;
    /// returns true if all the points in the group are displayed
    bool isDisplayed(void) const;

public:
    /// returns true if one of the group's points is a home for a robot
    std::shared_ptr<Point> containsHomePoint(void) const;
    /// returns true if there is no points in the group
    bool isEmpty(void) const { return points.size() == 0; }

public:
    /**
     * @brief addPoint
     * @param pointPtr
     * @return bool
     * add a point to the current group using a pointer on a Point objet
     * checks whether the current group already contains a point with the same name, if yes the point is not added
     * and the function returns 0, else if the name is empty thee the point is not added and the function returns -1
     * otherwize the point is added and the function returns true
     * This function is overloaded so that it can take a name and coordinates to create a point on the fly and add it
     * It can also take a const reference on an existing point
     */
    bool addPoint(const std::shared_ptr<Point> pointPtr);
    int addPoint(const QString name, const float x, const float y);

    /**
     * @brief removePoint
     * @param name
     * Takes a name as argument and removes the point whose name corresponds to it
     * Nothing is done if such point does not exist
     */
    void removePoint(const QString name);

    /**
     * @brief removePoint
     * @param index
     * Checks whether or not the index is within the range (0, size_of_the_group-1)
     * if so, removes the index-th point of the group
     */
    void removePoint(const int index);

private:
    std::shared_ptr<Points> groupPoints;
    QString name;
    QVector<std::shared_ptr<Point>> points;
};

std::ostream& operator <<(std::ostream& stream, const Group& group);

/**
 * @brief operator <<
 * @param out
 * @param group
 * @return QDataStream&
 * Overloads the << and >> operators in order to be able to serialize a Group objet
 */
QDataStream& operator<<(QDataStream& out, const Group& group);
QDataStream& operator>>(QDataStream& in, Group& group);


#endif // GROUP_H
