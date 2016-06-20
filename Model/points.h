#ifndef POINTS_H
#define POINTS_H

class Group;
class Point;
class QDataStream;

#include <QVector>
#include <QString>
#include <memory>
#include <iostream>

/**
 * @brief The Points class
 * This class provides a model for a list of points organized in groups
 * A Points objet is identified by a vector of pointers on Group objets
 */

class Points {

public:
    Points(void);

    /// a helper class to overload the << operator
    void display(std::ostream& stream) const;

public:
    QVector<std::shared_ptr<Group>> getGroups(void) const { return groups; }

    /**
     * @brief count
     * @return int
     * returns the number of groups of the list of points
     */
    int count(void) const { return groups.size(); }

    bool addGroup(const Group& group);

    /**
     * @brief groupNames
     * @return QVector<QString>
     * returns a vector containing the names of the groups that compose our list of points
     */
    QVector<QString> groupNames(void) const;

    /**
     * @brief removeGroup
     * @param index
     * Checks whether or not the index is within the range (0, #_of_groups-1)
     * if so, removes the index-th group of the current Points object
     * Otherwise does not do anything
     */
    void removeGroup(const int index);

    /**
     * @brief findGroup
     * @param groupName
     * @return std::shared_ptr<Group>
     * returns a pointer on the group whose name is the one passed as argument if such group exists
     * otherwise returns a NULL pointer
     */
    std::shared_ptr<Group> findGroup(const QString groupName) const;

    /**
     * @brief findPoint
     * @param name
     * @return std::shared_ptr<Point>
     * returns a pointer on the Point whose name is passed as an argument
     */
    std::shared_ptr<Point> findPoint(const QString name) const;

    /**
     * @brief findPointIndexes
     * @param name
     * @return std::pair<int, int>
     * returns a pair of indexes which correspond to the group index and the index of the point whose name is passed as an argument
     *  in this group respectively
     */
    std::pair<int, int> findPointIndexes(const QString name) const;

    void clearGroups();

private:
    QVector<std::shared_ptr<Group>> groups;
};


std::ostream& operator <<(std::ostream& stream, Points const& points);

/**
 * @brief operator <<
 * @param out
 * @param group
 * @return QDataStream&
 * Overloads the << and >> operators in order to be able to serialize a Group objet
 */
QDataStream& operator<<(QDataStream& out, const Points& points);
QDataStream& operator>>(QDataStream& in, Points& points);

#endif // POINTS_H
