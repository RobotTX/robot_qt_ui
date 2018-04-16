#ifndef PATHPOINT_H
#define PATHPOINT_H

class Point;

#include <QObject>
#include <QPointer>

class PathPoint : public QObject {

public:

    PathPoint(const QString _name, const double _x, const double _y, const int _waitTime, const int _orientation, const QString _speechName, const QString _speechContent, const int _speechTime, QObject *parent);

    QPointer<Point> getPoint(void) const { return point; }
    int getWaitTime(void) const { return waitTime; }
    QString getSpeechName(void) const { return speechName; }
    QString getSpeechContent(void) const { return speechContent; }
    int getSpeechTime(void) const { return speechTime; }

    void setWaitTime(const int _waitTime) { waitTime = _waitTime; }

private:
    /**
     * @brief waitTime
     * How long the robot needs to wait if its action is to wait
     */
    int waitTime;

    /**
     * @brief speechName
     * speech name sent to robot
     */
    QString speechName;

    /**
     * @brief speechContent
     * What would the robot say along his path
     */
    QString speechContent;

    /**
     * @brief speechTime
     * How long the robot needs to wait before saying a speech
     */
    int speechTime;

    /**
     * @brief point
     * The point where the robot goes
     */
    QPointer<Point> point;
};

#endif /// PATHPOINT_H
