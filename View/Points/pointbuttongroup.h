#ifndef POINTBUTTONGROUP_H
#define POINTBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;
class QMouseEvent;
class QAbstractButton;

#include <QObject>
#include <QWidget>
#include <QSharedPointer>

/**
 * @brief The PointButtonGroup class
 * provides a widget to display a group of points and perform operations on them such as edition, deletion,
 * displaying or hiding them on the path and display information about them
 */
class PointButtonGroup: public QWidget
{
    Q_OBJECT
public:
    PointButtonGroup(QSharedPointer<Points> points, const QString groupName, QWidget *parent);
    ~PointButtonGroup(){}

    /// Getters
    QButtonGroup* getButtonGroup(void) const { return btnGroup; }
    QAbstractButton* getButtonByName(const QString name) const;
    int getButtonIdByName(const QString name) const;
    QString getGroupName(void) const { return groupName; }

    /**
     * @brief setGroup
     * @param _groupName
     * Displays the points of the group whose name is groupName
     */
    void setGroup(const QString _groupName, QSharedPointer<Points> points);

    /**
     * @brief setCheckable
     * @param checkable
     * sets the QButtonGroup to be checkable
     */
    void setCheckable(const bool checkable);

    /**
     * @brief uncheck
     * unchecks all buttons
     */
    void uncheck(void);

    /**
     * @brief updatePoints
     * remove and create the buttons
     */
    void updatePoints(QSharedPointer<Points> points);

protected:
    void resizeEvent(QResizeEvent *event);

signals:
    /// emitted when the group is recreated and connections must be reestablished
    void updateConnectionsRequest();

private:
    QVBoxLayout* layout;
    QButtonGroup* btnGroup;
    QString groupName;

};

#endif // POINTBUTTONGROUP_H
