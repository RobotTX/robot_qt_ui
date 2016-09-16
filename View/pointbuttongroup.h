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

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    QAbstractButton* getButtonByName(const QString name) const;
    int getButtonIdByName(const QString name) const;
    QString getGroupName(void) const { return groupName; }

public:
    void deleteButtons(void);
    /**
     * @brief setGroup
     * @param _groupName
     * Displays the points of the group whose name is groupName
     */
    void setGroup(const QString _groupName);
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
     * @brief createButtons
     * creates the buttons
     */
    void createButtons();

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    QString groupName;
    QSharedPointer<Points> points;

signals:
    /// emitted when the group is recreated and connections must be reestablished
    void updateConnectionsRequest();

protected:
    void resizeEvent(QResizeEvent *event);
};

#endif // POINTBUTTONGROUP_H
