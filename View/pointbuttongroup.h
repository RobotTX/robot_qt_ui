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

class PointButtonGroup: public QWidget
{
    Q_OBJECT
public:
    PointButtonGroup(QSharedPointer<Points> points, const QString groupIndex, QWidget *parent);
    ~PointButtonGroup(){}

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    QAbstractButton* getButtonByName(const QString name) const;
    int getButtonIdByName(const QString name) const;
    QString getGroupIndex(void) const { return groupIndex; }

public:
    void deleteButtons(void);
    void setGroup(const QString groupIndex);
    void setCheckable(const bool checkable);
    void uncheck(void);
    void createButtons();

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    QString groupIndex;
    QSharedPointer<Points> points;

signals:
    void updateConnectionsRequest();

protected:
    void resizeEvent(QResizeEvent *event);
};

#endif // POINTBUTTONGROUP_H
