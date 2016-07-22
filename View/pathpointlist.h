#ifndef PATHPOINTLIST_H
#define PATHPOINTLIST_H

class QModelIndex;
#include <QListWidget>

/**
 * @brief The PathPointList class
 * The widget which display the list of pathPoint in the left menu when creating a path
 */
class PathPointList : public QListWidget{
    Q_OBJECT

public:
    PathPointList(QWidget *parent);
    void update(const int indexNb, const int action, const int time = 0);

    void refresh(void);

private slots:
    void itemMoved(QModelIndex parent, int start, int end, QModelIndex destination, int row);

signals:
    void itemMovedSignal(QModelIndex, int, int, QModelIndex, int);
};

#endif // PATHPOINTLIST_H
