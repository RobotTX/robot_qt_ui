#ifndef MERGEMAPLISTWIDGET_H
#define MERGEMAPLISTWIDGET_H

#include <QListWidget>

/**
 * @brief The MergeMapListWidget class
 * The list widget containing the list of map in the MergeMapWidget and ScanMapWidget
 */
class MergeMapListWidget : public QListWidget {
        Q_OBJECT
public:
    MergeMapListWidget(QWidget* parent = Q_NULLPTR);

signals:
    void dirKeyPressed(int key);

protected:
    void keyPressEvent(QKeyEvent* event);

};

#endif /// MERGEMAPLISTWIDGET_H
