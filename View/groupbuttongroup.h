#ifndef GROUPBUTTONGROUP_H
#define GROUPBUTTONGROUP_H

class Points;
class QButtonGroup;
class QVBoxLayout;
class QMouseEvent;
class QLineEdit;

#include <QButtonGroup>
#include <QWidget>
#include <memory>
#include <QObject>

class GroupButtonGroup: public QWidget
{
    Q_OBJECT
public:
    GroupButtonGroup(Points const& _points, QWidget *_parent);
    ~GroupButtonGroup(){}

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    QLineEdit* getModifyEdit(void) const { return modifyEdit; }

public:
    void deleteButtons(void);
    void update(const Points& _points);
    void uncheck(void);
    void setEnabled(const bool enable);

protected:
    void mouseDoubleClickEvent(QMouseEvent *event);

private:
    QLineEdit* modifyEdit;
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
    QWidget* parent;
    /// to avoid resizing of the icons after deletions of points and groups
    const QSize BUTTON_SIZE = parentWidget()->size()/2;

signals:
    void doubleClick(int);
    void updateConnectionsRequest();

};

#endif // GROUPBUTTONGROUP_H
