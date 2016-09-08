/****************************************************************************
** Meta object code from reading C++ file 'pathpointcreationwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "View/pathpointcreationwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pathpointcreationwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PathPointCreationWidget_t {
    QByteArrayData data[12];
    char stringdata0[165];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PathPointCreationWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PathPointCreationWidget_t qt_meta_stringdata_PathPointCreationWidget = {
    {
QT_MOC_LITERAL(0, 0, 23), // "PathPointCreationWidget"
QT_MOC_LITERAL(1, 24, 14), // "saveEditSignal"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 24), // "PathPointCreationWidget*"
QT_MOC_LITERAL(4, 65, 16), // "cancelEditSignal"
QT_MOC_LITERAL(5, 82, 13), // "actionChanged"
QT_MOC_LITERAL(6, 96, 15), // "removePathPoint"
QT_MOC_LITERAL(7, 112, 13), // "actionClicked"
QT_MOC_LITERAL(8, 126, 6), // "action"
QT_MOC_LITERAL(9, 133, 8), // "saveEdit"
QT_MOC_LITERAL(10, 142, 10), // "cancelEdit"
QT_MOC_LITERAL(11, 153, 11) // "timeChanged"

    },
    "PathPointCreationWidget\0saveEditSignal\0"
    "\0PathPointCreationWidget*\0cancelEditSignal\0"
    "actionChanged\0removePathPoint\0"
    "actionClicked\0action\0saveEdit\0cancelEdit\0"
    "timeChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PathPointCreationWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x06 /* Public */,
       4,    1,   62,    2, 0x06 /* Public */,
       5,    3,   65,    2, 0x06 /* Public */,
       6,    1,   72,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   75,    2, 0x08 /* Private */,
       9,    0,   78,    2, 0x08 /* Private */,
      10,    0,   79,    2, 0x08 /* Private */,
      11,    1,   80,    2, 0x08 /* Private */,
       6,    0,   83,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void, QMetaType::Int, QMetaType::Int, QMetaType::QString,    2,    2,    2,
    QMetaType::Void, 0x80000000 | 3,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void,

       0        // eod
};

void PathPointCreationWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PathPointCreationWidget *_t = static_cast<PathPointCreationWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->saveEditSignal((*reinterpret_cast< PathPointCreationWidget*(*)>(_a[1]))); break;
        case 1: _t->cancelEditSignal((*reinterpret_cast< PathPointCreationWidget*(*)>(_a[1]))); break;
        case 2: _t->actionChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 3: _t->removePathPoint((*reinterpret_cast< PathPointCreationWidget*(*)>(_a[1]))); break;
        case 4: _t->actionClicked((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->saveEdit(); break;
        case 6: _t->cancelEdit(); break;
        case 7: _t->timeChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->removePathPoint(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PathPointCreationWidget* >(); break;
            }
            break;
        case 1:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PathPointCreationWidget* >(); break;
            }
            break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< PathPointCreationWidget* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (PathPointCreationWidget::*_t)(PathPointCreationWidget * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PathPointCreationWidget::saveEditSignal)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (PathPointCreationWidget::*_t)(PathPointCreationWidget * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PathPointCreationWidget::cancelEditSignal)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (PathPointCreationWidget::*_t)(int , int , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PathPointCreationWidget::actionChanged)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (PathPointCreationWidget::*_t)(PathPointCreationWidget * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PathPointCreationWidget::removePathPoint)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject PathPointCreationWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PathPointCreationWidget.data,
      qt_meta_data_PathPointCreationWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PathPointCreationWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PathPointCreationWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PathPointCreationWidget.stringdata0))
        return static_cast<void*>(const_cast< PathPointCreationWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int PathPointCreationWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void PathPointCreationWidget::saveEditSignal(PathPointCreationWidget * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PathPointCreationWidget::cancelEditSignal(PathPointCreationWidget * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void PathPointCreationWidget::actionChanged(int _t1, int _t2, QString _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void PathPointCreationWidget::removePathPoint(PathPointCreationWidget * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
