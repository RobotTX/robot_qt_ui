/****************************************************************************
** Meta object code from reading C++ file 'displayselectedpoint.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "View/displayselectedpoint.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'displayselectedpoint.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_DisplaySelectedPoint_t {
    QByteArrayData data[11];
    char stringdata0[157];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DisplaySelectedPoint_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DisplaySelectedPoint_t qt_meta_stringdata_DisplaySelectedPoint = {
    {
QT_MOC_LITERAL(0, 0, 20), // "DisplaySelectedPoint"
QT_MOC_LITERAL(1, 21, 11), // "nameChanged"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 10), // "resetState"
QT_MOC_LITERAL(4, 45, 16), // "GraphicItemState"
QT_MOC_LITERAL(5, 62, 11), // "invalidName"
QT_MOC_LITERAL(6, 74, 24), // "CreatePointWidget::Error"
QT_MOC_LITERAL(7, 99, 25), // "setSelectedRobotFromPoint"
QT_MOC_LITERAL(8, 125, 11), // "removePoint"
QT_MOC_LITERAL(9, 137, 14), // "checkPointName"
QT_MOC_LITERAL(10, 152, 4) // "name"

    },
    "DisplaySelectedPoint\0nameChanged\0\0"
    "resetState\0GraphicItemState\0invalidName\0"
    "CreatePointWidget::Error\0"
    "setSelectedRobotFromPoint\0removePoint\0"
    "checkPointName\0name"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DisplaySelectedPoint[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   44,    2, 0x06 /* Public */,
       3,    1,   49,    2, 0x06 /* Public */,
       5,    2,   52,    2, 0x06 /* Public */,
       7,    1,   57,    2, 0x06 /* Public */,
       8,    0,   60,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    1,   61,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    2,    2,
    QMetaType::Void, 0x80000000 | 4,    2,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 6,    2,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Int, QMetaType::QString,   10,

       0        // eod
};

void DisplaySelectedPoint::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DisplaySelectedPoint *_t = static_cast<DisplaySelectedPoint *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->nameChanged((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 1: _t->resetState((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 2: _t->invalidName((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< CreatePointWidget::Error(*)>(_a[2]))); break;
        case 3: _t->setSelectedRobotFromPoint((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->removePoint(); break;
        case 5: { int _r = _t->checkPointName((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (DisplaySelectedPoint::*_t)(QString , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DisplaySelectedPoint::nameChanged)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (DisplaySelectedPoint::*_t)(GraphicItemState );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DisplaySelectedPoint::resetState)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (DisplaySelectedPoint::*_t)(QString , CreatePointWidget::Error );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DisplaySelectedPoint::invalidName)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (DisplaySelectedPoint::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DisplaySelectedPoint::setSelectedRobotFromPoint)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (DisplaySelectedPoint::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DisplaySelectedPoint::removePoint)) {
                *result = 4;
                return;
            }
        }
    }
}

const QMetaObject DisplaySelectedPoint::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_DisplaySelectedPoint.data,
      qt_meta_data_DisplaySelectedPoint,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *DisplaySelectedPoint::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DisplaySelectedPoint::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_DisplaySelectedPoint.stringdata0))
        return static_cast<void*>(const_cast< DisplaySelectedPoint*>(this));
    return QWidget::qt_metacast(_clname);
}

int DisplaySelectedPoint::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void DisplaySelectedPoint::nameChanged(QString _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void DisplaySelectedPoint::resetState(GraphicItemState _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void DisplaySelectedPoint::invalidName(QString _t1, CreatePointWidget::Error _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void DisplaySelectedPoint::setSelectedRobotFromPoint(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void DisplaySelectedPoint::removePoint()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
