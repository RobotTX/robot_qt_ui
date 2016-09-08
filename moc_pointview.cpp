/****************************************************************************
** Meta object code from reading C++ file 'pointview.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "View/pointview.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pointview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PointView_t {
    QByteArrayData data[18];
    char stringdata0[272];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PointView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PointView_t qt_meta_stringdata_PointView = {
    {
QT_MOC_LITERAL(0, 0, 9), // "PointView"
QT_MOC_LITERAL(1, 10, 17), // "pointRightClicked"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 16), // "pointLeftClicked"
QT_MOC_LITERAL(4, 46, 4), // "name"
QT_MOC_LITERAL(5, 51, 1), // "x"
QT_MOC_LITERAL(6, 53, 1), // "y"
QT_MOC_LITERAL(7, 55, 12), // "addPointPath"
QT_MOC_LITERAL(8, 68, 16), // "GraphicItemState"
QT_MOC_LITERAL(9, 85, 19), // "addNoRobotPointPath"
QT_MOC_LITERAL(10, 105, 10), // "homeEdited"
QT_MOC_LITERAL(11, 116, 19), // "moveEditedPathPoint"
QT_MOC_LITERAL(12, 136, 26), // "editedPointPositionChanged"
QT_MOC_LITERAL(13, 163, 25), // "editedHomePositionChanged"
QT_MOC_LITERAL(14, 189, 16), // "pathPointChanged"
QT_MOC_LITERAL(15, 206, 16), // "hoverEventSignal"
QT_MOC_LITERAL(16, 223, 21), // "PointView::PixmapType"
QT_MOC_LITERAL(17, 245, 26) // "updatePathPainterPointView"

    },
    "PointView\0pointRightClicked\0\0"
    "pointLeftClicked\0name\0x\0y\0addPointPath\0"
    "GraphicItemState\0addNoRobotPointPath\0"
    "homeEdited\0moveEditedPathPoint\0"
    "editedPointPositionChanged\0"
    "editedHomePositionChanged\0pathPointChanged\0"
    "hoverEventSignal\0PointView::PixmapType\0"
    "updatePathPainterPointView"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PointView[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      11,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x06 /* Public */,
       3,    3,   72,    2, 0x06 /* Public */,
       7,    4,   79,    2, 0x06 /* Public */,
       9,    3,   88,    2, 0x06 /* Public */,
      10,    1,   95,    2, 0x06 /* Public */,
      11,    1,   98,    2, 0x06 /* Public */,
      12,    2,  101,    2, 0x06 /* Public */,
      13,    3,  106,    2, 0x06 /* Public */,
      14,    2,  113,    2, 0x06 /* Public */,
      15,    2,  118,    2, 0x06 /* Public */,
      17,    0,  123,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double,    4,    5,    6,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double, 0x80000000 | 8,    4,    5,    6,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double,    4,    5,    6,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, 0x80000000 | 8,    2,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    2,    2,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::QString,    2,    2,    2,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    2,    2,
    QMetaType::Void, 0x80000000 | 16, QMetaType::QString,    2,    2,
    QMetaType::Void,

       0        // eod
};

void PointView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PointView *_t = static_cast<PointView *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->pointRightClicked((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->pointLeftClicked((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 2: _t->addPointPath((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< GraphicItemState(*)>(_a[4]))); break;
        case 3: _t->addNoRobotPointPath((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 4: _t->homeEdited((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->moveEditedPathPoint((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 6: _t->editedPointPositionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 7: _t->editedHomePositionChanged((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 8: _t->pathPointChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 9: _t->hoverEventSignal((*reinterpret_cast< PointView::PixmapType(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 10: _t->updatePathPainterPointView(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (PointView::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::pointRightClicked)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (PointView::*_t)(QString , double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::pointLeftClicked)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (PointView::*_t)(QString , double , double , GraphicItemState );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::addPointPath)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (PointView::*_t)(QString , double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::addNoRobotPointPath)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (PointView::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::homeEdited)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (PointView::*_t)(GraphicItemState );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::moveEditedPathPoint)) {
                *result = 5;
                return;
            }
        }
        {
            typedef void (PointView::*_t)(double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::editedPointPositionChanged)) {
                *result = 6;
                return;
            }
        }
        {
            typedef void (PointView::*_t)(float , float , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::editedHomePositionChanged)) {
                *result = 7;
                return;
            }
        }
        {
            typedef void (PointView::*_t)(double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::pathPointChanged)) {
                *result = 8;
                return;
            }
        }
        {
            typedef void (PointView::*_t)(PointView::PixmapType , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::hoverEventSignal)) {
                *result = 9;
                return;
            }
        }
        {
            typedef void (PointView::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointView::updatePathPainterPointView)) {
                *result = 10;
                return;
            }
        }
    }
}

const QMetaObject PointView::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_PointView.data,
      qt_meta_data_PointView,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PointView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PointView::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PointView.stringdata0))
        return static_cast<void*>(const_cast< PointView*>(this));
    if (!strcmp(_clname, "QGraphicsPixmapItem"))
        return static_cast< QGraphicsPixmapItem*>(const_cast< PointView*>(this));
    return QObject::qt_metacast(_clname);
}

int PointView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void PointView::pointRightClicked(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PointView::pointLeftClicked(QString _t1, double _t2, double _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void PointView::addPointPath(QString _t1, double _t2, double _t3, GraphicItemState _t4)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void PointView::addNoRobotPointPath(QString _t1, double _t2, double _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void PointView::homeEdited(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void PointView::moveEditedPathPoint(GraphicItemState _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void PointView::editedPointPositionChanged(double _t1, double _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void PointView::editedHomePositionChanged(float _t1, float _t2, QString _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void PointView::pathPointChanged(double _t1, double _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void PointView::hoverEventSignal(PointView::PixmapType _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void PointView::updatePathPainterPointView()
{
    QMetaObject::activate(this, &staticMetaObject, 10, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
