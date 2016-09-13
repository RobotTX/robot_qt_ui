/****************************************************************************
** Meta object code from reading C++ file 'pathpainter.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../View/pathpainter.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pathpainter.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PathPainter_t {
    QByteArrayData data[22];
    char stringdata0[261];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PathPainter_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PathPainter_t qt_meta_stringdata_PathPainter = {
    {
QT_MOC_LITERAL(0, 0, 11), // "PathPainter"
QT_MOC_LITERAL(1, 12, 12), // "updatePoints"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 2), // "id"
QT_MOC_LITERAL(4, 29, 4), // "name"
QT_MOC_LITERAL(5, 34, 13), // "resetPathSlot"
QT_MOC_LITERAL(6, 48, 16), // "GraphicItemState"
QT_MOC_LITERAL(7, 65, 6), // "_state"
QT_MOC_LITERAL(8, 72, 16), // "addPathPointSlot"
QT_MOC_LITERAL(9, 89, 1), // "x"
QT_MOC_LITERAL(10, 91, 1), // "y"
QT_MOC_LITERAL(11, 93, 19), // "deletePathPointSlot"
QT_MOC_LITERAL(12, 113, 21), // "updatePathPainterSlot"
QT_MOC_LITERAL(13, 135, 8), // "savePath"
QT_MOC_LITERAL(14, 144, 30), // "updatePathPainterPointViewSlot"
QT_MOC_LITERAL(15, 175, 25), // "orderPathPointChangedSlot"
QT_MOC_LITERAL(16, 201, 4), // "from"
QT_MOC_LITERAL(17, 206, 2), // "to"
QT_MOC_LITERAL(18, 209, 17), // "actionChangedSlot"
QT_MOC_LITERAL(19, 227, 6), // "action"
QT_MOC_LITERAL(20, 234, 8), // "waitTime"
QT_MOC_LITERAL(21, 243, 17) // "editPathPointSlot"

    },
    "PathPainter\0updatePoints\0\0id\0name\0"
    "resetPathSlot\0GraphicItemState\0_state\0"
    "addPathPointSlot\0x\0y\0deletePathPointSlot\0"
    "updatePathPainterSlot\0savePath\0"
    "updatePathPainterPointViewSlot\0"
    "orderPathPointChangedSlot\0from\0to\0"
    "actionChangedSlot\0action\0waitTime\0"
    "editPathPointSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PathPainter[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   59,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   64,    2, 0x08 /* Private */,
       8,    3,   67,    2, 0x08 /* Private */,
      11,    2,   74,    2, 0x08 /* Private */,
      12,    2,   79,    2, 0x08 /* Private */,
      14,    1,   84,    2, 0x08 /* Private */,
      15,    2,   87,    2, 0x08 /* Private */,
      18,    3,   92,    2, 0x08 /* Private */,
      21,    4,   99,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::QString,    3,    4,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double,    4,    9,   10,
    QMetaType::Void, QMetaType::Int, 0x80000000 | 6,    3,    7,
    QMetaType::Void, 0x80000000 | 6, QMetaType::Bool,    7,   13,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   16,   17,
    QMetaType::Void, QMetaType::Int, QMetaType::Int, QMetaType::QString,    3,   19,   20,
    QMetaType::Void, QMetaType::Int, QMetaType::QString, QMetaType::Double, QMetaType::Double,    3,    4,    9,   10,

       0        // eod
};

void PathPainter::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PathPainter *_t = static_cast<PathPainter *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updatePoints((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 1: _t->resetPathSlot((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 2: _t->addPathPointSlot((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 3: _t->deletePathPointSlot((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< GraphicItemState(*)>(_a[2]))); break;
        case 4: _t->updatePathPainterSlot((*reinterpret_cast< GraphicItemState(*)>(_a[1])),(*reinterpret_cast< const bool(*)>(_a[2]))); break;
        case 5: _t->updatePathPainterPointViewSlot((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 6: _t->orderPathPointChangedSlot((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 7: _t->actionChangedSlot((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 8: _t->editPathPointSlot((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (PathPainter::*_t)(int , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PathPainter::updatePoints)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject PathPainter::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_PathPainter.data,
      qt_meta_data_PathPainter,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PathPainter::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PathPainter::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PathPainter.stringdata0))
        return static_cast<void*>(const_cast< PathPainter*>(this));
    if (!strcmp(_clname, "QGraphicsPathItem"))
        return static_cast< QGraphicsPathItem*>(const_cast< PathPainter*>(this));
    return QObject::qt_metacast(_clname);
}

int PathPainter::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void PathPainter::updatePoints(int _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
