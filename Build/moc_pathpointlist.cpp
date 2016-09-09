/****************************************************************************
** Meta object code from reading C++ file 'pathpointlist.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../View/pathpointlist.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pathpointlist.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PathPointList_t {
    QByteArrayData data[9];
    char stringdata0[74];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PathPointList_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PathPointList_t qt_meta_stringdata_PathPointList = {
    {
QT_MOC_LITERAL(0, 0, 13), // "PathPointList"
QT_MOC_LITERAL(1, 14, 15), // "itemMovedSignal"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 9), // "itemMoved"
QT_MOC_LITERAL(4, 41, 6), // "parent"
QT_MOC_LITERAL(5, 48, 5), // "start"
QT_MOC_LITERAL(6, 54, 3), // "end"
QT_MOC_LITERAL(7, 58, 11), // "destination"
QT_MOC_LITERAL(8, 70, 3) // "row"

    },
    "PathPointList\0itemMovedSignal\0\0itemMoved\0"
    "parent\0start\0end\0destination\0row"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PathPointList[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    5,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    5,   35,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QModelIndex, QMetaType::Int, QMetaType::Int, QMetaType::QModelIndex, QMetaType::Int,    2,    2,    2,    2,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::QModelIndex, QMetaType::Int, QMetaType::Int, QMetaType::QModelIndex, QMetaType::Int,    4,    5,    6,    7,    8,

       0        // eod
};

void PathPointList::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PathPointList *_t = static_cast<PathPointList *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->itemMovedSignal((*reinterpret_cast< QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QModelIndex(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5]))); break;
        case 1: _t->itemMoved((*reinterpret_cast< QModelIndex(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QModelIndex(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (PathPointList::*_t)(QModelIndex , int , int , QModelIndex , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PathPointList::itemMovedSignal)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject PathPointList::staticMetaObject = {
    { &QListWidget::staticMetaObject, qt_meta_stringdata_PathPointList.data,
      qt_meta_data_PathPointList,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PathPointList::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PathPointList::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PathPointList.stringdata0))
        return static_cast<void*>(const_cast< PathPointList*>(this));
    return QListWidget::qt_metacast(_clname);
}

int PathPointList::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QListWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void PathPointList::itemMovedSignal(QModelIndex _t1, int _t2, int _t3, QModelIndex _t4, int _t5)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
