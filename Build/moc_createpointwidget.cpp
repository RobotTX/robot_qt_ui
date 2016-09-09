/****************************************************************************
** Meta object code from reading C++ file 'createpointwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../View/createpointwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'createpointwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_CreatePointWidget_t {
    QByteArrayData data[12];
    char stringdata0[191];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CreatePointWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CreatePointWidget_t qt_meta_stringdata_CreatePointWidget = {
    {
QT_MOC_LITERAL(0, 0, 17), // "CreatePointWidget"
QT_MOC_LITERAL(1, 18, 10), // "pointSaved"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 11), // "invalidName"
QT_MOC_LITERAL(4, 42, 24), // "CreatePointWidget::Error"
QT_MOC_LITERAL(5, 67, 22), // "displayMessageCreation"
QT_MOC_LITERAL(6, 90, 15), // "resetMessageTop"
QT_MOC_LITERAL(7, 106, 26), // "saveEditSelecPointBtnEvent"
QT_MOC_LITERAL(8, 133, 14), // "checkPointName"
QT_MOC_LITERAL(9, 148, 15), // "showGroupLayout"
QT_MOC_LITERAL(10, 164, 15), // "hideGroupLayout"
QT_MOC_LITERAL(11, 180, 10) // "pointAdded"

    },
    "CreatePointWidget\0pointSaved\0\0invalidName\0"
    "CreatePointWidget::Error\0"
    "displayMessageCreation\0resetMessageTop\0"
    "saveEditSelecPointBtnEvent\0checkPointName\0"
    "showGroupLayout\0hideGroupLayout\0"
    "pointAdded"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CreatePointWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    4,   54,    2, 0x06 /* Public */,
       3,    2,   63,    2, 0x06 /* Public */,
       5,    1,   68,    2, 0x06 /* Public */,
       6,    2,   71,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   76,    2, 0x08 /* Private */,
       8,    0,   77,    2, 0x08 /* Private */,
       9,    0,   78,    2, 0x08 /* Private */,
      10,    1,   79,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double, QMetaType::QString,    2,    2,    2,    2,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 4,    2,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    2,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Int,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   11,

       0        // eod
};

void CreatePointWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        CreatePointWidget *_t = static_cast<CreatePointWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->pointSaved((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 1: _t->invalidName((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< CreatePointWidget::Error(*)>(_a[2]))); break;
        case 2: _t->displayMessageCreation((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->resetMessageTop((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 4: _t->saveEditSelecPointBtnEvent(); break;
        case 5: { int _r = _t->checkPointName();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 6: _t->showGroupLayout(); break;
        case 7: _t->hideGroupLayout((*reinterpret_cast< const bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (CreatePointWidget::*_t)(QString , double , double , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CreatePointWidget::pointSaved)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (CreatePointWidget::*_t)(QString , CreatePointWidget::Error );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CreatePointWidget::invalidName)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (CreatePointWidget::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CreatePointWidget::displayMessageCreation)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (CreatePointWidget::*_t)(QString , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&CreatePointWidget::resetMessageTop)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject CreatePointWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_CreatePointWidget.data,
      qt_meta_data_CreatePointWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CreatePointWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CreatePointWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CreatePointWidget.stringdata0))
        return static_cast<void*>(const_cast< CreatePointWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int CreatePointWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void CreatePointWidget::pointSaved(QString _t1, double _t2, double _t3, QString _t4)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CreatePointWidget::invalidName(QString _t1, CreatePointWidget::Error _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CreatePointWidget::displayMessageCreation(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void CreatePointWidget::resetMessageTop(QString _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
