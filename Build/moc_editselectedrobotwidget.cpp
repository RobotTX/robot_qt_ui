/****************************************************************************
** Meta object code from reading C++ file 'editselectedrobotwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../View/editselectedrobotwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'editselectedrobotwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_EditSelectedRobotWidget_t {
    QByteArrayData data[20];
    char stringdata0[282];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_EditSelectedRobotWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_EditSelectedRobotWidget_t qt_meta_stringdata_EditSelectedRobotWidget = {
    {
QT_MOC_LITERAL(0, 0, 23), // "EditSelectedRobotWidget"
QT_MOC_LITERAL(1, 24, 10), // "robotSaved"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 27), // "showEditSelectedRobotWidget"
QT_MOC_LITERAL(4, 64, 27), // "hideEditSelectedRobotWidget"
QT_MOC_LITERAL(5, 92, 8), // "showPath"
QT_MOC_LITERAL(6, 101, 15), // "clearMapOfPaths"
QT_MOC_LITERAL(7, 117, 7), // "newHome"
QT_MOC_LITERAL(8, 125, 14), // "updateHomeMenu"
QT_MOC_LITERAL(9, 140, 15), // "updatePathsMenu"
QT_MOC_LITERAL(10, 156, 26), // "saveEditSelecRobotBtnEvent"
QT_MOC_LITERAL(11, 183, 14), // "checkRobotName"
QT_MOC_LITERAL(12, 198, 13), // "checkWifiName"
QT_MOC_LITERAL(13, 212, 8), // "openMenu"
QT_MOC_LITERAL(14, 221, 12), // "openHomeMenu"
QT_MOC_LITERAL(15, 234, 9), // "deletePwd"
QT_MOC_LITERAL(16, 244, 10), // "assignPath"
QT_MOC_LITERAL(17, 255, 8), // "QAction*"
QT_MOC_LITERAL(18, 264, 6), // "action"
QT_MOC_LITERAL(19, 271, 10) // "assignHome"

    },
    "EditSelectedRobotWidget\0robotSaved\0\0"
    "showEditSelectedRobotWidget\0"
    "hideEditSelectedRobotWidget\0showPath\0"
    "clearMapOfPaths\0newHome\0updateHomeMenu\0"
    "updatePathsMenu\0saveEditSelecRobotBtnEvent\0"
    "checkRobotName\0checkWifiName\0openMenu\0"
    "openHomeMenu\0deletePwd\0assignPath\0"
    "QAction*\0action\0assignHome"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_EditSelectedRobotWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x06 /* Public */,
       3,    0,   95,    2, 0x06 /* Public */,
       4,    0,   96,    2, 0x06 /* Public */,
       5,    2,   97,    2, 0x06 /* Public */,
       6,    0,  102,    2, 0x06 /* Public */,
       7,    1,  103,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    0,  106,    2, 0x0a /* Public */,
       9,    0,  107,    2, 0x0a /* Public */,
      10,    0,  108,    2, 0x08 /* Private */,
      11,    0,  109,    2, 0x08 /* Private */,
      12,    0,  110,    2, 0x08 /* Private */,
      13,    0,  111,    2, 0x08 /* Private */,
      14,    0,  112,    2, 0x08 /* Private */,
      15,    0,  113,    2, 0x08 /* Private */,
      16,    1,  114,    2, 0x08 /* Private */,
      19,    1,  117,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    2,    2,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 17,   18,
    QMetaType::Void, 0x80000000 | 17,   18,

       0        // eod
};

void EditSelectedRobotWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        EditSelectedRobotWidget *_t = static_cast<EditSelectedRobotWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->robotSaved(); break;
        case 1: _t->showEditSelectedRobotWidget(); break;
        case 2: _t->hideEditSelectedRobotWidget(); break;
        case 3: _t->showPath((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 4: _t->clearMapOfPaths(); break;
        case 5: _t->newHome((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->updateHomeMenu(); break;
        case 7: _t->updatePathsMenu(); break;
        case 8: _t->saveEditSelecRobotBtnEvent(); break;
        case 9: _t->checkRobotName(); break;
        case 10: _t->checkWifiName(); break;
        case 11: _t->openMenu(); break;
        case 12: _t->openHomeMenu(); break;
        case 13: _t->deletePwd(); break;
        case 14: _t->assignPath((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 15: _t->assignHome((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (EditSelectedRobotWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&EditSelectedRobotWidget::robotSaved)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (EditSelectedRobotWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&EditSelectedRobotWidget::showEditSelectedRobotWidget)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (EditSelectedRobotWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&EditSelectedRobotWidget::hideEditSelectedRobotWidget)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (EditSelectedRobotWidget::*_t)(QString , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&EditSelectedRobotWidget::showPath)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (EditSelectedRobotWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&EditSelectedRobotWidget::clearMapOfPaths)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (EditSelectedRobotWidget::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&EditSelectedRobotWidget::newHome)) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject EditSelectedRobotWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_EditSelectedRobotWidget.data,
      qt_meta_data_EditSelectedRobotWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *EditSelectedRobotWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *EditSelectedRobotWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_EditSelectedRobotWidget.stringdata0))
        return static_cast<void*>(const_cast< EditSelectedRobotWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int EditSelectedRobotWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}

// SIGNAL 0
void EditSelectedRobotWidget::robotSaved()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void EditSelectedRobotWidget::showEditSelectedRobotWidget()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void EditSelectedRobotWidget::hideEditSelectedRobotWidget()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void EditSelectedRobotWidget::showPath(QString _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void EditSelectedRobotWidget::clearMapOfPaths()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}

// SIGNAL 5
void EditSelectedRobotWidget::newHome(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
