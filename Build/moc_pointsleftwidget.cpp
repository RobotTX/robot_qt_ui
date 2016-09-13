/****************************************************************************
** Meta object code from reading C++ file 'pointsleftwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../View/pointsleftwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pointsleftwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PointsLeftWidget_t {
    QByteArrayData data[21];
    char stringdata0[317];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PointsLeftWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PointsLeftWidget_t qt_meta_stringdata_PointsLeftWidget = {
    {
QT_MOC_LITERAL(0, 0, 16), // "PointsLeftWidget"
QT_MOC_LITERAL(1, 17, 8), // "newGroup"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 4), // "name"
QT_MOC_LITERAL(4, 32, 13), // "modifiedGroup"
QT_MOC_LITERAL(5, 46, 23), // "modifiedGroupAfterClick"
QT_MOC_LITERAL(6, 70, 12), // "enableReturn"
QT_MOC_LITERAL(7, 83, 20), // "messageCreationGroup"
QT_MOC_LITERAL(8, 104, 20), // "messageCreationPoint"
QT_MOC_LITERAL(9, 125, 19), // "resetPathPointViews"
QT_MOC_LITERAL(10, 145, 11), // "deleteGroup"
QT_MOC_LITERAL(11, 157, 14), // "checkGroupName"
QT_MOC_LITERAL(12, 172, 13), // "enableButtons"
QT_MOC_LITERAL(13, 186, 6), // "button"
QT_MOC_LITERAL(14, 193, 16), // "QAbstractButton*"
QT_MOC_LITERAL(15, 210, 19), // "cancelCreationGroup"
QT_MOC_LITERAL(16, 230, 18), // "emitNewGroupSignal"
QT_MOC_LITERAL(17, 249, 21), // "modifyGroupAfterClick"
QT_MOC_LITERAL(18, 271, 19), // "reconnectModifyEdit"
QT_MOC_LITERAL(19, 291, 20), // "sendMessageEditGroup"
QT_MOC_LITERAL(20, 312, 4) // "code"

    },
    "PointsLeftWidget\0newGroup\0\0name\0"
    "modifiedGroup\0modifiedGroupAfterClick\0"
    "enableReturn\0messageCreationGroup\0"
    "messageCreationPoint\0resetPathPointViews\0"
    "deleteGroup\0checkGroupName\0enableButtons\0"
    "button\0QAbstractButton*\0cancelCreationGroup\0"
    "emitNewGroupSignal\0modifyGroupAfterClick\0"
    "reconnectModifyEdit\0sendMessageEditGroup\0"
    "code"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PointsLeftWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       8,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   94,    2, 0x06 /* Public */,
       4,    1,   97,    2, 0x06 /* Public */,
       5,    1,  100,    2, 0x06 /* Public */,
       6,    0,  103,    2, 0x06 /* Public */,
       7,    2,  104,    2, 0x06 /* Public */,
       8,    0,  109,    2, 0x06 /* Public */,
       9,    0,  110,    2, 0x06 /* Public */,
      10,    0,  111,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    1,  112,    2, 0x0a /* Public */,
      12,    1,  115,    2, 0x08 /* Private */,
      12,    1,  118,    2, 0x08 /* Private */,
      15,    0,  121,    2, 0x08 /* Private */,
      16,    0,  122,    2, 0x08 /* Private */,
      17,    1,  123,    2, 0x08 /* Private */,
      18,    0,  126,    2, 0x08 /* Private */,
      19,    1,  127,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Int, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void, 0x80000000 | 14,   13,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   20,

       0        // eod
};

void PointsLeftWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PointsLeftWidget *_t = static_cast<PointsLeftWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newGroup((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->modifiedGroup((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->modifiedGroupAfterClick((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->enableReturn(); break;
        case 4: _t->messageCreationGroup((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 5: _t->messageCreationPoint(); break;
        case 6: _t->resetPathPointViews(); break;
        case 7: _t->deleteGroup(); break;
        case 8: { int _r = _t->checkGroupName((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 9: _t->enableButtons((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: _t->enableButtons((*reinterpret_cast< QAbstractButton*(*)>(_a[1]))); break;
        case 11: _t->cancelCreationGroup(); break;
        case 12: _t->emitNewGroupSignal(); break;
        case 13: _t->modifyGroupAfterClick((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 14: _t->reconnectModifyEdit(); break;
        case 15: _t->sendMessageEditGroup((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (PointsLeftWidget::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointsLeftWidget::newGroup)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (PointsLeftWidget::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointsLeftWidget::modifiedGroup)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (PointsLeftWidget::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointsLeftWidget::modifiedGroupAfterClick)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (PointsLeftWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointsLeftWidget::enableReturn)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (PointsLeftWidget::*_t)(QString , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointsLeftWidget::messageCreationGroup)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (PointsLeftWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointsLeftWidget::messageCreationPoint)) {
                *result = 5;
                return;
            }
        }
        {
            typedef void (PointsLeftWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointsLeftWidget::resetPathPointViews)) {
                *result = 6;
                return;
            }
        }
        {
            typedef void (PointsLeftWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PointsLeftWidget::deleteGroup)) {
                *result = 7;
                return;
            }
        }
    }
}

const QMetaObject PointsLeftWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PointsLeftWidget.data,
      qt_meta_data_PointsLeftWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PointsLeftWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PointsLeftWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PointsLeftWidget.stringdata0))
        return static_cast<void*>(const_cast< PointsLeftWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int PointsLeftWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void PointsLeftWidget::newGroup(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PointsLeftWidget::modifiedGroup(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void PointsLeftWidget::modifiedGroupAfterClick(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void PointsLeftWidget::enableReturn()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void PointsLeftWidget::messageCreationGroup(QString _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void PointsLeftWidget::messageCreationPoint()
{
    QMetaObject::activate(this, &staticMetaObject, 5, Q_NULLPTR);
}

// SIGNAL 6
void PointsLeftWidget::resetPathPointViews()
{
    QMetaObject::activate(this, &staticMetaObject, 6, Q_NULLPTR);
}

// SIGNAL 7
void PointsLeftWidget::deleteGroup()
{
    QMetaObject::activate(this, &staticMetaObject, 7, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
