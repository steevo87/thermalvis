/****************************************************************************
** Meta object code from reading C++ file 'mainwindow_streamer.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../streamer/mainwindow_streamer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow_streamer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow_streamer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      29,   21,   20,   20, 0x08,
      56,   21,   20,   20, 0x08,
      85,   21,   20,   20, 0x08,
     113,   21,   20,   20, 0x08,
     142,   21,   20,   20, 0x08,
     171,   21,   20,   20, 0x08,
     204,   21,   20,   20, 0x08,
     243,  237,   20,   20, 0x08,
     280,  237,   20,   20, 0x08,
     316,  237,   20,   20, 0x08,
     358,  237,   20,   20, 0x08,
     399,  237,   20,   20, 0x08,
     435,   20,   20,   20, 0x08,
     470,   20,   20,   20, 0x08,
     516,   20,   20,   20, 0x08,
     550,   20,   20,   20, 0x08,
     587,   20,   20,   20, 0x08,
     626,   20,   20,   20, 0x08,
     655,   20,   20,   20, 0x08,
     687,   20,   20,   20, 0x08,
     717,   20,   20,   20, 0x08,
     749,   20,   20,   20, 0x08,
     786,   20,   20,   20, 0x08,
     821,   20,   20,   20, 0x08,
     848,   20,   20,   20, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow_streamer[] = {
    "MainWindow_streamer\0\0checked\0"
    "on_debugMode_toggled(bool)\0"
    "on_verboseMode_toggled(bool)\0"
    "on_output8bit_toggled(bool)\0"
    "on_output16bit_toggled(bool)\0"
    "on_outputColor_toggled(bool)\0"
    "on_autoTemperature_toggled(bool)\0"
    "on_undistortImages_toggled(bool)\0index\0"
    "on_normMode_currentIndexChanged(int)\0"
    "on_mapCode_currentIndexChanged(int)\0"
    "on_inputDatatype_currentIndexChanged(int)\0"
    "on_detectorMode_currentIndexChanged(int)\0"
    "on_usbMode_currentIndexChanged(int)\0"
    "on_maxReadAttempts_returnPressed()\0"
    "on_desiredDegreesPerGraylevel_returnPressed()\0"
    "on_maxNucInterval_returnPressed()\0"
    "on_zeroDegreesOffset_returnPressed()\0"
    "on_degreesPerGraylevel_returnPressed()\0"
    "on_framerate_returnPressed()\0"
    "on_threshFactor_returnPressed()\0"
    "on_normFactor_returnPressed()\0"
    "on_fusionFactor_returnPressed()\0"
    "on_serialPollingRate_returnPressed()\0"
    "on_maxNucThreshold_returnPressed()\0"
    "on_minTemp_returnPressed()\0"
    "on_maxTemp_returnPressed()\0"
};

void MainWindow_streamer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow_streamer *_t = static_cast<MainWindow_streamer *>(_o);
        switch (_id) {
        case 0: _t->on_debugMode_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_verboseMode_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_output8bit_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_output16bit_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_outputColor_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_autoTemperature_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_undistortImages_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_normMode_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_mapCode_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_inputDatatype_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_detectorMode_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_usbMode_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_maxReadAttempts_returnPressed(); break;
        case 13: _t->on_desiredDegreesPerGraylevel_returnPressed(); break;
        case 14: _t->on_maxNucInterval_returnPressed(); break;
        case 15: _t->on_zeroDegreesOffset_returnPressed(); break;
        case 16: _t->on_degreesPerGraylevel_returnPressed(); break;
        case 17: _t->on_framerate_returnPressed(); break;
        case 18: _t->on_threshFactor_returnPressed(); break;
        case 19: _t->on_normFactor_returnPressed(); break;
        case 20: _t->on_fusionFactor_returnPressed(); break;
        case 21: _t->on_serialPollingRate_returnPressed(); break;
        case 22: _t->on_maxNucThreshold_returnPressed(); break;
        case 23: _t->on_minTemp_returnPressed(); break;
        case 24: _t->on_maxTemp_returnPressed(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow_streamer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow_streamer::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow_streamer,
      qt_meta_data_MainWindow_streamer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow_streamer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow_streamer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow_streamer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow_streamer))
        return static_cast<void*>(const_cast< MainWindow_streamer*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow_streamer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
