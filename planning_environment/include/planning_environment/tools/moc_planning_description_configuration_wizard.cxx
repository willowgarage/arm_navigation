/****************************************************************************
** Meta object code from reading C++ file 'planning_description_configuration_wizard.h'
**
** Created: Mon Jun 20 13:19:29 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "planning_description_configuration_wizard.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'planning_description_configuration_wizard.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PlanningDescriptionConfigurationWizard[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      44,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      49,   40,   39,   39, 0x05,
      74,   69,   39,   39, 0x05,

 // slots: signature, parameters, type, tag, flags
     122,  116,   99,   39, 0x0a,
     164,  153,   39,   39, 0x0a,
     188,  153,   39,   39, 0x0a,
     212,   69,   39,   39, 0x0a,
     238,  153,   39,   39, 0x0a,
     266,  153,   39,   39, 0x0a,
     290,  153,   39,   39, 0x0a,
     316,  153,   39,   39, 0x0a,
     340,  153,   39,   39, 0x0a,
     381,  368,   39,   39, 0x0a,
     412,  116,   39,   39, 0x2a,
     439,   39,   39,   39, 0x0a,
     461,   39,   39,   39, 0x0a,
     481,   39,   39,   39, 0x0a,
     504,   39,   39,   39, 0x0a,
     531,   39,   39,   39, 0x0a,
     549,   39,   39,   39, 0x0a,
     576,   39,   39,   39, 0x0a,
     605,   39,   39,   39, 0x0a,
     632,   39,   39,   39, 0x0a,
     653,   39,   39,   39, 0x0a,
     674,   39,   39,   39, 0x0a,
     694,   39,   39,   39, 0x0a,
     713,   39,   39,   39, 0x0a,
     731,   39,   39,   39, 0x0a,
     751,   39,   39,   39, 0x0a,
     773,   39,   39,   39, 0x0a,
     793,   39,   39,   39, 0x0a,
     816,   39,   39,   39, 0x0a,
     843,   39,   39,   39, 0x0a,
     870,   39,   39,   39, 0x0a,
     899,   39,   39,   39, 0x0a,
     930,   39,   39,   39, 0x0a,
     966,   39,   39,   39, 0x0a,
    1005,   39,   39,   39, 0x0a,
    1037,   39,   39,   39, 0x0a,
    1065,   39,   39,   39, 0x0a,
    1098,   39,   39,   39, 0x0a,
    1137, 1132,   39,   39, 0x0a,
    1159,   39,   39,   39, 0x0a,
    1172,   39,   39,   39, 0x0a,
    1188,   39,   39,   39, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PlanningDescriptionConfigurationWizard[] = {
    "PlanningDescriptionConfigurationWizard\0"
    "\0progress\0changeProgress(int)\0name\0"
    "changeLabel(const char*)\0std::vector<int>\0"
    "table\0getSelectedRows(QTableWidget*)\0"
    "checkState\0easyButtonToggled(bool)\0"
    "hardButtonToggled(bool)\0"
    "labelChanged(const char*)\0"
    "verySafeButtonToggled(bool)\0"
    "safeButtonToggled(bool)\0"
    "normalButtonToggled(bool)\0"
    "fastButtonToggled(bool)\0"
    "veryFastButtonToggled(bool)\0table,column\0"
    "toggleTable(QTableWidget*,int)\0"
    "toggleTable(QTableWidget*)\0"
    "defaultTogglePushed()\0oftenTogglePushed()\0"
    "adjacentTogglePushed()\0"
    "occasionallyTogglePushed()\0dofTogglePushed()\0"
    "selectJointButtonClicked()\0"
    "deselectJointButtonClicked()\0"
    "deleteGroupButtonClicked()\0"
    "acceptChainClicked()\0acceptGroupClicked()\0"
    "baseLinkTreeClick()\0tipLinkTreeClick()\0"
    "validateDoneBox()\0groupTableClicked()\0"
    "defaultTableClicked()\0oftenTableClicked()\0"
    "adjacentTableChanged()\0"
    "occasionallyTableClicked()\0"
    "dofSelectionTableChanged()\0"
    "oftenCollisionTableChanged()\0"
    "defaultCollisionTableChanged()\0"
    "occasionallyCollisionTableChanged()\0"
    "generateOccasionallyInCollisionTable()\0"
    "generateOftenInCollisionTable()\0"
    "generateAdjacentLinkTable()\0"
    "generateAlwaysInCollisionTable()\0"
    "generateDefaultInCollisionTable()\0"
    "file\0fileSelected(QString)\0writeFiles()\0"
    "autoConfigure()\0update()\0"
};

const QMetaObject PlanningDescriptionConfigurationWizard::staticMetaObject = {
    { &QWizard::staticMetaObject, qt_meta_stringdata_PlanningDescriptionConfigurationWizard,
      qt_meta_data_PlanningDescriptionConfigurationWizard, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PlanningDescriptionConfigurationWizard::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PlanningDescriptionConfigurationWizard::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PlanningDescriptionConfigurationWizard::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PlanningDescriptionConfigurationWizard))
        return static_cast<void*>(const_cast< PlanningDescriptionConfigurationWizard*>(this));
    return QWizard::qt_metacast(_clname);
}

int PlanningDescriptionConfigurationWizard::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWizard::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: changeProgress((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: changeLabel((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 2: { std::vector<int> _r = getSelectedRows((*reinterpret_cast< QTableWidget*(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::vector<int>*>(_a[0]) = _r; }  break;
        case 3: easyButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: hardButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: labelChanged((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 6: verySafeButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: safeButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: normalButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: fastButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: veryFastButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: toggleTable((*reinterpret_cast< QTableWidget*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 12: toggleTable((*reinterpret_cast< QTableWidget*(*)>(_a[1]))); break;
        case 13: defaultTogglePushed(); break;
        case 14: oftenTogglePushed(); break;
        case 15: adjacentTogglePushed(); break;
        case 16: occasionallyTogglePushed(); break;
        case 17: dofTogglePushed(); break;
        case 18: selectJointButtonClicked(); break;
        case 19: deselectJointButtonClicked(); break;
        case 20: deleteGroupButtonClicked(); break;
        case 21: acceptChainClicked(); break;
        case 22: acceptGroupClicked(); break;
        case 23: baseLinkTreeClick(); break;
        case 24: tipLinkTreeClick(); break;
        case 25: validateDoneBox(); break;
        case 26: groupTableClicked(); break;
        case 27: defaultTableClicked(); break;
        case 28: oftenTableClicked(); break;
        case 29: adjacentTableChanged(); break;
        case 30: occasionallyTableClicked(); break;
        case 31: dofSelectionTableChanged(); break;
        case 32: oftenCollisionTableChanged(); break;
        case 33: defaultCollisionTableChanged(); break;
        case 34: occasionallyCollisionTableChanged(); break;
        case 35: generateOccasionallyInCollisionTable(); break;
        case 36: generateOftenInCollisionTable(); break;
        case 37: generateAdjacentLinkTable(); break;
        case 38: generateAlwaysInCollisionTable(); break;
        case 39: generateDefaultInCollisionTable(); break;
        case 40: fileSelected((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 41: writeFiles(); break;
        case 42: autoConfigure(); break;
        case 43: update(); break;
        default: ;
        }
        _id -= 44;
    }
    return _id;
}

// SIGNAL 0
void PlanningDescriptionConfigurationWizard::changeProgress(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PlanningDescriptionConfigurationWizard::changeLabel(const char * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
