/******************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the Qt Quick Compiler.
**
** $QT_BEGIN_LICENSE:COMM$
**
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** $QT_END_LICENSE$
**
******************************************************************************/

#include <private/qv4function_p.h>
#include <private/qqmlirbuilder_p.h>
#include <private/qqmlglobal_p.h>
#include <private/qqmldirparser_p.h>
#include <QDir>
#include <QFileInfo>
#include <qqmlprivate.h>

QT_BEGIN_NAMESPACE
namespace QmlIR {
    struct IRLoader {
        inline PoolList<Property> *properties(Object *o) { return o->properties; }
        inline PoolList<Alias> *aliases(Object *o) { return o->aliases; }
        inline PoolList<Signal> *qmlSignals(Object *o) { return o->qmlSignals; }
        inline PoolList<Binding> *bindings(Object *o) { return o->bindings; }
        inline PoolList<Function> *functions(Object *o) { return o->functions; }
    };
}
QT_END_NAMESPACE

namespace {
    struct Loader : public QmlIR::IRLoader
    {
        Loader(const QV4::CompiledData::Unit *unit, QmlIR::Document *output);

        void load();

    private:
        QmlIR::Object *loadObject(const QV4::CompiledData::Object *serializedObject);

        template <typename _Tp> _Tp *New() { return pool->New<_Tp>(); }

        const QV4::CompiledData::Unit *unit;
        QmlIR::Document *output;
        QQmlJS::MemoryPool *pool;
    };

    Loader::Loader(const QV4::CompiledData::Unit *qmlData, QmlIR::Document *output)
        : unit(qmlData)
        , output(output)
    {
        pool = output->jsParserEngine.pool();
    }

    void Loader::load()
    {
        output->jsGenerator.stringTable.clear();
        for (uint i = 0; i < unit->stringTableSize; ++i)
            output->jsGenerator.stringTable.registerString(unit->stringAt(i));

        for (quint32 i = 0; i < unit->nImports; ++i)
            output->imports << unit->importAt(i);

        if (unit->flags & QV4::CompiledData::Unit::IsSingleton) {
            QmlIR::Pragma *p = New<QmlIR::Pragma>();
            p->location = QV4::CompiledData::Location();
            p->type = QmlIR::Pragma::PragmaSingleton;
            output->pragmas << p;
        }

        output->indexOfRootObject = unit->indexOfRootObject;

        for (uint i = 0; i < unit->nObjects; ++i) {
            const QV4::CompiledData::Object *serializedObject = unit->objectAt(i);
            QmlIR::Object *object = loadObject(serializedObject);
            output->objects.append(object);
        }
    }

    struct FakeExpression : public QQmlJS::AST::NullExpression
    {
        FakeExpression(int start, int length)
            : location(start, length)
        {}

        virtual QQmlJS::AST::SourceLocation firstSourceLocation() const
        { return location; }

        virtual QQmlJS::AST::SourceLocation lastSourceLocation() const
        { return location; }

    private:
        QQmlJS::AST::SourceLocation location;
    };

    QmlIR::Object *Loader::loadObject(const QV4::CompiledData::Object *serializedObject)
    {
        QmlIR::Object *object = pool->New<QmlIR::Object>();
        object->init(pool, serializedObject->inheritedTypeNameIndex, serializedObject->idNameIndex);

        object->indexOfDefaultPropertyOrAlias = serializedObject->indexOfDefaultPropertyOrAlias;
        object->defaultPropertyIsAlias = serializedObject->defaultPropertyIsAlias;
        object->flags = serializedObject->flags;
        object->id = serializedObject->id;
        object->location = serializedObject->location;
        object->locationOfIdProperty = serializedObject->locationOfIdProperty;

        QVector<int> functionIndices;
        functionIndices.reserve(serializedObject->nFunctions + serializedObject->nBindings / 2);

        for (uint i = 0; i < serializedObject->nBindings; ++i) {
            QmlIR::Binding *b = pool->New<QmlIR::Binding>();
            *static_cast<QV4::CompiledData::Binding*>(b) = serializedObject->bindingTable()[i];
            bindings(object)->append(b);
            if (b->type == QV4::CompiledData::Binding::Type_Script) {
                functionIndices.append(b->value.compiledScriptIndex);
                b->value.compiledScriptIndex = functionIndices.count() - 1;

                QmlIR::CompiledFunctionOrExpression *foe = pool->New<QmlIR::CompiledFunctionOrExpression>();
                foe->disableAcceleratedLookups = true;
                foe->nameIndex = 0;

                QQmlJS::AST::ExpressionNode *expr;

                if (b->stringIndex != quint32(0)) {
                    const int start = output->code.length();
                    const QString script = output->stringAt(b->stringIndex);
                    const int length = script.length();
                    output->code.append(script);
                    expr = new (pool) FakeExpression(start, length);
                } else
                    expr = new (pool) QQmlJS::AST::NullExpression();
                foe->node = new (pool) QQmlJS::AST::ExpressionStatement(expr); // dummy
                object->functionsAndExpressions->append(foe);
            }
        }

        Q_ASSERT(object->functionsAndExpressions->count == functionIndices.count());

        for (uint i = 0; i < serializedObject->nSignals; ++i) {
            const QV4::CompiledData::Signal *serializedSignal = serializedObject->signalAt(i);
            QmlIR::Signal *s = pool->New<QmlIR::Signal>();
            s->nameIndex = serializedSignal->nameIndex;
            s->location = serializedSignal->location;
            s->parameters = pool->New<QmlIR::PoolList<QmlIR::SignalParameter> >();

            for (uint i = 0; i < serializedSignal->nParameters; ++i) {
                QmlIR::SignalParameter *p = pool->New<QmlIR::SignalParameter>();
                *static_cast<QV4::CompiledData::Parameter*>(p) = *serializedSignal->parameterAt(i);
                s->parameters->append(p);
            }

            qmlSignals(object)->append(s);
        }

        {
            const QV4::CompiledData::Property *serializedProperty = serializedObject->propertyTable();
            for (uint i = 0; i < serializedObject->nProperties; ++i, ++serializedProperty) {
                QmlIR::Property *p = pool->New<QmlIR::Property>();
                *static_cast<QV4::CompiledData::Property*>(p) = *serializedProperty;
                properties(object)->append(p);
            }
        }

        {
            const QV4::CompiledData::Alias *serializedAlias = serializedObject->aliasTable();
            for (uint i = 0; i < serializedObject->nAliases; ++i, ++serializedAlias) {
                QmlIR::Alias *a = pool->New<QmlIR::Alias>();
                *static_cast<QV4::CompiledData::Alias*>(a) = *serializedAlias;
                aliases(object)->append(a);
            }
        }

        QQmlJS::Engine *jsParserEngine = &output->jsParserEngine;

        const QV4::CompiledData::LEUInt32 *functionIdx = serializedObject->functionOffsetTable();
        for (uint i = 0; i < serializedObject->nFunctions; ++i, ++functionIdx) {
            QmlIR::Function *f = pool->New<QmlIR::Function>();
            const QV4::CompiledData::Function *compiledFunction = unit->functionAt(*functionIdx);

            functionIndices.append(*functionIdx);
            f->index = functionIndices.count() - 1;
            f->location = compiledFunction->location;
            f->nameIndex = compiledFunction->nameIndex;

            QQmlJS::AST::FormalParameterList *paramList = 0;
            const QV4::CompiledData::LEUInt32 *formalNameIdx = compiledFunction->formalsTable();
            for (uint i = 0; i < compiledFunction->nFormals; ++i, ++formalNameIdx) {
                const QString formal = unit->stringAt(*formalNameIdx);
                QStringRef paramNameRef = jsParserEngine->newStringRef(formal);

                if (paramList)
                    paramList = new (pool) QQmlJS::AST::FormalParameterList(paramList, paramNameRef);
                else
                    paramList = new (pool) QQmlJS::AST::FormalParameterList(paramNameRef);
            }

            if (paramList)
                paramList = paramList->finish();

            const QString name = unit->stringAt(compiledFunction->nameIndex);
            f->functionDeclaration = new(pool) QQmlJS::AST::FunctionDeclaration(jsParserEngine->newStringRef(name), paramList, /*body*/0);

            f->formals.allocate(pool, int(compiledFunction->nFormals));
            formalNameIdx = compiledFunction->formalsTable();
            for (uint i = 0; i < compiledFunction->nFormals; ++i, ++formalNameIdx)
                f->formals[i] = *formalNameIdx;

            functions(object)->append(f);
        }

        object->runtimeFunctionIndices.allocate(pool, functionIndices);

        return object;
    }

struct AOTCompilationUnit : public QV4::CompiledData::CompilationUnit
{
    virtual ~AOTCompilationUnit();
    virtual void linkBackendToEngine(QV4::ExecutionEngine *engine);

    virtual QV4::CompiledData::Unit *createUnitData(QmlIR::Document *irDocument);

    QV4::ReturnedValue (**functionTable)(QV4::ExecutionEngine*);
};

AOTCompilationUnit::~AOTCompilationUnit()
{
}

void AOTCompilationUnit::linkBackendToEngine(QV4::ExecutionEngine *engine)
{
    runtimeFunctions.resize(data->functionTableSize);
    runtimeFunctions.fill(0);
    for (int i = 0 ;i < runtimeFunctions.size(); ++i) {
        const QV4::CompiledData::Function *compiledFunction = data->functionAt(i);

        QV4::Function *runtimeFunction = new QV4::Function(engine, this, compiledFunction,
                                                           (QV4::ReturnedValue (*)(QV4::ExecutionEngine *, const uchar*)) functionTable[i]);
        runtimeFunctions[i] = runtimeFunction;
    }
}

QV4::CompiledData::Unit *AOTCompilationUnit::createUnitData(QmlIR::Document *irDocument)
{
    Q_ASSERT(irDocument->javaScriptCompilationUnit);
    QQmlRefPointer<QV4::CompiledData::CompilationUnit> compilationUnit = irDocument->javaScriptCompilationUnit;
    QV4::CompiledData::Unit *jsUnit = const_cast<QV4::CompiledData::Unit*>(irDocument->javaScriptCompilationUnit->data);

    QV4::Compiler::StringTableGenerator &stringTable = irDocument->jsGenerator.stringTable;

    // Collect signals that have had a change in signature (from onClicked to onClicked(mouse) for example)
    // and now need fixing in the QV4::CompiledData. Also register strings at the same time, to finalize
    // the string table.
    QVector<quint32> changedSignals;
    QVector<QQmlJS::AST::FormalParameterList*> changedSignalParameters;
    for (int i = 0, count = irDocument->objects.count(); i < count; ++i) {
        QmlIR::Object *o = irDocument->objects.at(i);
        for (QmlIR::Binding *binding = o->firstBinding(); binding; binding = binding->next) {
            if (!(binding->flags & QV4::CompiledData::Binding::IsSignalHandlerExpression))
                continue;

            quint32 functionIndex = binding->value.compiledScriptIndex;
            QmlIR::CompiledFunctionOrExpression *foe = o->functionsAndExpressions->slowAt(functionIndex);
            if (!foe)
                continue;

            // save absolute index
            changedSignals << o->runtimeFunctionIndices.at(functionIndex);

            Q_ASSERT(foe->node);
            Q_ASSERT(QQmlJS::AST::cast<QQmlJS::AST::FunctionDeclaration*>(foe->node));

            QQmlJS::AST::FormalParameterList *parameters = QQmlJS::AST::cast<QQmlJS::AST::FunctionDeclaration*>(foe->node)->formals;
            changedSignalParameters << parameters;

            for (; parameters; parameters = parameters->next)
                stringTable.registerString(parameters->name.toString());
        }
    }

    QVector<quint32> signalParameterNameTable;
    quint32 signalParameterNameTableOffset = jsUnit->unitSize;

    // Update signal signatures
    if (!changedSignals.isEmpty()) {
        if (jsUnit == compilationUnit->data) {
            char *unitCopy = (char*)malloc(jsUnit->unitSize);
            memcpy(unitCopy, jsUnit, jsUnit->unitSize);
            jsUnit = reinterpret_cast<QV4::CompiledData::Unit*>(unitCopy);
        }

        for (int i = 0; i < changedSignals.count(); ++i) {
            const uint functionIndex = changedSignals.at(i);
            // The data is now read-write due to the copy above, so the const_cast is ok.
            QV4::CompiledData::Function *function = const_cast<QV4::CompiledData::Function *>(jsUnit->functionAt(functionIndex));
            Q_ASSERT(function->nFormals == quint32(0));

            function->formalsOffset = signalParameterNameTableOffset - jsUnit->functionOffsetTable()[functionIndex];

            for (QQmlJS::AST::FormalParameterList *parameters = changedSignalParameters.at(i);
                 parameters; parameters = parameters->next) {
                signalParameterNameTable.append(stringTable.getStringId(parameters->name.toString()));
                function->nFormals = function->nFormals + 1;
            }

            // Hack to ensure an activation is created.
            function->flags |= QV4::CompiledData::Function::HasCatchOrWith | QV4::CompiledData::Function::HasDirectEval;

            signalParameterNameTableOffset += function->nFormals * sizeof(quint32);
        }
    }

    if (!signalParameterNameTable.isEmpty()) {
        Q_ASSERT(jsUnit != compilationUnit->data);
        const uint signalParameterTableSize = signalParameterNameTable.count() * sizeof(quint32);
        uint newSize = jsUnit->unitSize + signalParameterTableSize;
        const uint oldSize = jsUnit->unitSize;
        char *unitWithSignalParameters = (char*)realloc(jsUnit, newSize);
        memcpy(unitWithSignalParameters + oldSize, signalParameterNameTable.constData(), signalParameterTableSize);
        jsUnit = reinterpret_cast<QV4::CompiledData::Unit*>(unitWithSignalParameters);
        jsUnit->unitSize = newSize;
    }

    if (jsUnit != compilationUnit->data)
        jsUnit->flags &= ~QV4::CompiledData::Unit::StaticData;

    return jsUnit;
}

struct Registry {
    Registry();

    QMutex mutex;

    QHash<QString, const QQmlPrivate::CachedQmlUnit*> resourcePathToCachedUnit;

    static const QQmlPrivate::CachedQmlUnit *lookupCachedUnit(const QUrl &url);
};

Q_GLOBAL_STATIC(Registry, registry)

Registry::Registry()
{
    QQmlPrivate::RegisterQmlUnitCacheHook registration;
    registration.version = 0;
    registration.lookupCachedQmlUnit = &lookupCachedUnit;
    QQmlPrivate::qmlregister(QQmlPrivate::QmlUnitCacheHookRegistration, &registration);
}

const QQmlPrivate::CachedQmlUnit *Registry::lookupCachedUnit(const QUrl &url)
{
    QString resourcePath;
    if (url.scheme() == QLatin1String("qrc"))
        resourcePath = url.path();

    resourcePath = QDir::cleanPath(resourcePath);

    if (resourcePath.isEmpty())
        return 0;

    Registry *r = registry();
    QMutexLocker locker(&r->mutex);
    return r->resourcePathToCachedUnit.value(resourcePath, 0);
}

QV4::CompiledData::CompilationUnit *createCompilationUnit(const QV4::CompiledData::Unit *unitData, QV4::ReturnedValue (**functions)(QV4::ExecutionEngine *))
{
    AOTCompilationUnit *unit = new AOTCompilationUnit;
    unit->data = const_cast<QV4::CompiledData::Unit*>(unitData);
    unit->functionTable = functions;
    return unit;
}

void loadQmlIR(QmlIR::Document *document, const QQmlPrivate::CachedQmlUnit *cachedQmlUnit)
{
    Loader loader(cachedQmlUnit->qmlData, document);
    loader.load();
    document->javaScriptCompilationUnit.adopt(cachedQmlUnit->createCompilationUnit());
}

} // anonymous namespace
#include <qqmlprivate.h>

static const unsigned char qt_resource_tree[] = {
0,
0,0,0,0,2,0,0,0,3,0,0,0,1,0,0,0,
8,0,2,0,0,0,1,0,0,0,6,0,0,0,120,0,
0,0,0,0,1,0,0,0,0,0,0,0,66,0,2,0,
0,0,1,0,0,0,4,0,0,0,66,0,2,0,0,0,
1,0,0,0,5,0,0,0,86,0,0,0,0,0,1,0,
0,0,0,0,0,0,22,0,2,0,0,0,1,0,0,0,
7,0,0,0,36,0,0,0,0,0,1,0,0,0,0};
static const unsigned char qt_resource_names[] = {
0,
1,0,0,0,47,0,47,0,4,0,5,60,85,0,77,0,
101,0,110,0,117,0,4,0,5,207,199,0,86,0,105,0,
101,0,119,0,12,2,40,82,92,0,77,0,97,0,105,0,
110,0,77,0,101,0,110,0,117,0,46,0,113,0,109,0,
108,0,7,10,202,182,195,0,67,0,117,0,115,0,116,0,
111,0,109,0,115,0,14,4,18,125,188,0,77,0,101,0,
110,0,117,0,66,0,117,0,116,0,116,0,111,0,110,0,
46,0,113,0,109,0,108,0,8,8,1,90,92,0,109,0,
97,0,105,0,110,0,46,0,113,0,109,0,108};
static const unsigned char qt_resource_empty_payout[] = { 0, 0, 0, 0, 0 };
QT_BEGIN_NAMESPACE
extern Q_CORE_EXPORT bool qRegisterResourceData(int, const unsigned char *, const unsigned char *, const unsigned char *);
QT_END_NAMESPACE
typedef QV4::ReturnedValue (*AOTFunction)(QV4::ExecutionEngine*);
namespace QtQuickCompilerGeneratedModule
{
    namespace __main_qml
    {
        extern const unsigned char qmlData[];
        extern AOTFunction moduleFunctions[];
        QV4::CompiledData::CompilationUnit *createCompilationUnit() { return ::createCompilationUnit(reinterpret_cast<const QV4::CompiledData::Unit*>(qmlData), moduleFunctions); }
        const QQmlPrivate::CachedQmlUnit unit = {
            reinterpret_cast<const QV4::CompiledData::Unit*>(qmlData), &createCompilationUnit, &::loadQmlIR
        };

}
namespace _Customs_Customs_MenuButton_qml
{
        extern const unsigned char qmlData[];
        extern AOTFunction moduleFunctions[];
        QV4::CompiledData::CompilationUnit *createCompilationUnit() { return ::createCompilationUnit(reinterpret_cast<const QV4::CompiledData::Unit*>(qmlData), moduleFunctions); }
        const QQmlPrivate::CachedQmlUnit unit = {
            reinterpret_cast<const QV4::CompiledData::Unit*>(qmlData), &createCompilationUnit, &::loadQmlIR
        };

}
namespace _Menu_View_MainMenu_qml
{
        extern const unsigned char qmlData[];
        extern AOTFunction moduleFunctions[];
        QV4::CompiledData::CompilationUnit *createCompilationUnit() { return ::createCompilationUnit(reinterpret_cast<const QV4::CompiledData::Unit*>(qmlData), moduleFunctions); }
        const QQmlPrivate::CachedQmlUnit unit = {
            reinterpret_cast<const QV4::CompiledData::Unit*>(qmlData), &createCompilationUnit, &::loadQmlIR
        };

}
}
static void registerCompilationUnits(){
    static bool initialized = false;
    if (initialized) return;
    initialized = true;
    ::Registry *r = registry();
    QMutexLocker locker(&r->mutex);
    r->resourcePathToCachedUnit.insert(QStringLiteral("/main.qml"), &QtQuickCompilerGeneratedModule::__main_qml::unit);
    r->resourcePathToCachedUnit.insert(QStringLiteral("/Customs/Customs/MenuButton.qml"), &QtQuickCompilerGeneratedModule::_Customs_Customs_MenuButton_qml::unit);
    r->resourcePathToCachedUnit.insert(QStringLiteral("/Menu/View/MainMenu.qml"), &QtQuickCompilerGeneratedModule::_Menu_View_MainMenu_qml::unit);
QT_PREPEND_NAMESPACE(qRegisterResourceData)(/*version*/0x01, qt_resource_tree, qt_resource_names, qt_resource_empty_payout);

}
Q_CONSTRUCTOR_FUNCTION(registerCompilationUnits);
int QT_MANGLE_NAMESPACE(qInitResources_qml)() {
    ::registerCompilationUnits();
    Q_INIT_RESOURCE(qml_qtquickcompiler);
    return 1;
}
