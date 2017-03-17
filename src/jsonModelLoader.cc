#include <node.h>
#include <node_object_wrap.h>
#include <string>
#include <map>
#include <vector>

#include "glpk/glpk.h"
#include "common.h"
#include "jsonModelLoader.h"

using namespace std;
typedef v8::String String;
typedef v8::Local<v8::Value> V8Value;
typedef v8::Local<v8::Object> V8Object;
typedef v8::Local<v8::Array> V8Array;

static V8Value fromString(const char *str) {
    return Nan::New<String>(str).ToLocalChecked();
}

class ArrayWrapper {
public:
    ArrayWrapper(V8Value value) {
        this->array = V8Array::Cast(array);
    }

    ArrayWrapper(V8Array array) {
        this->array = array;
    }

    V8Value operator [](int i) {
        return this->array->Get(i);
    }

private:
    V8Array array;
};

class ObjectWrapper {
public:
    ObjectWrapper(V8Value value) {
        this->obj = V8Object::Cast(value);
    }

    ObjectWrapper(V8Object obj) {
        this->obj = obj;
    }

    V8Value operator [](V8Value key) {
        return this->obj->Get(key);
    }

    V8Value operator [](const char *key) {
        return this->obj->Get(fromString(key));
    }

    V8Value operator [](string key) {
        return this->obj->Get(fromString(key.c_str()));
    }

    size_t size() {
        V8Array propertyNames = this->obj->GetPropertyNames();
        return propertyNames->Length();
    }

    V8Array getPropertyNames() {
        return this->obj->GetPropertyNames();
    }

    V8Value getPropertyName(size_t i) {
        return this->getPropertyNames()->Get(i);
    }

private:
    V8Object obj;
};

static void checkAndThrow(bool condition, const char *message) {
    if (condition) {
        throw message;
    }
}

static string toString(V8Value value) {
    return V8TOCSTRING(value);
}

static int getConstraintType(string operation) {
    if (operation == "max") {
        return GLP_DB;
    }

    if (operation == "range") {
        return GLP_DB;
    }

    if (operation == "lower") {
        return GLP_LO;
    }

    if (operation == "upper") {
        return GLP_UP;
    }

    if (operation == "equal") {
        return GLP_FX;
    }

    return GLP_FR;
}

static pair<double, double> getConstraintBounds(string operation, V8Value operand) {
    if (operation == "max") {
        return make_pair(0.0, operand->NumberValue());
    }

    if (operation == "range") {
        ArrayWrapper range(operand);
        return make_pair(range[0]->NumberValue(), range[1]->NumberValue());
    }

    if (operation == "lower") {
        return make_pair(operand->NumberValue(), 0.0);
    }

    if (operation == "upper") {
        return make_pair(0.0, operand->NumberValue());
    }

    if (operation == "equal") {
        double value = operand->NumberValue();
        return make_pair(value, value);
    }

    return make_pair(0.0, 0.0);
}

static int getVariableKind(ObjectWrapper variable) {
    string kind = toString(variable["kind"]);

    if (kind == "binary") {
        return GLP_BV;
    }

    if (kind == "integer") {
        return GLP_IV;
    }

    return GLP_CV;
}

static void addVariables(glp_prob *problem, ObjectWrapper model, map<string, int> constraintIndices) {
    vector<int> ia(1);
    vector<int> ja(1);
    vector<double> ar(1);

    V8Value objective = model["objective"];
    ObjectWrapper variables(model["variables"]);

    size_t numVariables = variables.size();
    glp_add_cols(problem, numVariables);

    ArrayWrapper propertyNames(variables.getPropertyNames());

    for (size_t i = 0; i < numVariables; ++i) {
        size_t index = i + 1;
        string name = toString(propertyNames[i]);
        ObjectWrapper variable(variables[name]);

        glp_set_col_name(problem, index, name.c_str());
        glp_set_col_kind(problem, index, getVariableKind(variable));

        ObjectWrapper values(variable["values"]);
        glp_set_obj_coef(problem, index, values[objective]->NumberValue());

        size_t numValues = values.size();
        ArrayWrapper valueNames(values.getPropertyNames());
        for (size_t i = 0; i < numValues; ++i) {
            V8Value name = valueNames[i];
            map<string, int>::iterator constraint = constraintIndices.find(toString(name));
            if (constraint != constraintIndices.end()) {
                ia.push_back(constraint->second);
                ja.push_back(index);
                ar.push_back(values[name]->NumberValue());
            }
        }
    }

    glp_load_matrix(problem, ar.size() - 1, ia.data(), ja.data(), ar.data());
}

static map<string, int> addConstraints(glp_prob *problem, ObjectWrapper model) {
    ObjectWrapper constraints(model["constraints"]);

    size_t size = constraints.size();
    glp_add_rows(problem, size);

    map<string, int> constraintIndices;
    ArrayWrapper propertyNames(constraints.getPropertyNames());

    for (size_t i = 0; i < size; ++i) {
        size_t index = i + 1;
        string name = toString(propertyNames[i]);
        constraintIndices[name] = index;

        glp_set_row_name(problem, index, name.c_str());

        ObjectWrapper constraint(constraints[name]);
        checkAndThrow(constraint.size() != 1, "Constraints may contain only a single operation.");

        V8Value propertyName = constraint.getPropertyName(0);
        string operation = toString(propertyName);
        V8Value operand = constraint[propertyName];

        int type = getConstraintType(operation);
        pair<double, double> bounds = getConstraintBounds(operation, operand);
        glp_set_row_bnds(problem, index, type, bounds.first, bounds.second);
    }

    return constraintIndices;
}

static int getDirection(ObjectWrapper model) {
    string direction = toString(model["direction"]);

    if (direction == "maximize") {
        return GLP_MAX;
    }

    if (direction == "minimize") {
        return GLP_MIN;
    }

    checkAndThrow(true, "'direction' must be either 'minimize' or 'maximize'");
    return GLP_MAX;
}

namespace NodeGLPK {
    void JsonModelLoader::Load(glp_prob *problem, V8Object modelObj) {
        try {
            ObjectWrapper model(modelObj);

            glp_set_prob_name(problem, toString(model["name"]).c_str());
            glp_set_obj_dir(problem, getDirection(model));

            map<string, int> constraintIndices = addConstraints(problem, model);
            addVariables(problem, model, constraintIndices);
        } catch (const char *msg) {
            Nan::ThrowTypeError(msg);
        } catch (string msg) {
            Nan::ThrowTypeError(msg.c_str());
        } catch (...) {
            Nan::ThrowTypeError("Unknown execption");
        }
    }
}
