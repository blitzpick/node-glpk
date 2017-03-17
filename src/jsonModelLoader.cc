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
typedef map<string, V8Value> ValueMap;
typedef vector<ValueMap> VariableList;
typedef map<string, int> IntMap;
typedef vector<V8Value> Vector;

static clock_t start = clock();
static void logTime(const char * message) {
    clock_t now = clock();
    printf("%s: %lf\n", message, (now - start) / 1000.0);
    start = now;
}

static void checkAndThrow(bool condition, const char *message) {
    if (condition) {
        Nan::ThrowTypeError(message);
        throw message;
    }
}

static double toDouble(V8Value value) {
    return value->NumberValue();
}

static string toString(V8Value value) {
    return string(V8TOCSTRING(value));
}

static ValueMap toMap(V8Value value) {
    ValueMap properties;

    V8Object obj = V8Object::Cast(value);
    V8Array propertyNames = obj->GetPropertyNames();
    for (uint32_t i = 0; i < propertyNames->Length(); i++) {
        V8Value name = propertyNames->Get(i);
        V8Value value = obj->Get(name);

        properties[string(V8TOCSTRING(name))] = value;
    }

    return properties;
}

static Vector toVector(V8Value value) {
    V8Array array = V8Array::Cast(value);
    Vector values;

    for (uint32_t i = 0; i < array->Length(); i++) {
        V8Value value = array->Get(i);
        values.push_back(value);
    }

    return values;
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
        return make_pair(0.0, toDouble(operand));
    }

    if (operation == "range") {
        Vector range = toVector(operand);
        return make_pair(toDouble(range[0]), toDouble(range[1]));
    }

    if (operation == "lower") {
        return make_pair(toDouble(operand), 0.0);
    }

    if (operation == "upper") {
        return make_pair(0.0, toDouble(operand));
    }

    if (operation == "equal") {
        double value = toDouble(operand);
        return make_pair(value, value);
    }

    return make_pair(0.0, 0.0);
}

static int getVariableKind(ValueMap variable) {
    string kind = toString(variable["kind"]);

    if (kind == "binary") {
        return GLP_BV;
    }

    if (kind == "integer") {
        return GLP_IV;
    }

    return GLP_CV;
}

static void populateMatrix(glp_prob *problem, VariableList variables, IntMap constraintIndices) {
    vector<int> rowIndices(1);
    vector<int> columnIndices(1);
    vector<double> values(1);

    for (size_t j = 0; j < variables.size(); ++j) {
        ValueMap props = variables[j];
        ValueMap::iterator it;
        for (it = props.begin(); it != props.end(); ++it) {
            string name = it->first;
            IntMap::iterator constraint = constraintIndices.find(name);
            if (constraint != constraintIndices.end()) {
                int i = constraint->second;
                rowIndices.push_back(i);
                columnIndices.push_back(j + 1);
                values.push_back(toDouble(it->second));
            }
        }
    }
    glp_load_matrix(problem, values.size() - 1, rowIndices.data(), columnIndices.data(), values.data());
}

static VariableList addVariables(glp_prob *problem, ValueMap model) {
    string objective = toString(model["objective"]);

    ValueMap variables = toMap(model["variables"]);
    glp_add_cols(problem, variables.size());

    VariableList variableList;

    size_t i = 1;
    ValueMap::iterator it;
    for (it = variables.begin(); it != variables.end(); ++it, ++i) {
        string name = it->first;
        ValueMap variable = toMap(it->second);

        glp_set_col_name(problem, i, name.c_str());
        glp_set_col_kind(problem, i, getVariableKind(variable));

        ValueMap values = toMap(variable["values"]);
        variableList.push_back(values);
        glp_set_obj_coef(problem, i, toDouble(values[objective]));
    }

    return variableList;
}

static IntMap addConstraints(glp_prob *problem, ValueMap model) {
    ValueMap constraints = toMap(model["constraints"]);
    glp_add_rows(problem, constraints.size());

    IntMap constraintIndices;

    size_t i = 1;
    ValueMap::iterator it;
    for (it = constraints.begin(); it != constraints.end(); ++it, ++i) {
        string name = it->first;
        constraintIndices[name] = i;

        glp_set_row_name(problem, i, name.c_str());

        ValueMap constraint = toMap(it->second);
        checkAndThrow(constraint.size() != 1, "Constraints may contain only a single operation.");

        ValueMap::iterator onlyProperty = constraint.begin();
        string operation = onlyProperty->first;
        V8Value operand = onlyProperty->second;

        int type = getConstraintType(operation);
        pair<double, double> bounds = getConstraintBounds(operation, operand);
        glp_set_row_bnds(problem, i, type, bounds.first, bounds.second);
    }

    return constraintIndices;
}

static int getDirection(ValueMap model) {
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
        logTime("start");
        ValueMap model = toMap(modelObj);
        logTime("model toMap");

        glp_set_prob_name(problem, toString(model["name"]).c_str());
        glp_set_obj_dir(problem, getDirection(model));

        logTime("before addConstraints");
        IntMap constraintIndices = addConstraints(problem, model);
        logTime("added constraints");
        VariableList variables = addVariables(problem, model);
        logTime("added variables");
        populateMatrix(problem, variables, constraintIndices);
        logTime("populated matrix");
    }
}
