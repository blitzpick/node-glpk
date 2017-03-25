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

static string toString(V8Value value) {
    return V8TOCSTRING(value);
}

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

    V8Value operator [](int index) {
        return this->obj->Get(index);
    }

    bool hasProperty(const char * key) {
        return this->obj->Has(fromString(key));
    }

    bool isUndefined() {
        return obj->IsUndefined();
    }

    bool isObject() {
        return obj->IsObject();
    }

    int size() {
        V8Array propertyNames = this->obj->GetPropertyNames();
        return propertyNames->Length();
    }

    V8Array getPropertyNames() {
        return this->obj->GetPropertyNames();
    }

    V8Value getPropertyName(int i) {
        return this->getPropertyNames()->Get(i);
    }

private:
    V8Object obj;
};

class Model {
public:
    Model(V8Object model) :
        modelWrapper(model),
        ia(1),
        ja(1),
        ar(1) {
    }

    V8Value operator[](const char *key) {
        return this->modelWrapper[key];
    }

    void addMatrixEntry(int i, int j, double value) {
        if (value == 0.0) {
            return;
        }

        this->ia.push_back(i);
        this->ja.push_back(j);
        this->ar.push_back(value);
    }

    void loadMatrixIntoProblem(glp_prob *problem) {
        glp_load_matrix(
            problem,
            this->ar.size() - 1,
            this->ia.data(),
            this->ja.data(),
            this->ar.data()
        );
    }

private:
    ObjectWrapper modelWrapper;
    vector<int> ia;
    vector<int> ja;
    vector<double> ar;
};

static void checkAndThrow(bool condition, string message) {
    if (condition) {
        throw message;
    }
}

static void setProblemName(glp_prob *problem, Model &model) {
    V8Value name = model["name"];
    if (name->IsString()) {
        glp_set_prob_name(problem, toString(name).c_str());
    }
}

static void setDirection(glp_prob *problem, Model &model) {
    const char * errorMessage = "The model's direction property must be either 'minimize' or 'maximize'";

    V8Value directionValue = model["direction"];
    checkAndThrow(!directionValue->IsString(), errorMessage);

    string direction = toString(directionValue);

    if (direction == "maximize") {
        glp_set_obj_dir(problem, GLP_MAX);
        return;
    }

    if (direction == "minimize") {
        glp_set_obj_dir(problem, GLP_MIN);
        return;
    }

    throw errorMessage;
}

static void configureConstraint(glp_prob *problem, ObjectWrapper &constraint, string name, int index) {
    glp_set_row_name(problem, index, name.c_str());

    if (constraint.hasProperty("range")) {
        V8Value range = constraint["range"];
        checkAndThrow(!range->IsArray(), "The range property of constraint '" + name + "' must be an array");
        ObjectWrapper rangeArray(range);
        checkAndThrow(rangeArray["length"]->Uint32Value() != 2, "The range property of constraint '" + name + "' must be an array of two numbers");
        V8Value lowerBoundValue = rangeArray[0];
        V8Value upperBoundValue = rangeArray[1];
        checkAndThrow(!lowerBoundValue->IsNumber() || !upperBoundValue->IsNumber(), "The range property of constraint '" + name + "' must be an array of two numbers");
        double lowerBound = lowerBoundValue->NumberValue();
        double upperBound = upperBoundValue->NumberValue();
        checkAndThrow(lowerBound >= upperBound, "The first number in the range property of constraint '" + name + "' must be lower than the second");
        glp_set_row_bnds(problem, index, GLP_DB, lowerBound, upperBound);
        return;
    }

    if (constraint.hasProperty("lower")) {
        V8Value lowerBound = constraint["lower"];
        checkAndThrow(!lowerBound->IsNumber(), "The lower property of constraint '" + name + "' must be a number");
        glp_set_row_bnds(problem, index, GLP_LO, lowerBound->NumberValue(), 0.0);
        return;
    }

    if (constraint.hasProperty("upper")) {
        V8Value upperBound = constraint["upper"];
        checkAndThrow(!upperBound->IsNumber(), "The upper property of constraint '" + name + "' must be a number");
        glp_set_row_bnds(problem, index, GLP_UP, 0.0, upperBound->NumberValue());
        return;
    }

    if (constraint.hasProperty("equal")) {
        V8Value numberValue = constraint["equal"];
        checkAndThrow(!numberValue->IsNumber(), "The equal property of constraint '" + name + "' must be a number");
        double number = numberValue->NumberValue();
        glp_set_row_bnds(problem, index, GLP_FX, number, number);
        return;
    }

    if (constraint.hasProperty("free")) {
        V8Value freeValue = constraint["free"];
        checkAndThrow(!freeValue->IsTrue(), "The free property of constraint '" + name + "' must be true");
        glp_set_row_bnds(problem, index, GLP_FR, 0.0, 0.0);
        return;
    }

    throw "The constraint '" + name + "' must contain an operation (one of 'range', 'lower', 'upper', 'equal', or 'free')";
}

static vector<string> getValidConstraintNames(ObjectWrapper &constraints) {
    vector<string> validConstraintNames;
    if (!constraints.isUndefined()) {
        checkAndThrow(!constraints.isObject(), "The 'constraints' property must be an object");
        ArrayWrapper constraintNames(constraints.getPropertyNames());
        int size = constraints.size();
        for (int i = 0; i < size; ++i) {
            string name = toString(constraintNames[i]);
            V8Value constraint = constraints[name];
            if (!constraint->IsUndefined()) {
                checkAndThrow(!constraint->IsObject(), "The constraint '" + name + "' is not an object");
                validConstraintNames.push_back(name);
            }
        }
    }

    return validConstraintNames;
}

static void addConstraints(glp_prob *problem, Model &model) {
    ObjectWrapper constraints(model["constraints"]);
    vector<string> constraintNames = getValidConstraintNames(constraints);
    int size = constraintNames.size();

    if (size == 0) {
        return;
    }

    glp_add_rows(problem, size);

    for (int i = 0; i < size; ++i) {
        string name = constraintNames[i];
        ObjectWrapper constraint(constraints[name]);

        configureConstraint(problem, constraint, name, i + 1);
    }
}

static int getVariableKind(ObjectWrapper &variable, string name) {
    V8Value kindValue = variable["kind"];
    checkAndThrow(kindValue->IsUndefined(), "The variable '" + name + "' does not have a kind property");
    checkAndThrow(!kindValue->IsString(), "The 'kind' property of the variable '" + name + "' must be a string");
    string kind = toString(kindValue);

    if (kind == "binary") {
        return GLP_BV;
    }

    if (kind == "integer") {
        return GLP_IV;
    }

    if (kind == "continuous") {
        return GLP_CV;
    }

    throw "The 'kind' property of the variable named '" + name + "' must be one of 'binary', 'integer', or 'continuous'";
}

static vector<string> getValidVariableNames(ObjectWrapper &variables) {
    vector<string> validVariableNames;

    if (!variables.isUndefined()) {
        checkAndThrow(!variables.isObject(), "The 'variables' property must be an object");
        ArrayWrapper variableNames(variables.getPropertyNames());
        int size = variables.size();
        for (int i = 0; i < size; ++i) {
            string name = toString(variableNames[i]);
            V8Value variable = variables[name];
            if (!variable->IsUndefined()) {
                checkAndThrow(!variable->IsObject(), "The variable '" + name + "' is not an object");
                validVariableNames.push_back(name);
            }
        }
    }

    return validVariableNames;
}

static void addVariables(glp_prob *problem, Model &model) {
    V8Value objectiveNameValue = model["objective"];
    checkAndThrow(objectiveNameValue->IsUndefined(), "You must specify an objective value name in the model");
    checkAndThrow(!objectiveNameValue->IsString(), "The model's objective property must be a string");
    string objectiveName = toString(objectiveNameValue);

    glp_set_obj_name(problem, objectiveName.c_str());

    ObjectWrapper variables(model["variables"]);
    vector<string> variableNames = getValidVariableNames(variables);
    int size = variableNames.size();

    if (size == 0) {
        return;
    }

    glp_add_cols(problem, size);

    for (int i = 0; i < size; ++i) {
        int variableIndex = i + 1;
        string variableName = variableNames[i];
        ObjectWrapper variable(variables[variableName]);

        glp_set_col_name(problem, variableIndex, variableName.c_str());
        glp_set_col_kind(problem, variableIndex, getVariableKind(variable, variableName));

        ObjectWrapper values(variable["values"]);
        checkAndThrow(!values.isObject(), "The 'values' property of the variable '" + variableName + "' must be an object");

        V8Value objectiveValue = values[objectiveName];
        checkAndThrow(!objectiveValue->IsNumber(), "The '" + objectiveName + "' property of the variable '" + variableName + "' must be a number");
        glp_set_obj_coef(problem, variableIndex, objectiveValue->NumberValue());

        int numValues = values.size();
        ArrayWrapper valueNames(values.getPropertyNames());
        for (int i = 0; i < numValues; ++i) {
            string name = toString(valueNames[i]);
            V8Value value = values[name];
            int constraintIndex = glp_find_row(problem, name.c_str());
            checkAndThrow(!value->IsNumber(), "The '" + name + "' property of the variable '" + variableName + "' must be a number");
            double number = value->NumberValue();
            if (number != 0 && constraintIndex != 0) {
                model.addMatrixEntry(constraintIndex, variableIndex, number);
            }
        }
    }
}

static vector<string> getExplicitConstraintNames(ObjectWrapper &constraints) {
    vector<string> explicitConstraintNames;

    vector<string> validConstraintNames = getValidConstraintNames(constraints);
    int size = validConstraintNames.size();

    for (int i = 0; i < size; ++i) {
        string name = validConstraintNames[i];
        ObjectWrapper constraint(constraints[name]);
        V8Value coefficients = constraint["coefficients"];
        if (coefficients->IsObject()) {
            explicitConstraintNames.push_back(name);
        }
    }

    return explicitConstraintNames;
}

static void addExplicitConstraintValues(glp_prob *problem, Model &model) {
    ObjectWrapper constraints(model["constraints"]);
    vector<string> explicitConstraintNames = getExplicitConstraintNames(constraints);
    int size = explicitConstraintNames.size();

    for (int i = 0; i < size; ++i) {
        string constraintName = explicitConstraintNames[i];
        int constraintIndex = glp_find_row(problem, constraintName.c_str());
        ObjectWrapper constraint(constraints[constraintName]);
        ObjectWrapper coefficients(constraint["coefficients"]);
        ArrayWrapper coefficientNames(coefficients.getPropertyNames());

        int numCoefficients = coefficients.size();
        for (int i = 0; i < numCoefficients; ++i) {
            string variableName = toString(coefficientNames[i]);
            int variableIndex = glp_find_col(problem, variableName.c_str());
            checkAndThrow(variableIndex == 0, "The constraint '" + constraintName + "' references a variable '" + variableName + "' which does not exist");

            V8Value coefficient = coefficients[variableName];
            checkAndThrow(!coefficient->IsNumber(), "The coefficient for variable '" + variableName + "' in the constraint '" + constraintName + "' must be a number");
            double number = coefficient->NumberValue();
            if (number != 0) {
                model.addMatrixEntry(constraintIndex, variableIndex, coefficient->NumberValue());
            }
        }
    }
}

namespace NodeGLPK {
    void JsonModelLoader::Load(glp_prob *problem, V8Object _model) {
        try {
            // Enable names for constraints and variables
            glp_create_index(problem);

            Model model(_model);
            setProblemName(problem, model);
            setDirection(problem, model);
            addConstraints(problem, model);
            addVariables(problem, model);
            addExplicitConstraintValues(problem, model);

            model.loadMatrixIntoProblem(problem);
        } catch (const char *msg) {
            Nan::ThrowTypeError(msg);
        } catch (string msg) {
            Nan::ThrowTypeError(msg.c_str());
        } catch (...) {
            Nan::ThrowTypeError("Unknown execption");
        }
    }
}
