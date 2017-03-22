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

    bool isUndefined() {
        return obj->IsUndefined();
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

    void addConstraintIndex(string name, int index) {
        this->constraintIndices[name] = index;
    }

    int getConstraintIndex(string name) {
        map<string, int>::iterator constraint = this->constraintIndices.find(name);
        if (constraint == this->constraintIndices.end()) {
            return -1;
        }

        return constraint->second;
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
    map<string, int> constraintIndices;
    vector<int> ia;
    vector<int> ja;
    vector<double> ar;
};

static void checkAndThrow(bool condition, const char *message) {
    if (condition) {
        throw message;
    }
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

    if (operation == "unbounded") {
        return GLP_FR;
    }

    throw (string("Unrecognized constraint type: ") + operation).c_str();
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

static void setConstraintBounds(glp_prob *problem, ObjectWrapper constraint, int index, V8Value operationName) {
    string operation = toString(operationName);
    V8Value operand = constraint[operationName];

    int type = getConstraintType(operation);
    pair<double, double> bounds = getConstraintBounds(operation, operand);
    glp_set_row_bnds(problem, index, type, bounds.first, bounds.second);
}

static V8Value getDependentConstraintOperationName(ObjectWrapper dependentConstraint) {
    ArrayWrapper propertyNames(dependentConstraint.getPropertyNames());
    V8Value firstPropertyName = propertyNames[0];
    if (toString(firstPropertyName) != "terms") {
        return firstPropertyName;
    }

    return propertyNames[1];
}

static void addDependentConstraints(glp_prob *problem, ObjectWrapper dependentConstraints) {
    if (dependentConstraints.isUndefined()) {
        return;
    }

    int numDependentConstraints = dependentConstraints.size();

    int start = glp_add_rows(problem, numDependentConstraints);
    int numValues = glp_get_num_cols(problem);

    ArrayWrapper dependentConstraintNames(dependentConstraints.getPropertyNames());
    for (int i = 0; i < numDependentConstraints; ++i) {
        int constraintIndex = start + i;
        string name = toString(dependentConstraintNames[i]);
        ObjectWrapper dependentConstraint(dependentConstraints[name]);

        if (!dependentConstraint.isUndefined()) {
            glp_set_row_name(problem, constraintIndex, name.c_str());

            checkAndThrow(dependentConstraint.size() != 2, "Dependent constraints must contain a terms object and an operation");

            V8Value operationName = getDependentConstraintOperationName(dependentConstraint);
            setConstraintBounds(problem, dependentConstraint, constraintIndex, operationName);

            vector<double> values(numValues + 1);
            int rowIndices[numValues + 1];
            double rowValues[numValues + 1];

            ObjectWrapper terms = dependentConstraint["terms"];

            int numTerms = terms.size();
            ArrayWrapper termNames(terms.getPropertyNames());
            for (int i = 0; i < numTerms; ++i) {
                V8Value termName = termNames[i];
                ObjectWrapper term(terms[termName]);

                if (!term.isUndefined()) {
                    checkAndThrow(term.size() != 2, "Dependent constraint terms must contain a coefficient and a constant");
                    double coefficient = term["coefficient"]->NumberValue();
                    double constant = term["constant"]->NumberValue();

                    int constraintIndex = glp_find_row(problem, toString(termName).c_str());
                    checkAndThrow(constraintIndex <= 0, "Found an unknown constraint name in the terms object");
                    int count = glp_get_mat_row(problem, constraintIndex, rowIndices, rowValues);
                    for (int i = 0; i < count; ++i) {
                        values[rowIndices[i + 1]] += ((rowValues[i + 1] * coefficient) + constant);
                    }
                }
            }

            int indices[numValues + 1];
            for (int i = 0; i < numValues; ++i) {
                indices[i + 1] = i + 1;
            }
            glp_set_mat_row(problem, constraintIndex, numValues, indices, values.data());
        }
    }
}

static void addVariables(glp_prob *problem, Model &model) {
    V8Value objective = model["objective"];
    checkAndThrow(objective->IsUndefined(), "You must specify an objective value in the model");

    ObjectWrapper variables(model["variables"]);
    checkAndThrow(variables.isUndefined(), "You must specify variables in the model");

    int numVariables = variables.size();
    glp_add_cols(problem, numVariables);

    ArrayWrapper variableNames(variables.getPropertyNames());
    for (int i = 0; i < numVariables; ++i) {
        int index = i + 1;
        V8Value name = variableNames[i];
        ObjectWrapper variable(variables[name]);

        if (!variable.isUndefined()) {
            glp_set_col_name(problem, index, toString(name).c_str());
            glp_set_col_kind(problem, index, getVariableKind(variable));

            ObjectWrapper values(variable["values"]);
            glp_set_obj_coef(problem, index, values[objective]->NumberValue());

            int numValues = values.size();
            ArrayWrapper valueNames(values.getPropertyNames());
            for (int i = 0; i < numValues; ++i) {
                V8Value name = valueNames[i];
                int constraintIndex = model.getConstraintIndex(toString(name));
                if (constraintIndex > 0) {
                    double value = values[name]->NumberValue();
                    model.addMatrixEntry(constraintIndex, index, value);
                }
            }
        }
    }
}

static void addConstraints(glp_prob *problem, Model &model) {
    ObjectWrapper constraints(model["constraints"]);
    checkAndThrow(constraints.isUndefined(), "You must specify constraints in the model");

    int size = constraints.size();
    glp_add_rows(problem, size);

    ArrayWrapper constraintNames(constraints.getPropertyNames());

    for (int i = 0; i < size; ++i) {
        int index = i + 1;
        string name = toString(constraintNames[i]);
        ObjectWrapper constraint(constraints[name]);

        if (!constraint.isUndefined()) {
            model.addConstraintIndex(name, index);

            glp_set_row_name(problem, index, name.c_str());

            checkAndThrow(constraint.size() != 1, "Constraints may contain only a single operation.");

            V8Value operationName = constraint.getPropertyName(0);
            setConstraintBounds(problem, constraint, index, operationName);
        }
    }
}

static int getDirection(Model &model) {
    V8Value directionValue = model["direction"];
    checkAndThrow(!directionValue->IsString(), "The model's direction property must be either 'minimize' or 'maximize'");

    string direction = toString(model["direction"]);

    if (direction == "maximize") {
        return GLP_MAX;
    }

    if (direction == "minimize") {
        return GLP_MIN;
    }

    throw "'direction' must be either 'minimize' or 'maximize'";
}

namespace NodeGLPK {
    void JsonModelLoader::Load(glp_prob *problem, V8Object _model) {
        try {
            Model model(_model);

            glp_create_index(problem);

            V8Value name = model["name"];
            if (name->IsString()) {
                glp_set_prob_name(problem, toString(name).c_str());
            }

            glp_set_obj_dir(problem, getDirection(model));

            addConstraints(problem, model);
            addVariables(problem, model);

            model.loadMatrixIntoProblem(problem);

            ObjectWrapper dependentConstraints(model["dependentConstraints"]);
            addDependentConstraints(problem, dependentConstraints);
        } catch (const char *msg) {
            Nan::ThrowTypeError(msg);
        } catch (string msg) {
            Nan::ThrowTypeError(msg.c_str());
        } catch (...) {
            Nan::ThrowTypeError("Unknown execption");
        }
    }

    void JsonModelLoader::AddDependentConstraints(glp_prob *problem, V8Object dependentConstraints) {
        try {
            addDependentConstraints(problem, dependentConstraints);
        } catch (const char *msg) {
            Nan::ThrowTypeError(msg);
        } catch (string msg) {
            Nan::ThrowTypeError(msg.c_str());
        } catch (...) {
            Nan::ThrowTypeError("Unknown execption");
        }
    }
}
