#include <node.h>
#include <node_object_wrap.h>

#include "glpk/glpk.h"

namespace NodeGLPK {
    using namespace v8;

    class JsonModelLoader {
    public:
        static void Load(glp_prob *problem, Local<Object> modelObj);
    };
}
