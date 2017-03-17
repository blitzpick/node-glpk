var glp = require("..");
var model = require("./model.json");
var prob = new glp.Problem();

prob.loadJsonModel(model);

prob.scaleSync(glp.SF_AUTO);
prob.simplexSync({
  presolve: glp.ON
});
if (prob.getNumInt() > 0) {
  function callback(tree) {
    if (tree.reason() == glp.IBINGO) {
      // ...
    }
  }
  prob.intoptSync({
    cbFunc: callback
  });
}

console.log("objective: " + prob.mipObjVal());
prob.delete();
