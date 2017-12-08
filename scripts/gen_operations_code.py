addition_overloads =  """


inline AffExpr operator+(const Var& x, double y) {
  return exprAdd(AffExpr(x), y);
}
inline AffExpr operator+(const AffExpr& x, double y) {
  return exprAdd(x, y);
}
inline QuadExpr operator+(const QuadExpr& x, double y) {
  return exprAdd(x, y);
}

inline AffExpr operator+(const Var& x, const Var& y) {
  return exprAdd(AffExpr(x), y);
}
inline AffExpr operator+(const AffExpr& x, const Var& y) {
  return exprAdd(x, y);
}
inline QuadExpr operator+(const QuadExpr& x, const Var& y) {
  return exprAdd(x, y);
}

inline AffExpr operator+(const Var& x, const AffExpr& y) {
  return exprAdd(AffExpr(x), y);
}
inline AffExpr operator+(const AffExpr& x, const AffExpr& y) {
  return exprAdd(x, y);
}
inline QuadExpr operator+(const QuadExpr& x, const AffExpr& y) {
  return exprAdd(x, y);
}

inline QuadExpr operator+(const Var& x, const QuadExpr& y) {
  return exprAdd(QuadExpr(x), y);
}
inline QuadExpr operator+(const AffExpr& x, const QuadExpr& y) {
  return exprAdd(QuadExpr(x), y);
}
inline QuadExpr operator+(const QuadExpr& x, const QuadExpr& y) {
  return exprAdd(x, y);
}
"""

subtraction_overloads = addition_overloads.replace("operator+", "operator-").replace("exprAdd","exprSub")

def print_overloads():
    print addition_overloads
    print subtraction_overloads
    
addition_funcs = """


inline AffExpr exprAdd(AffExpr a, double b) {
  exprInc(a, b);
  return a;
}
inline AffExpr exprAdd(AffExpr a, const Var& b) {
  exprInc(a, b);
  return a;
}
inline AffExpr exprAdd(AffExpr a, const AffExpr& b) {
  exprInc(a, b);
  return a;
}
inline QuadExpr exprAdd(QuadExpr a, double b) {
  exprInc(a, b);
  return a;
}
inline QuadExpr exprAdd(QuadExpr a, const Var& b) {
  exprInc(a, b);
  return a;
}
inline QuadExpr exprAdd(QuadExpr a, const AffExpr& b) {
  exprInc(a, b);
  return a;
}
inline QuadExpr exprAdd(QuadExpr a, const QuadExpr& b) {
  exprInc(a, b);
  return a;
}    
"""

subtraction_funcs = addition_funcs.replace("Add", "Sub").replace("Inc","Dec")

def print_funcs():
    print addition_funcs
    print subtraction_funcs
    
    
print_overloads()
print "///////////////"
print_funcs()