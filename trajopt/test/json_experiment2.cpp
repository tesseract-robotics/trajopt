#include "json_serializer.h"
#include <iostream>
using namespace std;
/**
 * Additional semantics: default values
 */
class Poop {
public:
  int x;
  float y;
  enum Color {
    red, green, pink
  };
  Color col;
  Poop() {}
  Poop(int _x, float _y, Color _col) : x(_x), y(_y), col(_col) {}
  void Serialize(JsonSerializer& s) {
    s.SerializeNVP(x);
    s.SerializeNVP(y);
    s.SerializeNVP(col);
  }
};

int main(int argc, char* argv[]) {
  JsonSerializer s(true);
  Poop p(1,2,Poop::red);
  p.Serialize(s);
  cout << s.JsonValue << endl;;

  Json::Value v;
  v["x"] = 3;
  v["y"] = "asdf";
  JsonSerializer s1(false);
  s1.JsonValue = v;
  Poop p1;
  p1.Serialize(s1);
  cout << p1.x << " " << p1.y << " " << p1.col << endl;

}
