#include <json/json.h>
#include <iostream>
#include <string>
using namespace std;
using namespace Json;
int main() {
  Value x;
  x.append(3);
  x.append(4);
  cout << x << endl;
  Value y;
  y["x"]["z"] = 2;
  y["asdf"];
  cout << y << endl;
  Value k;
  cout << k.isArray() << endl;;
}
