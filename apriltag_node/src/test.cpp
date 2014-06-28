#include <iostream>
using std::cout;
using std::endl;
#include "apriltagmap.h"

int main() {
  kr::Apriltag tag;
  cout << "id: " << tag.id << endl;
  cout << "c: " << tag.c << endl;
  cout << "p: " << tag.p[0] << endl;
  return 0;
}
