#include <iostream>
#include "math/math.hpp"

using namespace std;
using namespace robot_math;

int main()
{
    vector2f v1;
    v1 << 1.0f, 2.0f;
    cout << v1.length() << endl;
    vector2f v2 = v1;
    cout << v1 + v2 << endl;
    vector3f v3(4, 5);
    cout << v3 << endl;
    return 0;
}
