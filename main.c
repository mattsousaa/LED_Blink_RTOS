#include <iostream>

using namespace std;

class Point{

private:
	int x, y;
};

int main(){

	Point p1, p2;
	p1.x = 5;
	p1.y = 3;

	cout << p1.x << " " << p1.y << endl;

	return 0;


}