#include "collider.h"
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;


Collider::~Collider(){

}

void Collider::init(){

}

PlaneCollider::PlaneCollider(Vector3f x, Vector3f y, Vector3f z, Vector3f w){
    _quad = new Quad();
}

PlaneCollider::PlaneCollider(){
    _quad = new Quad();
}

PlaneCollider::~PlaneCollider(){

}

void PlaneCollider::init(){

}
