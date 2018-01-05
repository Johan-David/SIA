#ifndef COLLIDER_H
#define COLLIDER_H
#include "quad.h"
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;


class Collider
{
public:
    virtual ~Collider();
    virtual void init();
};

class PlaneCollider : public Collider
{
public:
    virtual ~PlaneCollider();
    PlaneCollider(Vector3f x, Vector3f y, Vector3f z, Vector3f w);
    PlaneCollider();

    void init();

    Quad* getCollider(){return _quad;}
private:
    Quad* _quad;
};

#endif // COLLIDER_H
