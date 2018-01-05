#ifndef VIEWER_H
#define VIEWER_H

#include "opengl.h"
#include "shader.h"
#include "camera.h"
#include "particles.h"
#include "collider.h"
#include "quad.h"
#include "mesh.h"
#include "sphere.h"

#include <iostream>

class Viewer{
public:
    //! Constructor
    Viewer();
    virtual ~Viewer();

    // gl stuff
    void init(int w, int h);
    void display();
    void updateScene();
    void reshape(int w, int h);
    void loadProgram();

    // events
    void mousePressed(GLFWwindow* window, int button, int action, int mods);
    void mouseMoved(int x, int y);
    void mouseScroll(double x, double y);
    void keyPressed(int key, int action, int mods);
    void charPressed(int key);
    Quad* getQuad(){return quad;}

private:
    int _winWidth, _winHeight;

    Camera _cam;

    Shader _simplePrg, _blinnPrg;

    ParticleSystem _psys;
    AnchorForce _mouseForce = AnchorForce(nullptr, Vector3d(0,0,0), 1000, 1);

    // Mouse parameters
    Eigen::Vector2f _lastMousePos;
    int _button = -1;
    bool _mod = false;
    PlaneCollider* _collider;
    Quad* quad;
    Mesh* _mesh;
    Mesh* _mesh2;

    std::vector<Shape*> _shapes;
    std::vector<float> _specularCoef;

    Sphere _pointLight;
    Eigen::Vector3f _lightColor;
    float _lightAngle = 0.f;

    std::vector<Sphere*> _lights;
    std::vector<Eigen::Vector3f*> _lightsColors;


};

#endif
