#include "viewer.h"
#include "camera.h"
#include "quad.h"

extern int WIDTH;
extern int HEIGHT;

using namespace Eigen;

Viewer::Viewer()
{
}

Viewer::~Viewer()
{
}

////////////////////////////////////////////////////////////////////////////////
// GL stuff

// initialize OpenGL context
void Viewer::init(int w, int h){
    _winWidth = w;
    _winHeight = h;

    // Couleur d'arrière plan
    glClearColor(0.0, 0.0, 0.0, 0.0);

    loadProgram();

    Quad* quad = new Quad();
    quad->init();
    quad->setTransformationMatrix( AngleAxisf((float)M_PI_2, Vector3f(-1,0,0)) * Scaling(20.f,20.f,1.f) * Translation3f(0,0,-0.5));
    _shapes.push_back(quad);
    _specularCoef.push_back(0.);

    _quad = new Quad();
    _quad->init();

    Mesh* mesh = new Mesh();
    mesh->load(DATA_DIR"/models/tw.off");
    mesh->init();
    _shapes.push_back(mesh);
    _specularCoef.push_back(0.75);

    mesh = new Mesh();
    mesh->load(DATA_DIR"/models/sphere.off");
    mesh->init();
    mesh->setTransformationMatrix(Translation3f(0,0,2.f) * Scaling(0.5f) );
    _shapes.push_back(mesh);
    _specularCoef.push_back(0.2);

    _pointLight = Sphere(0.025f);
    _pointLight.init();
    _pointLight.setTransformationMatrix(Affine3f(Translation3f(1,0.75,1)));
    _lightColor = Vector3f(1,1,1);

    //Lights
    Sphere * light1 = new Sphere(0.025f);
    light1->init();
    light1->setTransformationMatrix(Affine3f(Translation3f(0,1,5)));
    Vector3f * lightColor1 = new Vector3f(0.0,1.0,0.0);
    _lights.push_back(light1);
    _lightsColors.push_back(lightColor1);

    Sphere * light2 = new Sphere(0.025f);
    light2->init();
    light2->setTransformationMatrix(Affine3f(Translation3f(-2,0.75,-1)));
    Vector3f * lightColor2 = new Vector3f(1,0.0,0.0);
    _lights.push_back(light2);
    _lightsColors.push_back(lightColor2);

    Sphere * light3 = new Sphere(0.025f);
    light3->init();
    light3->setTransformationMatrix(Affine3f(Translation3f(1,0.75,1)));
    Vector3f * lightColor3 = new Vector3f(0.0,0.0,1.0);
    _lights.push_back(light3);
    _lightsColors.push_back(lightColor3);

    AlignedBox3f aabb;
    for(int i=0; i<_shapes.size(); ++i)
        aabb.extend(_shapes[i]->boundingBox());

    _cam.setSceneCenter(aabb.center());
    _cam.setSceneRadius(aabb.sizes().maxCoeff());
    _cam.setSceneDistance(_cam.sceneRadius() * 3.f);
    _cam.setMinNear(0.1f);
    _cam.setNearFarOffsets(-_cam.sceneRadius() * 100.f,
                            _cam.sceneRadius() * 100.f);
    _cam.setScreenViewport(AlignedBox2f(Vector2f(0.0,0.0), Vector2f(w,h)));

    fbo = new FBO();
    fbo->init(w,h);


    glViewport(0, 0, w, h);
    glEnable(GL_DEPTH_TEST);
}

void Viewer::reshape(int w, int h){
    _winWidth = w;
    _winHeight = h;
    _cam.setScreenViewport(AlignedBox2f(Vector2f(0.0,0.0), Vector2f(w,h)));
    glViewport(0, 0, w, h);
}


/*!
   callback to draw graphic primitives
 */
void Viewer::display()
{
    fbo->bind();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    _gBufferPrg.activate();
    glUniformMatrix4fv(_gBufferPrg.getUniformLocation("projection_matrix"),1,false,_cam.computeProjectionMatrix().data());
    glUniformMatrix4fv(_gBufferPrg.getUniformLocation("view_matrix"),1,false,_cam.computeViewMatrix().data());

    for(int i=0; i<_shapes.size(); ++i)
    {
        glUniformMatrix4fv(_gBufferPrg.getUniformLocation("model_matrix"),1,false,_shapes[i]->getTransformationMatrix().data());
        Matrix3f normal_matrix = (_cam.computeViewMatrix()*_shapes[i]->getTransformationMatrix()).linear().inverse().transpose();
        glUniformMatrix3fv(_gBufferPrg.getUniformLocation("normal_matrix"), 1, GL_FALSE, normal_matrix.data());
        glUniform1f(_gBufferPrg.getUniformLocation("specular_coef"),_specularCoef[i]);

        _shapes[i]->display(&_gBufferPrg);
    }
    _gBufferPrg.deactivate();
    fbo->unbind();
    //fbo->savePNG("out");

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_DST_COLOR);
    glBlendEquation(GL_MAX);
    glClear(GL_COLOR_BUFFER_BIT);

    _lightAngle += M_PI_2/100.0;
    for (int i = 0; i < _lights.size(); i++) {
        _deferredPrg.activate();

        Vector3f lightPos = (_lights[i]->getTransformationMatrix()*Vector4f(0,0,0,1)).head(3);
        Vector4f lightPosH;
        lightPosH << lightPos, 1.f;

        Vector2f sizeWindow = Vector2f(WIDTH, HEIGHT);
        Matrix4f matInv = _cam.computeProjectionMatrix().inverse();
        glUniformMatrix4fv(_deferredPrg.getUniformLocation("inv_projection_matrix"),1,false,matInv.data());
        glUniform2fv(_deferredPrg.getUniformLocation("size_window"),1,sizeWindow.data());
        glUniform4fv(_deferredPrg.getUniformLocation("light_pos"),1,(_cam.computeViewMatrix()*lightPosH).eval().data());
        glUniform3fv(_deferredPrg.getUniformLocation("light_col"),1,_lightsColors[i]->data());

        //Textures
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fbo->renderedTexture[0]);
        glUniform1i(_deferredPrg.getUniformLocation("color_buffer"),0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, fbo->renderedTexture[1]);
        glUniform1i(_deferredPrg.getUniformLocation("normal_buffer"),1);

        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, fbo->depthTexture);
        glUniform1i(_deferredPrg.getUniformLocation("depth_buffer"),2);

        _quad->display(&_deferredPrg);

        _deferredPrg.deactivate();

        // Draw pointlight sources
        _simplePrg.activate();
        glUniformMatrix4fv(_simplePrg.getUniformLocation("projection_matrix"),1,false,_cam.computeProjectionMatrix().data());
        glUniformMatrix4fv(_simplePrg.getUniformLocation("view_matrix"),1,false,_cam.computeViewMatrix().data());
        glUniformMatrix4fv(_simplePrg.getUniformLocation("model_matrix"),1,false,_lights[i]->getTransformationMatrix().data());
        glUniform3fv(_simplePrg.getUniformLocation("light_col"),1,_lightsColors[i]->data());
        _lights[i]->display(&_simplePrg);
        _simplePrg.deactivate();
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}


void Viewer::updateScene()
{
    display();
}

void Viewer::loadProgram()
{
    _simplePrg.loadFromFiles(DATA_DIR"/shaders/simple.vert", DATA_DIR"/shaders/simple.frag");
    _gBufferPrg.loadFromFiles(DATA_DIR"/shaders/gbuffer.vert", DATA_DIR"/shaders/gbuffer.frag");
    _deferredPrg.loadFromFiles(DATA_DIR"/shaders/deferred.vert", DATA_DIR"/shaders/deferred.frag");
    checkError();
}

////////////////////////////////////////////////////////////////////////////////
// Events

/*!
   callback to manage mouse : called when user press or release mouse button
   You can change in this function the way the user
   interact with the system.
 */
void Viewer::mousePressed(GLFWwindow *window, int button, int action)
{
    if(action == GLFW_PRESS) {
        if(button == GLFW_MOUSE_BUTTON_LEFT)
        {
            _cam.startRotation(_lastMousePos);
        }
        else if(button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            _cam.startTranslation(_lastMousePos);
        }
        _button = button;
    }else if(action == GLFW_RELEASE) {
        if(_button == GLFW_MOUSE_BUTTON_LEFT)
        {
            _cam.endRotation();
        }
        else if(_button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            _cam.endTranslation();
        }
        _button = -1;
    }
}


/*!
   callback to manage mouse : called when user move mouse with button pressed
   You can change in this function the way the user
   interact with the system.
 */
void Viewer::mouseMoved(int x, int y)
{
    if(_button == GLFW_MOUSE_BUTTON_LEFT)
    {
        _cam.dragRotate(Vector2f(x,y));
    }
    else if(_button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        _cam.dragTranslate(Vector2f(x,y));
    }
    _lastMousePos = Vector2f(x,y);
}

void Viewer::mouseScroll(double x, double y)
{
    _cam.zoom((y>0)? 1.1: 1./1.1);
}

/*!
   callback to manage keyboard interactions
   You can change in this function the way the user
   interact with the system.
 */
void Viewer::keyPressed(int key, int action, int mods)
{
    if(key == GLFW_KEY_R && action == GLFW_PRESS)
        loadProgram();
}

void Viewer::charPressed(int key)
{
}
