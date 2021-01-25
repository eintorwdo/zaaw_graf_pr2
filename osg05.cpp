#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cmath>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osg/Texture2D>
#include <osg/AnimationPath>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowTexture>

#include <osgGA/GUIEventHandler>

bool fire = false;
float latAngle = 90;
float angle = 45;
float v = 25;
float g = 9.81;
float t = 0;

class BallSimulation{
    osg::MatrixTransform* cannonBall;
    public:
        BallSimulation(osg::MatrixTransform* cannonBall){
            this->cannonBall = cannonBall;
        };

        void updateBall(){
            if(fire == true){
                osg::Matrix matrix = cannonBall->getMatrix();
                osg::Vec3f trans;
                osg::Quat rot;
                osg::Vec3f scale;
                osg::Quat so;
                matrix.decompose(trans, rot, scale, so);
                if(trans.z() > 0){
                    float radians = (angle*M_PI)/180;
                    float radiansLat = (latAngle*M_PI)/180;
                    float x = v * cos(radians) * t * sin(radiansLat);
                    float y = v * cos(radians) * t * cos(radiansLat);
                    float z = v * sin(radians) * t - g*pow(t,2)/2 + 1;
                    matrix.makeTranslate(osg::Vec3f(x, y, z));
                    cannonBall->setMatrix(matrix);
                    t = t + 0.013;
                }
                else{
                    t = 0;
                    fire = false;
                }
            }
        }

        void resetBall(){
            if(fire == false){
                std::cout << "reset" << std::endl;
                osg::Matrix matrix = cannonBall->getMatrix();
                osg::Vec3f trans;
                osg::Quat rot;
                osg::Vec3f scale;
                osg::Quat so;
                matrix.decompose(trans, rot, scale, so);
                matrix.makeTranslate(osg::Vec3f(0, 0, 1));
                cannonBall->setMatrix(matrix);
            }
        }
};

class KeyHandler : public osgGA::GUIEventHandler{
    public:
        KeyHandler(BallSimulation * bs){
            this->bs = bs;
        };

        virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
            switch(ea.getEventType()){
                case(osgGA::GUIEventAdapter::KEYDOWN):{
                    switch(ea.getKey()){
                        case 'a' : case 'A':{
                                std::cout << "fire" << std::endl;
                                fire = true;
                            }
                            break;
                        case 'r' : case 'R':{
                                bs->resetBall();
                            }
                            break;
                        case 65361:{
                                if(fire == false){
                                    latAngle = std::fmax(20,latAngle - 1);
                                    std::cout << latAngle << std::endl;
                                }
                            }
                            break;
                        case 65363:{
                                if(fire == false){
                                    latAngle = std::fmin(160,latAngle + 1);
                                    std::cout << latAngle << std::endl;
                                }
                            }
                        case 65362:{
                                if(fire == false){
                                    angle = std::fmin(80,angle + 1);
                                    std::cout << angle << std::endl;
                                }
                            }
                            break;
                        case 65364:{
                                if(fire == false){
                                    angle = std::fmax(10,angle - 1);
                                    std::cout << angle << std::endl;
                                }
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                }
                default:
                    break;
            }

            return false;
        }

    protected:
        BallSimulation * bs;
        osg::ref_ptr<osg::MatrixTransform> _model;
};

auto stworz_scene()
{
    const int ReceivesShadowTraversalMask = 0x1;
    const int CastsShadowTraversalMask = 0x2;

    auto scn = new osgShadow::ShadowedScene();
    // auto scn = new osg::Group();

    // swiatlo
    osg::ref_ptr<osg::LightSource> lightSource(new osg::LightSource());                                 
    lightSource->getLight()->setLightNum(0);                             
    lightSource->getLight()->setPosition(osg::Vec4(0.0, 0.0, 100.0, 1.0));
    lightSource->getLight()->setAmbient(osg::Vec4(1, 1, 1.0, 1.0));
    lightSource->getLight()->setDirection(osg::Vec3(0.0, 0.0, 0.0));

    osg::ref_ptr<osg::StateSet> ss = scn->getOrCreateStateSet();
    ss->setMode(GL_LIGHT0, osg::StateAttribute::ON);
    scn->addChild(lightSource);


    scn->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
    scn->setCastsShadowTraversalMask(CastsShadowTraversalMask);
    osg::ref_ptr<osgShadow::ShadowTexture> st = new osgShadow::ShadowTexture();
    scn->setShadowTechnique(st);

    // podloze
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(-500.0f, -500.0f, 0.0f));
    vertices->push_back(osg::Vec3(-500.0f, 500.0f, 0.0f));
    vertices->push_back(osg::Vec3(500.0f, 500.0f, 0.0f));
    vertices->push_back(osg::Vec3(500.0f, -500.0f, 0.0f));
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
    osg::ref_ptr<osg::Geometry>geom = new osg::Geometry;
    geom->setVertexArray(vertices.get());
    geom->setNormalArray((normals.get()));
    geom->setNormalBinding((osg::Geometry::BIND_OVERALL));
    geom->addPrimitiveSet((new osg::DrawArrays(GL_QUADS,0,4)));

    // tekstura trawy
    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0.0f, 0.0f);
    (*texcoords)[1].set(15.0f, 0.0f);
    (*texcoords)[2].set(15.0f, 15.0f);
    (*texcoords)[3].set(0.0f, 15.0f);
    geom->setTexCoordArray(0,texcoords);
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D(osgDB::readImageFile("grass.jpg"));
    texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT ); 
    texture->setWrap( osg::Texture::WRAP_R, osg::Texture::REPEAT );
    texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT ); 
    geom -> getOrCreateStateSet() -> setTextureAttributeAndModes(0, texture.get());
    osg::ref_ptr<osg::Geode> wall = new osg::Geode;
    wall->addDrawable(geom.get());
    // cien
    wall->setNodeMask(ReceivesShadowTraversalMask);
    scn->addChild(wall);

    osg::MatrixTransform * t;

    // armata
    osg::Node * cannon = osgDB::readNodeFile("cannon.obj");
    // cien
    // geom_node->setNodeMask(CastsShadowTraversalMask);
    t = new osg::MatrixTransform();
    t->setMatrix(osg::Matrix::translate(0, 0, 0));
    t->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    t->addChild(cannon);

    scn->addChild(t);


    // kula armatnia
    osg::Geode * geom_node = new osg::Geode();
    osg::ShapeDrawable * drw = new osg::ShapeDrawable();
    drw->setShape(new osg::Sphere(osg::Vec3(0, 0.0, 0), 1));
    texture = new osg::Texture2D(osgDB::readImageFile("wood.jpg"));
    drw -> getOrCreateStateSet() -> setTextureAttributeAndModes(0, texture.get());
    geom_node->addDrawable(drw);
    // cien
    geom_node->setNodeMask(CastsShadowTraversalMask);
    t = new osg::MatrixTransform();
    t->setMatrix(osg::Matrix::translate(0, 0, 1));
    t->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    t->addChild(geom_node);

    scn->addChild(t);

    std::vector<osg::Node*> arr(2);
    arr[0] = scn;
    arr[1] = t;

    return std::make_tuple(scn, t);
}

int main(int argc, char * argv[])
{
    auto tup = stworz_scene();
    osg::Node* scn = std::get<0>(tup);
    osg::MatrixTransform* cannonBall = std::get<1>(tup);
    BallSimulation * bs = new BallSimulation(cannonBall);
    KeyHandler * kh1 = new KeyHandler(bs);
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(200, 200, 800, 600);
    viewer.setSceneData(scn);
    viewer.addEventHandler(kh1);

    osg::Vec3d eye( -100.0, 0.0, 35.0 );
    osg::Vec3d center( 0.0, 0.0, 0.0 );
    osg::Vec3d up( 0.0, 0.0, 1.0 );

    // viewer.setCameraManipulator (NULL);
    viewer.getCamera()->setViewMatrixAsLookAt( eye, center, up );

    while(!viewer.done()){
        bs->updateBall();
        viewer.frame();
    }
    return 0;

    // return viewer.run();
}
