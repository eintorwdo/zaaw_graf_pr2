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

#include <osgGA/GUIEventHandler>

bool fire = false;
float angle = M_PI/4;
float v = 25;
float g = 9.81;
std::chrono::time_point<std::chrono::high_resolution_clock> startTime;

class KeyHandler : public osgGA::GUIEventHandler{
    public:
        KeyHandler(){};

        virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
            switch(ea.getEventType()){
                case(osgGA::GUIEventAdapter::KEYDOWN):{
                    switch(ea.getKey()){
                        case 'a' : case 'A':{
                                std::cout << "fire" << std::endl;
                                fire = true;
                                startTime = std::chrono::high_resolution_clock::now();
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
        osg::ref_ptr<osg::MatrixTransform> _model;
};

auto stworz_scene()
{
    auto scn = new osg::Group();

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(-100.0f, -100.0f, 0.0f));
    vertices->push_back(osg::Vec3(-100.0f, 100.0f, 0.0f));
    vertices->push_back(osg::Vec3(100.0f, 100.0f, 0.0f));
    vertices->push_back(osg::Vec3(100.0f, -100.0f, 0.0f));
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
    osg::ref_ptr<osg::Geometry>geom = new osg::Geometry;
    geom->setVertexArray(vertices.get());
    geom->setNormalArray((normals.get()));
    geom->setNormalBinding((osg::Geometry::BIND_OVERALL));
    geom->addPrimitiveSet((new osg::DrawArrays(GL_QUADS,0,4)));

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
    scn->addChild(wall);

    osg::MatrixTransform * t;
    osg::Geode * geom_node = new osg::Geode();
    osg::ShapeDrawable * drw = new osg::ShapeDrawable();
    drw->setShape(new osg::Sphere(osg::Vec3(0, 0.0, 0), 1));
    texture = new osg::Texture2D(osgDB::readImageFile("wood.jpg"));
    drw -> getOrCreateStateSet() -> setTextureAttributeAndModes(0, texture.get());
    geom_node->addDrawable(drw);
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
    KeyHandler * kh1 = new KeyHandler();
    KeyHandler * kh2 = new KeyHandler();
    auto tup = stworz_scene();
    osg::Node* scn = std::get<0>(tup);
    osg::MatrixTransform* cannonBall = std::get<1>(tup);
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(200, 200, 800, 600);
    viewer.setSceneData(scn);
    viewer.addEventHandler(kh1);
    // viewer.addEventHandler(kh2);

    osg::Vec3d eye( -100.0, 0.0, 15.0 );
    osg::Vec3d center( 0.0, 0.0, 0.0 );
    osg::Vec3d up( 0.0, 0.0, 1.0 );

    // viewer.setCameraManipulator (NULL);
    viewer.getCamera()->setViewMatrixAsLookAt( eye, center, up );

    std::chrono::time_point<std::chrono::high_resolution_clock> t1;

    while(!viewer.done()){
        if(fire == true){
            t1 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float, std::milli> duration = t1 - startTime;
            float durationVal = duration.count();
            std::cout << duration.count()/1000 << std::endl;
            osg::Matrix matrix = cannonBall->getMatrix();
            osg::Vec3f trans;
            osg::Quat rot;
            osg::Vec3f scale;
            osg::Quat so;
            matrix.decompose(trans, rot, scale, so);
            if(trans.z() > 0){
                float x = v * cos(angle) * durationVal/1000;
                float z = v * sin(angle) * durationVal/1000 - g*pow(durationVal/1000,2)/2 + 1;
                matrix.makeTranslate(osg::Vec3f(x, trans.y(), z));
                cannonBall->setMatrix(matrix);
            }
            else{
                fire = false;
            }
        }
        viewer.frame();
    }
    return 0;

    // return viewer.run();
}
