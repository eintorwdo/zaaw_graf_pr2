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
float angle = 0;
float v = 55;
float g = 9.81;
float t = 0;

class BallSimulation{
    osg::MatrixTransform* cannonBall;
    osg::MatrixTransform* cannon;
    osg::MatrixTransform* cannonAndBall;
    double initialZ;
    double initialY;
    double initialX;

    public:
        BallSimulation(osg::MatrixTransform* cannonBall, osg::MatrixTransform * cannonAndBall, osg::MatrixTransform * cannon){
            this->cannonBall = cannonBall;
            this->cannonAndBall = cannonAndBall;
            this->cannon = cannon;
            osg::Matrix matrix = cannonBall->getMatrix();
            osg::Vec3f trans;
            osg::Quat rot;
            osg::Vec3f scale;
            osg::Quat so;
            matrix.decompose(trans, rot, scale, so);

            initialZ = trans.z();
            initialY = trans.y();
            initialX = trans.x();
        };

        void updateZ(){
            osg::Matrix matrix;
            osg::Vec3f trans;
            osg::Quat rot;
            osg::Vec3f scale;
            osg::Quat so;
            matrix.makeTranslate(osg::Vec3f(10, 0, 9.6));
            matrix = matrix * osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0));
            matrix = matrix * osg::Matrix::rotate((90-latAngle)*M_PI/180, osg::Vec3f(0,0,1));
            matrix.decompose(trans, rot, scale, so);
            matrix = matrix * osg::Matrix::translate(-10, 0, 0);
            cannonBall->setMatrix(matrix);
            matrix.decompose(trans, rot, scale, so);
            initialZ = trans.z();
            initialY = trans.y();
            initialX = trans.x();
        }

        void updateBall(){
            if(fire == true){
                osg::Matrix matrix = cannonBall->getMatrix();
                osg::Vec3f trans;
                osg::Quat rot;
                osg::Vec3f scale;
                osg::Quat so; 
                matrix.decompose(trans, rot, scale, so);
                if(trans.z() > 0.5){
                    double radians = (angle*M_PI)/180;
                    double radiansLat = (latAngle*M_PI)/180;
                    double x = v * cos(radians) * t * sin(radiansLat) + initialX;
                    double y = v * cos(radians) * t * cos(radiansLat) + initialY;
                    double z = v * sin(radians) * t - g*pow(t,2)/2 + initialZ;
                    matrix.makeTranslate(osg::Vec3f(x, y, z));
                    cannonBall->setMatrix(matrix);
                    cannonBall->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                    t = t + 0.013;
                }
                else{
                    t = 0;
                }
            }
        }

        void resetBall(){
            if(fire == false){
                std::cout << "reset" << std::endl;
                cannon->setMatrix(osg::Matrix::rotate(-latAngle*M_PI/180, osg::Vec3f(0,0,1.0))*osg::Matrix::scale(osg::Vec3f(4.5,4.5,4.5)));
                cannonAndBall->setMatrix(osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::translate(0,0,0.15*angle));
                osg::Matrix matrix;
                matrix.makeTranslate(osg::Vec3f(-0.7, 0, 9.6));
                cannonBall->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                cannonBall->setMatrix(matrix);
                updateZ();
            }
        }
};

class KeyHandler : public osgGA::GUIEventHandler{
    public:
        KeyHandler(BallSimulation * bs, osg::MatrixTransform * cannonAndBall){
            this->bs = bs;
            this->cannonAndBall = cannonAndBall;
        };

        virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
            osg::Matrix matrix;

            switch(ea.getEventType()){
                case(osgGA::GUIEventAdapter::KEYDOWN):{
                    switch(ea.getKey()){
                        case 'a' : case 'A':{
                                // std::cout << latAngle << ", " << angle << std::endl;
                                fire = true;
                            }
                            break;
                        case 'r' : case 'R':{
                                fire = false;
                                t = 0;
                                angle = 0;
                                latAngle = 90;
                                matrix = osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::translate(0,0,0.15*angle)*osg::Matrix::rotate((-latAngle+90)*M_PI/180, osg::Vec3f(0,0,1.0));
                                cannonAndBall->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                                cannonAndBall->setMatrix(matrix);
                                bs->resetBall();
                            }
                            break;
                        case 65361:{
                                if(fire == false){
                                    latAngle = std::fmax(20,latAngle - 1);
                                    matrix = osg::Matrix::translate(10,0,0)*osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::rotate((-latAngle+90)*M_PI/180, osg::Vec3f(0,0,1.0))*osg::Matrix::translate(-10,0,0);
                                    cannonAndBall->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                                    cannonAndBall->setMatrix(matrix);
                                    bs->updateZ();
                                }
                            }
                            break;
                        case 65363:{
                                if(fire == false){
                                    latAngle = std::fmin(160,latAngle + 1);
                                    // matrix = osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::translate(0,0,0.15*angle)*osg::Matrix::rotate((-latAngle+90)*M_PI/180, osg::Vec3f(0,0,1.0));
                                    matrix = osg::Matrix::translate(10,0,0)*osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::rotate((-latAngle+90)*M_PI/180, osg::Vec3f(0,0,1.0))*osg::Matrix::translate(-10,0,0);
                                    cannonAndBall->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                                    cannonAndBall->setMatrix(matrix);
                                    bs->updateZ();
                                }
                            }
                            break;
                        case 65362:{
                                if(fire == false){
                                    angle = std::fmin(50,angle + 1);
                                    // matrix = osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::translate(0,0,0.15*angle)*osg::Matrix::rotate((-latAngle+90)*M_PI/180, osg::Vec3f(0,0,1.0));
                                    matrix = osg::Matrix::translate(10,0,0)*osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::rotate((-latAngle+90)*M_PI/180, osg::Vec3f(0,0,1.0))*osg::Matrix::translate(-10,0,0);
                                    cannonAndBall->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                                    cannonAndBall->setMatrix(matrix);
                                    bs->updateZ();
                                }
                            }
                            break;
                        case 65364:{
                                if(fire == false){
                                    angle = std::fmax(0,angle - 1);
                                    // matrix = osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::translate(0,0,0.15*angle)*osg::Matrix::rotate((-latAngle+90)*M_PI/180, osg::Vec3f(0,0,1.0));
                                    matrix = osg::Matrix::translate(10,0,0)*osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::rotate((-latAngle+90)*M_PI/180, osg::Vec3f(0,0,1.0))*osg::Matrix::translate(-10,0,0);
                                    cannonAndBall->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                                    cannonAndBall->setMatrix(matrix);
                                    bs->updateZ();
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
        osg::MatrixTransform * cannonAndBall;
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
    (*texcoords)[1].set(55.0f, 0.0f);
    (*texcoords)[2].set(55.0f, 55.0f);
    (*texcoords)[3].set(0.0f, 55.0f);
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
    osg::MatrixTransform * t2;
    osg::MatrixTransform * cannonT;
    cannonT = new osg::MatrixTransform();

    // armata
    osg::Node * cannon = osgDB::readNodeFile("cannon.obj");
    t2 = new osg::MatrixTransform();
    t2->setMatrix(osg::Matrix::rotate(-latAngle*M_PI/180, osg::Vec3f(0,0,1.0))*osg::Matrix::scale(osg::Vec3f(4.5,4.5,4.5)));
    t2->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    t2->addChild(cannon);

    cannonT->addChild(t2);

    // kula armatnia
    osg::Geode * geom_node = new osg::Geode();
    osg::ShapeDrawable * drw = new osg::ShapeDrawable();
    drw->setShape(new osg::Sphere(osg::Vec3(0, 0.0, 0), 0.7));
    texture = new osg::Texture2D(osgDB::readImageFile("wood.jpg"));
    drw -> getOrCreateStateSet() -> setTextureAttributeAndModes(0, texture.get());
    geom_node->addDrawable(drw);
    // cien
    geom_node->setNodeMask(CastsShadowTraversalMask);
    t = new osg::MatrixTransform();
    t->setMatrix(osg::Matrix::translate(-0.7, 0, 9.6));
    t->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    t->addChild(geom_node);

    scn->addChild(t);

    // cannonAndBall->addChild(t);
    cannonT->setMatrix(osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::translate(0,0,0.15*angle));
    cannonT->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    scn->addChild(cannonT);

    return std::make_tuple(scn, t, cannonT, t2);
}

int main(int argc, char * argv[])
{
    auto tup = stworz_scene();
    osg::Node* scn = std::get<0>(tup);
    osg::MatrixTransform* cannonBall = std::get<1>(tup);
    osg::MatrixTransform* cannonT = std::get<2>(tup);
    osg::MatrixTransform* cannon = std::get<3>(tup);
    BallSimulation * bs = new BallSimulation(cannonBall, cannonT, cannon);
    KeyHandler * kh1 = new KeyHandler(bs, cannonT);
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(200, 200, 800, 600);
    viewer.setSceneData(scn);
    viewer.addEventHandler(kh1);

    osg::Vec3d eye( -200.0, 0.0, 45.0 );
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
