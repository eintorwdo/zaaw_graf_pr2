#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Texture2D>
#include <osg/AnimationPath>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowTexture>
#include <osgGA/GUIEventHandler>

bool fire = false;
float latAngle = 90;
float angle = 0;
float v = 68;
float g = 9.81;
float t = 0;

osg::AnimationPath * createPath(osg::MatrixTransform * mt){
    osg::ref_ptr<osg::AnimationPath> path = new osg::AnimationPath;
    path->setLoopMode(osg::AnimationPath::NO_LOOPING);
    osg::BoundingSphere sp = mt->computeBound();
    osg::Quat qt1 = osg::Quat((90-latAngle)*M_PI/180, osg::Vec3(0,0,1));
    osg::Quat qt2 = osg::Quat(M_PI/2, osg::Vec3(0,1,0));
    osg::Vec3 center = sp.center();
    osg::Vec3 scale = osg::Vec3(3,3,3);
    osg::Vec3 pos = osg::Vec3(center.x(), center.y(), center.z()-sp.radius());
    path->insert(0,osg::AnimationPath::ControlPoint(pos, qt1, scale));
    path->insert(0.2,osg::AnimationPath::ControlPoint(pos, qt2*qt1, scale));
    return path.release();
}

std::vector<int> generateRandomCoords(){
    int x = std::rand() % 300 + 100;
    int y = std::rand() % 301 + (-150);
    std::vector<int> coords;
    coords.push_back(x);
    coords.push_back(y);
    return coords;
}

class BowlingPin{
    osg::MatrixTransform * pinT;
    osg::AnimationPathCallback * apcb;
    bool fallen;

    public:
        BowlingPin(osg::MatrixTransform * pinT, osg::AnimationPathCallback * apcb){
            this->pinT = pinT;
            this->apcb = apcb;
            fallen = false;
        }

        void checkForIntersection(osg::MatrixTransform * cannonBall){
            if(!fallen){
                osg::BoundingSphere sphere1 = pinT->computeBound();
                osg::BoundingSphere sphere2 = cannonBall->computeBound();
                osg::Vec3 pinCtr = sphere1.center();
                osg::Vec3 ballCtr = sphere2.center();
                
                float distance = std::sqrt(std::pow(pinCtr.x()-ballCtr.x(),2)+std::pow(pinCtr.y()-ballCtr.y(),2)+std::pow(pinCtr.z()-ballCtr.z(),2));

                if(distance < sphere1.radius() + sphere2.radius()){
                    apcb->setPause(false);
                    apcb->setAnimationPath(createPath(pinT));
                    fallen = true;
                }
            }
        }

        void reset(){
            apcb->setPause(true);
            std::vector<int> coords = generateRandomCoords();
            pinT->setMatrix(osg::Matrix::scale(3,3,3)*osg::Matrix::translate(coords[0], coords[1], 0));
            apcb->reset();
            fallen = false;
        }

        bool isFallen(){
            return fallen;
        }
};

class BallSimulation{
    osg::MatrixTransform * cannonBall;
    osg::MatrixTransform * cannon;
    osg::MatrixTransform * cannonT;
    std::vector<BowlingPin *> pins;
    double initialZ;
    double initialY;
    double initialX;

    public:
        BallSimulation(osg::MatrixTransform * cannonBall, osg::MatrixTransform * cannonT, osg::MatrixTransform * cannon, std::vector<BowlingPin *> pins){
            this->cannonBall = cannonBall;
            this->cannonT = cannonT;
            this->cannon = cannon;
            this->pins = pins;
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

        void setInitialCoords(){
            osg::Matrix matrix;
            osg::Vec3f trans;
            osg::Quat rot;
            osg::Vec3f scale;
            osg::Quat so;
            matrix.makeTranslate(osg::Vec3f(10, 0, 9.6));
            matrix = matrix * osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0));
            matrix = matrix * osg::Matrix::rotate((90-latAngle)*M_PI/180, osg::Vec3f(0,0,1));
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
                    t = t + 0.05;
                    for(auto pin = pins.begin(); pin != pins.end(); pin++){
                        (*pin)->checkForIntersection(cannonBall);
                    }
                }
                else{
                    t = 0;
                    fire = false;
                    // resetBall();
                }
            }
        }

        osg::Matrix updateCannonRotation(){
            osg::Matrix matrix = osg::Matrix::translate(10,0,0)*osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::rotate((-latAngle+90)*M_PI/180, osg::Vec3f(0,0,1.0))*osg::Matrix::translate(-10,0,0);
            return matrix;
        }

        bool areAllPinsFallen(){
            for(auto pin = pins.begin(); pin != pins.end(); pin++){
                if(!(*pin)->isFallen()){
                    return false;
                }
            }
            return true;
        }

        void resetPins(){
            for(auto pin = pins.begin(); pin != pins.end(); pin++){
                (*pin)->reset();
            }
        }

        void resetCannon(){
            osg::Matrix matrix = updateCannonRotation();
            cannonT->setMatrix(matrix);
        }

        void resetBall(){
            if(fire == false){
                setInitialCoords();
                if(areAllPinsFallen()){
                    resetPins();
                }
            }
        }

        void rtCannonLeft(){
            latAngle = std::fmax(20,latAngle - 0.5);
            osg::Matrix matrix = updateCannonRotation();
            cannonT->setMatrix(matrix);
            setInitialCoords();
        }

        void rtCannonRight(){
            latAngle = std::fmin(160,latAngle + 0.5);
            osg::Matrix matrix = updateCannonRotation();
            cannonT->setMatrix(matrix);
            setInitialCoords();
        }

        void rtCannonUp(){
            angle = std::fmin(50,angle + 0.5);
            osg::Matrix matrix = updateCannonRotation();
            cannonT->setMatrix(matrix);
            setInitialCoords();
        }

        void rtCannonDown(){
            angle = std::fmax(0,angle - 0.5);
            osg::Matrix matrix = updateCannonRotation();
            cannonT->setMatrix(matrix);
            setInitialCoords();
        }
};

class KeyHandler : public osgGA::GUIEventHandler{
    public:
        KeyHandler(BallSimulation * bs){
            this->bs = bs;
        };

        virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
            osg::Matrix matrix;
            osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
            osg::Vec3 eye;
            osg::Vec3 center;
            osg::Vec3 up;
            viewer->getCamera()->getViewMatrixAsLookAt( eye, center, up );

            switch(ea.getEventType()){
                case(osgGA::GUIEventAdapter::KEYDOWN):{
                    switch(ea.getKey()){
                        case 32:{
                                if(fire == false){
                                    bs->resetBall();
                                    fire = true;
                                }
                            }
                            break;
                        case 'r' : case 'R':{
                                fire = false;
                                t = 0;
                                angle = 0;
                                latAngle = 90;
                                bs->resetBall();
                                bs->resetPins();
                                bs->resetCannon();
                                eye.set(osg::Vec3(-200,0,45));
                                viewer->getCamera()->setViewMatrixAsLookAt(eye, osg::Vec3(0,0,0), osg::Vec3(0,0,1));
                            }
                            break;
                        case 65361:{
                                if(fire == false){
                                    bs->rtCannonLeft();
                                    if(latAngle > 20){
                                        eye.set((eye.x())*cos(0.5*M_PI/180)-eye.y()*sin(0.5*M_PI/180), (eye.x())*sin(0.5*M_PI/180)+eye.y()*cos(0.5*M_PI/180), eye.z());
                                        viewer->getCamera()->setViewMatrixAsLookAt(eye, osg::Vec3(0,0,0), osg::Vec3(0,0,1));
                                    }
                                }
                            }
                            break;
                        case 65363:{
                                if(fire == false){
                                    bs->rtCannonRight();
                                    if(latAngle < 160){
                                        eye.set((eye.x())*cos(-0.5*M_PI/180)-eye.y()*sin(-0.5*M_PI/180), (eye.x())*sin(-0.5*M_PI/180)+eye.y()*cos(-0.5*M_PI/180), eye.z());
                                        viewer->getCamera()->setViewMatrixAsLookAt(eye, osg::Vec3(0,0,0), osg::Vec3(0,0,1));
                                    }
                                }
                            }
                            break;
                        case 65362:{
                                if(fire == false){
                                    bs->rtCannonUp();
                                }
                            }
                            break;
                        case 65364:{
                                if(fire == false){
                                    bs->rtCannonDown();
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
};


auto stworz_scene(){
    const int ReceivesShadowTraversalMask = 0x1;
    const int CastsShadowTraversalMask = 0x2;

    auto scn = new osgShadow::ShadowedScene();

    // swiatlo
    osg::ref_ptr<osg::LightSource> lightSource(new osg::LightSource());                                 
    lightSource->getLight()->setLightNum(0);                             
    lightSource->getLight()->setPosition(osg::Vec4(0.0, 0.0, 200.0, 1.0));
    lightSource->getLight()->setAmbient(osg::Vec4(1, 1, 1.0, 1.0));
    lightSource->getLight()->setDirection(osg::Vec3(0.0, 0.0, 0.0));

    osg::ref_ptr<osg::StateSet> ss = scn->getOrCreateStateSet();
    ss->setMode(GL_LIGHT0, osg::StateAttribute::ON);
    scn->addChild(lightSource);

    // ustawienia cieni
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
    geom->getOrCreateStateSet()->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
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
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D(osgDB::readImageFile("textures/grass3.jpg"));
    texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT ); 
    texture->setWrap( osg::Texture::WRAP_R, osg::Texture::REPEAT );
    texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT ); 
    geom -> getOrCreateStateSet() -> setTextureAttributeAndModes(0, texture.get());
    osg::ref_ptr<osg::Geode> wall = new osg::Geode;
    wall->addDrawable(geom.get());
    // cien
    wall->setNodeMask(ReceivesShadowTraversalMask);
    scn->addChild(wall);

    osg::MatrixTransform * t = new osg::MatrixTransform();
    osg::MatrixTransform * t2 = new osg::MatrixTransform();
    osg::MatrixTransform * cannonT = new osg::MatrixTransform();

    // armata
    osg::Node * cannon = osgDB::readNodeFile("cannon.obj");
    cannon->getOrCreateStateSet()->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
    t2->setMatrix(osg::Matrix::rotate(-latAngle*M_PI/180, osg::Vec3f(0,0,1.0))*osg::Matrix::scale(osg::Vec3f(4.5,4.5,4.5)));
    t2->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    t2->addChild(cannon);

    cannonT->setMatrix(osg::Matrix::rotate(-angle*M_PI/180, osg::Vec3f(0,1,0))*osg::Matrix::translate(0,0,0.15*angle));
    cannonT->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    cannonT->addChild(t2);
    scn->addChild(cannonT);

    // kula armatnia
    osg::Geode * geom_node = new osg::Geode();
    osg::ShapeDrawable * drw = new osg::ShapeDrawable();
    drw->getOrCreateStateSet()->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
    drw->setShape(new osg::Sphere(osg::Vec3(0, 0.0, 0), 0.7));
    texture = new osg::Texture2D(osgDB::readImageFile("textures/stone.jpg"));
    drw -> getOrCreateStateSet() -> setTextureAttributeAndModes(0, texture.get());
    geom_node->addDrawable(drw);
    // cien
    geom_node->setNodeMask(CastsShadowTraversalMask);
    t->setMatrix(osg::Matrix::translate(-0.7, 0, 9.6));
    t->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    t->addChild(geom_node);

    scn->addChild(t);

    // kregle
    std::vector<BowlingPin *> pins;
    osg::Node * pin = osgDB::readNodeFile("pin.obj");
    pin->getOrCreateStateSet()->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
    for(int i=0;i<10;i++){
        osg::MatrixTransform * pinT = new osg::MatrixTransform();
        std::vector<int> coords = generateRandomCoords();
        pinT->setMatrix(osg::Matrix::scale(3,3,3)*osg::Matrix::translate(coords[0], coords[1], 0));
        pinT->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
        pinT->addChild(pin);
        // animacja upadku
        osg::ref_ptr<osg::AnimationPathCallback> apcb = new osg::AnimationPathCallback;
        apcb->setPause(true);
        apcb->setAnimationPath(createPath(pinT));
        pinT->setUpdateCallback(apcb.get());
        BowlingPin * pin = new BowlingPin(pinT, apcb);
        pins.push_back(pin);
        scn->addChild(pinT);
    }

    return std::make_tuple(scn, t, cannonT, t2, pins);
}

int main(int argc, char * argv[]){
    srand((unsigned) time(0));
    auto tup = stworz_scene();
    osg::Node* scn = std::get<0>(tup);
    osg::MatrixTransform * cannonBall = std::get<1>(tup);
    osg::MatrixTransform * cannonT = std::get<2>(tup);
    osg::MatrixTransform * cannon = std::get<3>(tup);
    std::vector<BowlingPin *> pins = std::get<4>(tup);
    BallSimulation * bs = new BallSimulation(cannonBall, cannonT, cannon, pins);
    KeyHandler * kh1 = new KeyHandler(bs);
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(200, 200, 800, 600);
    viewer.setSceneData(scn);
    viewer.addEventHandler(kh1);

    osg::Vec3d eye( -200.0, 0.0, 45.0 );
    osg::Vec3d center( 0.0, 0.0, 0.0 );
    osg::Vec3d up( 0.0, 0.0, 1.0 );

    viewer.setCameraManipulator (NULL);
    viewer.getCamera()->setViewMatrixAsLookAt( eye, center, up );

    while(!viewer.done()){
        bs->updateBall();
        viewer.frame();
    }
    return 0;

    // return viewer.run();
}
