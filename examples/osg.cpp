#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/MatrixTransform>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <urdf_parser/urdf_parser.h>

int main(int argc, char* argv[]) {
  std::cout << "Parsing " << argc << std::endl;

  urdf::ModelInterfaceSharedPtr model;
  if(argc < 2) {
    model = urdf::parseURDF("/home/aorthey/git/FibrationTrees/data/nasa_valkyrie_model/robots/valkyrie_D.urdf");
  } else {
    std::string name = argv[1];
    std::cout << "Input:" << name << std::endl;
    model = urdf::parseURDF(name.c_str());
  }
  std::cout << "Parsing " << model->getName() << std::endl;
  
  std::vector<urdf::LinkSharedPtr> links;
  model->getLinks(links);

  // Create an OpenSceneGraph scene graph.
  osg::Group* root = new osg::Group();

  // Create the links and joints from the URDF model.
  for (const auto& link : links) {
    osg::MatrixTransform* transform = new osg::MatrixTransform();
    //transform->setMatrix(link->getTransform());

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,0), 0.5)));

    // osg::Material* material = new osg::Material();
    // material->setDiffuse(osg::Vec4(1.0, 0.0, 0.0, 1.0));
    // geode->setMaterial(material);

    transform->addChild(geode);

    root->addChild(transform);
  }

  // Create a window and render the scene graph.
  osgViewer::Viewer viewer;
  viewer.setSceneData(root);
  viewer.run();

  return 0;
}
