#include <iostream>
#include <fstream>
#include <glm/gtc/type_ptr.hpp>

#include "Scene.h"
#include "Particle.h"
#include "Shape.h"
#include "Program.h"
#include "Texture.h"
#include "MatrixStack.h"

using namespace std;
using namespace Eigen;

void Scene::loadDataInputFile(const std::string &DATA_DIR)
{
    string filename = DATA_DIR + "input.txt";
    ifstream in;
    in.open(filename);
    if(!in.good()) {
        cout << "Cannot read " << filename << endl;
        return;
    }
    cout << "Loading " << filename << endl;

    string line;
    while(1) {
        getline(in, line);
        if(in.eof()) {
            break;
        }
        if(line.empty()) {
            continue;
        }
        // Skip comments
        if(line.at(0) == '#') {
            continue;
        }
        // Parse lines
        string key, value;
        stringstream ss(line);
        // key
        ss >> key;
        if(key.compare("TEXTURE") == 0) {
            ss >> value;
            textureData.push_back(value);
        } else if(key.compare("MESH") == 0) {
            vector<string> mesh;
            ss >> value;
            mesh.push_back(value); // obj
            ss >> value;
            mesh.push_back(value); // texture
            meshData.push_back(mesh);
        } else {
            cout << "Unknown key word: " << key << endl;
        }
    }
    in.close();
}

Scene::Scene() :
	t(0.0),
	h(1e-2),
	grav(0.0, 0.0, 0.0)
{
}

Scene::~Scene()
{
}

void Scene::load(const string &RESOURCE_DIR, const string &DATA_DIR, int texUnit)
{
	// Units: meters, kilograms, seconds
	h = 5e-3;
	
	grav << 0.0, -9.8, 0.0;

    sphereShape = make_shared<Shape>();
    sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

    auto sphere = make_shared<Particle>(sphereShape);
    spheres.push_back(sphere);
    sphere->r = 0.13;
    sphere->x = Vector3d(0.0, 0.377, -0.02);

    hairGenMesh = DATA_DIR + "scalp.obj";

    loadDataInputFile(DATA_DIR);

    // Create shapes
    for(const auto &mesh : meshData) {
        auto shape = make_shared<Shape>();
        shapes.push_back(shape);
        shape->loadMesh(DATA_DIR + mesh[0]);
        shape->setTextureFilename(mesh[1]);
        shape->init();
    }

    for(const auto &filename : textureData) {
        auto textureKd = make_shared<Texture>();
        textureMap[filename] = textureKd;
        textureKd->setFilename(DATA_DIR + filename);
        textureKd->init();
        textureKd->setUnit(texUnit); // Bind to unit 1
        textureKd->setWrapModes(GL_REPEAT, GL_REPEAT);
    }

}

void Scene::init()
{
    hair = std::make_shared<Hair>(20, 50, 0.8, 0.4, hairGenMesh);
    hair->init();
    sphereShape->init();
}

void Scene::tare()
{
    for(int i = 0; i < (int)spheres.size(); ++i) {
        spheres[i]->tare();
    }
}

void Scene::reset()
{
	t = 0.0;
	hair->reset();
}

void Scene::step()
{
	t += h;

    // Move the sphere
//    if(!spheres.empty()) {
//        auto s = spheres.front();
//        Vector3d x0 = s->x;
//        s->x(2) = 0.5 * sin(0.5*t);
//    }

	// Simulate the hair
	hair->step(h, grav, spheres);
}

void Scene::draw(std::shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
    for(const auto &shape : shapes) {
        textureMap.at(shape->getTextureFilename())->bind(prog->getUniform("kdTex"));
//        textureMap[shape->getTextureFilename()]->bind(prog->getUniform("kdTex"));
        glLineWidth(1.0f); // for wireframe
//        glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
//        glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
        glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
        glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
        glUniform1f(prog->getUniform("s"), 200.0f);
        shape->setProgram(prog);
        shape->draw();
        textureMap.at(shape->getTextureFilename())->unbind();
    }

    glUniform3f(prog->getUniform("ka"), 0.1f, 0.1f, 0.1f);
    glUniform3f(prog->getUniform("ks"), 0.1f, 0.1f, 0.1f);
    glUniform1f(prog->getUniform("s"), 200.0f);
//    glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(1.0, 1.0, 1.0).data());
    for(int i = 0; i < (int)spheres.size(); ++i) {
        spheres[i]->draw(MV, prog);
    }
}

void Scene::drawHair(const std::shared_ptr<Program> prog) const
{
    hair->draw(prog);
}
