#include <iostream>

#include "Scene.h"
#include "Particle.h"
#include "Shape.h"
#include "Program.h"

using namespace std;
using namespace Eigen;

Scene::Scene(std::string hairGenMesh) :
	t(0.0),
	h(1e-2),
	grav(0.0, 0.0, 0.0),
	hairGenMesh(hairGenMesh)
{
//    this->hairGenMesh = hairGenMesh;
//    std::cout<<hairGenMesh<<"\n";
}

Scene::~Scene()
{
}

void Scene::load(const string &RESOURCE_DIR)
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

void Scene::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
    glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(1.0, 1.0, 1.0).data());
    for(int i = 0; i < (int)spheres.size(); ++i) {
        spheres[i]->draw(MV, prog);
    }
}

void Scene::drawHair(const std::shared_ptr<Program> prog) const
{
    hair->draw(prog);
}
