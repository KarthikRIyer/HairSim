#pragma once
#ifndef Scene_H
#define Scene_H

#include <vector>
#include <memory>
#include <string>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include "Hair.h"

class Cloth;
class Particle;
class MatrixStack;
class Program;
class Shape;

class Scene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Scene();
	virtual ~Scene();
	
	void load(const std::string &RESOURCE_DIR);
	void init();
	void tare();
	void reset();
	void step();
	
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const;
	void drawHair(const std::shared_ptr<Program> progHair) const;

	double getTime() const { return t; }
	
private:
	double t;
	double h;
	Eigen::Vector3d grav;

	std::shared_ptr<Hair> hair;

	std::shared_ptr<Shape> sphereShape;
	std::vector< std::shared_ptr<Particle> > spheres;
};

#endif
