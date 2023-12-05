#pragma once
#ifndef Scene_H
#define Scene_H

#include <vector>
#include <memory>
#include <string>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include "Hair.h"
#include "SimParams.h"

class Cloth;
class Particle;
class MatrixStack;
class Program;
class Shape;
class Texture;
class IForceField;
class Gravity;
class Wind;

class Scene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Scene();
	virtual ~Scene();

	void load(const std::string &RESOURCE_DIR, const std::string &DATA_DIR, int texUnit);
	void init();
	void tare();
	void reset();
	void updateSimParams(SimParams& simParams);
	void step();
	
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const;
	void drawHair(const std::shared_ptr<Program> progHair) const;

	double getTime() const { return t; }
	
private:
	double t;
	double h;
	SimParams simParams;
	std::string hairGenMesh;

	std::shared_ptr<Gravity> gravity;
	std::shared_ptr<Wind> wind;
	std::vector<std::shared_ptr<IForceField>> forceFields;

	std::shared_ptr<Hair> hair;

	std::shared_ptr<Shape> sphereShape;
	std::vector< std::shared_ptr<Particle> > spheres;

    std::vector<std::shared_ptr<Shape> > shapes;
    std::map<std::string, std::shared_ptr<Texture>> textureMap;

    std::vector<std::string> textureData;
    std::vector<std::vector<std::string>> meshData;

    void loadDataInputFile(const std::string &DATA_DIR);
};

#endif
