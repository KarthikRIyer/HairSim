#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

shared_ptr<Spring> createSpring(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, double E)
{
	auto s = make_shared<Spring>(p0, p1);
	s->E = E;
	Vector3d x0 = p0->x;
	Vector3d x1 = p1->x;
	Vector3d dx = x1 - x0;
	s->L = dx.norm();
	return s;
}

Cloth::Cloth(int rows, int cols,
			 const Vector3d &x00,
			 const Vector3d &x01,
			 const Vector3d &x10,
			 const Vector3d &x11,
			 double mass,
			 double stiffness)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(stiffness > 0.0);
	
	this->rows = rows;
	this->cols = cols;

	int particleCount = rows * cols;
	double particleMass = mass / particleCount;
	
	//
	// Create particles here
	//
	this->n = 0; // size of global vector (do not count fixed vertices)
	double r = 0.01; // Used for collisions
	int nVerts = rows*cols;
	int particleIndex = 0;
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			auto p = make_shared<Particle>();
			particles.push_back(p);
			p->r = r;
			double u = double(j)/(cols-1);
			double v = double(i)/(rows-1);

			Vector3d x0 = x00 * u + x01 * (1 - u);
			Vector3d x1 = x10 * u + x11 * (1 - u);
            // Populate the other member variables of p here
			p->x = x0 * v + x1 * (1 - v);
			p->x0 = x0 * v + x1 * (1 - v);
			p->v = Vector3d(0, 0, 0);
			p->v0 = Vector3d(0, 0, 0);
            p->m = particleMass;
            if ((i == 0 && j == 0) || (i == 0 && j == cols - 1)) {
                p->fixed = true;
                p->i = -1;
            }
            else {
                n+=3;
                p->fixed = false;
                p->i = particleIndex;
                particleIndex+=3;
            }
		}
	}
	
	//
	// Create springs here
	//

    for(int i = 0; i < rows - 1; ++i) {
        for(int j = 0; j < cols - 1; ++j) {
            int index00 = i * cols + j;
            int index01 = i * cols + j + 1;
            int index10 = (i + 1) * cols + j;
            int index11 = (i + 1) * cols + j + 1;

            //x springs
            springs.push_back(createSpring(particles[index00], particles[index01], stiffness));
            if (i == rows - 2) {
                springs.push_back(createSpring(particles[index10], particles[index11], stiffness));
            }
            //y springs
            springs.push_back(createSpring(particles[index00], particles[index10], stiffness));
            if (j == cols - 2) {
                springs.push_back(createSpring(particles[index01], particles[index11], stiffness));
            }
            //shear springs
            springs.push_back(createSpring(particles[index00], particles[index11], stiffness));
            springs.push_back(createSpring(particles[index01], particles[index10], stiffness));
        }
    }

    for(int i = 0; i < rows - 2; ++i) {
        for(int j = 0; j < cols - 2; ++j) {
            int index00 = i * cols + j;
            int index02 = i * cols + j + 2;
            int index20 = (i + 2) * cols + j;
            int index22 = (i + 2) * cols + j + 2;

            //x bending springs
            springs.push_back(createSpring(particles[index00], particles[index02], stiffness));
            if (i == rows - 3) {
                springs.push_back(createSpring(particles[index20], particles[index22], stiffness));
            }
            //y bending springs
            springs.push_back(createSpring(particles[index00], particles[index20], stiffness));
            if (j == cols - 3) {
                springs.push_back(createSpring(particles[index02], particles[index22], stiffness));
            }
        }
    }

	// Allocate system matrices and vectors
	M.resize(n,n);
	K.resize(n,n);
	v.resize(n);
	f.resize(n);
    b.resize(this->n);
    vNext.resize(this->n);
	
	// Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();

	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0));
			texBuf.push_back(j/(cols-1.0));
		}
	}

	// Elements (don't change)
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			// Triangle strip
			eleBuf.push_back(k0);
			eleBuf.push_back(k1);
		}
	}
}

Cloth::~Cloth()
{
}

void Cloth::tare()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->tare();
	}
}

void Cloth::reset()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->reset();
	}
	updatePosNor();
}

void Cloth::updatePosNor()
{
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector3d x = particles[k]->x;
			posBuf[3*k+0] = x(0);
			posBuf[3*k+1] = x(1);
			posBuf[3*k+2] = x(2);
		}
	}
	
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     /|\
			// u0 /_|_\ u1
			//    \ | /
			//     \|/
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector3d x = particles[k]->x;
			Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector3d nor(0.0, 0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = nor(0);
			norBuf[3*k+1] = nor(1);
			norBuf[3*k+2] = nor(2);
		}
	}
}

void Cloth::step(double h, const Vector3d &grav, const vector< shared_ptr<Particle> > spheres)
{
	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();
	M_.clear();
	K_.clear();

	//
	// IMPLEMENT ME!
	//

	b.setZero();
	vNext.setZero();
	for (int i = 0; i < particles.size(); i++) {
	    std::shared_ptr<Particle> p = particles[i];
        if (p->fixed) continue;
        int index = p->i;
        M_.emplace_back(T(index, index, p->m));
        M_.emplace_back(T(index+1, index+1, p->m));
        M_.emplace_back(T(index+2, index+2, p->m));
        v.segment<3>(index) << p->v.x(), p->v.y(), p->v.z();
        f.segment<3>(index) << grav.x()*p->m, grav.y()*p->m, grav.z()*p->m;
	}

    Matrix3d I(3,3);
    I.setIdentity();
	// Add spring forces
	for (int i = 0; i < springs.size(); i++) {
	    std::shared_ptr<Spring> spring = springs[i];
	    double E = spring->E;
	    double L = spring->L;
	    Vector3d deltaX = spring->p1->x - spring->p0->x;
	    double l = deltaX.norm();
        Vector3d fs = ((E * (l - L)) / l) * deltaX;

        Matrix3d Ks = (E/(l*l))*((1.0 - (l-L)/L)*(deltaX*deltaX.transpose()) + ((l - L)/l)*(deltaX.dot(deltaX))*I);

        if (!spring->p0->fixed) {
            int index = spring->p0->i;
            f.segment<3>(index) += fs;
            K_.emplace_back(T(index, index, -Ks.coeffRef(0, 0)));
            K_.emplace_back(T(index, index+1, -Ks.coeffRef(0, 1)));
            K_.emplace_back(T(index, index+2, -Ks.coeffRef(0, 2)));
            K_.emplace_back(T(index+1, index, -Ks.coeffRef(1, 0)));
            K_.emplace_back(T(index+1, index+1, -Ks.coeffRef(1, 1)));
            K_.emplace_back(T(index+1, index+2, -Ks.coeffRef(1, 2)));
            K_.emplace_back(T(index+2, index, -Ks.coeffRef(2, 0)));
            K_.emplace_back(T(index+2, index+1, -Ks.coeffRef(2, 1)));
            K_.emplace_back(T(index+2, index+2, -Ks.coeffRef(2, 2)));
        }
        if (!spring->p1->fixed) {
            int index = spring->p1->i;
            f.segment<3>(index) += -fs;
            K_.emplace_back(T(index, index, -Ks.coeffRef(0, 0)));
            K_.emplace_back(T(index, index+1, -Ks.coeffRef(0, 1)));
            K_.emplace_back(T(index, index+2, -Ks.coeffRef(0, 2)));
            K_.emplace_back(T(index+1, index, -Ks.coeffRef(1, 0)));
            K_.emplace_back(T(index+1, index+1, -Ks.coeffRef(1, 1)));
            K_.emplace_back(T(index+1, index+2, -Ks.coeffRef(1, 2)));
            K_.emplace_back(T(index+2, index, -Ks.coeffRef(2, 0)));
            K_.emplace_back(T(index+2, index+1, -Ks.coeffRef(2, 1)));
            K_.emplace_back(T(index+2, index+2, -Ks.coeffRef(2, 2)));
        }
        if (!spring->p0->fixed && !spring->p1->fixed) {
            int index0 = spring->p0->i;
            int index1 = spring->p1->i;

            K_.emplace_back(T(index0, index1, Ks.coeffRef(0, 0)));
            K_.emplace_back(T(index0, index1+1, Ks.coeffRef(0, 1)));
            K_.emplace_back(T(index0, index1+2, Ks.coeffRef(0, 2)));
            K_.emplace_back(T(index0+1, index1, Ks.coeffRef(1, 0)));
            K_.emplace_back(T(index0+1, index1+1, Ks.coeffRef(1, 1)));
            K_.emplace_back(T(index0+1, index1+2, Ks.coeffRef(1, 2)));
            K_.emplace_back(T(index0+2, index1, Ks.coeffRef(2, 0)));
            K_.emplace_back(T(index0+2, index1+1, Ks.coeffRef(2, 1)));
            K_.emplace_back(T(index0+2, index1+2, Ks.coeffRef(2, 2)));

            K_.emplace_back(T(index1, index0, Ks.coeffRef(0, 0)));
            K_.emplace_back(T(index1, index0+1, Ks.coeffRef(0, 1)));
            K_.emplace_back(T(index1, index0+2, Ks.coeffRef(0, 2)));
            K_.emplace_back(T(index1+1, index0, Ks.coeffRef(1, 0)));
            K_.emplace_back(T(index1+1, index0+1, Ks.coeffRef(1, 1)));
            K_.emplace_back(T(index1+1, index0+2, Ks.coeffRef(1, 2)));
            K_.emplace_back(T(index1+2, index0, Ks.coeffRef(2, 0)));
            K_.emplace_back(T(index1+2, index0+1, Ks.coeffRef(2, 1)));
            K_.emplace_back(T(index1+2, index0+2, Ks.coeffRef(2, 2)));
        }

	}

	// handle collisions
    double c = 1e1;
	for (int i = 0; i < spheres.size(); i++) {
	    std::shared_ptr<Particle> sphere = spheres[i];
	    for (int j = 0; j < particles.size(); j++) {
	        std::shared_ptr<Particle> particle = particles[j];

	        Vector3d deltaX = particle->x - sphere->x;
	        double l = deltaX.norm();
	        double d = particle->r + sphere->r - l;
	        if (d < 0) continue;
            Vector3d n = deltaX/l;
            Vector3d fc = c * d * n;
            Matrix3d Kc = c*d*I;
            int index = particle->i;
            f.segment<3>(index) += fc;
            K_.emplace_back(T(index, index, -Kc.coeffRef(0, 0)));
            K_.emplace_back(T(index, index+1, -Kc.coeffRef(0, 1)));
            K_.emplace_back(T(index, index+2, -Kc.coeffRef(0, 2)));
            K_.emplace_back(T(index+1, index, -Kc.coeffRef(1, 0)));
            K_.emplace_back(T(index+1, index+1, -Kc.coeffRef(1, 1)));
            K_.emplace_back(T(index+1, index+2, -Kc.coeffRef(1, 2)));
            K_.emplace_back(T(index+2, index, -Kc.coeffRef(2, 0)));
            K_.emplace_back(T(index+2, index+1, -Kc.coeffRef(2, 1)));
            K_.emplace_back(T(index+2, index+2, -Kc.coeffRef(2, 2)));
	    }
	}
    M.setFromTriplets(M_.begin(), M_.end());
    K.setFromTriplets(K_.begin(), K_.end());
	SparseMatrix<double> A = M - h*h*K;
	b = M * v + (h * f);

    ConjugateGradient<SparseMatrix<double>> cg;
    cg.setMaxIterations(25);
    cg.setTolerance(1e-6);
    cg.compute(A);
    vNext = cg.solveWithGuess(b, v);

    for (int i = 0; i < particles.size(); i++) {
        std::shared_ptr<Particle> p = particles[i];
        if (p->fixed) continue;
        int index = p->i;

        p->v.x() = vNext.segment<3>(index).coeffRef(0);
        p->v.y() = vNext.segment<3>(index).coeffRef(1);
        p->v.z() = vNext.segment<3>(index).coeffRef(2);

        p->x = p->x + h*p->v;
    }

	// Update position and normal buffers
	updatePosNor();
}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	glUniform3fv(p->getUniform("kdFront"), 1, Vector3f(1.0, 0.0, 0.0).data());
	glUniform3fv(p->getUniform("kdBack"),  1, Vector3f(1.0, 1.0, 0.0).data());
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("aNor");
	glEnableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	for(int i = 0; i < rows; ++i) {
		glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	}
	glDisableVertexAttribArray(h_nor);
	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}
