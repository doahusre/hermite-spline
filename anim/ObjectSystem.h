#ifndef MY_OBJECT_SYSTEM_H
#define MY_OBJECT_SYSTEM_H

#include "BaseSystem.h"
#include "Hermite.h"
#include <shared/defs.h>
#include "shared/opengl.h"
#include <util/util.h>
#include <GLModel/GLModel.h>
#include "animTcl.h"
#include "anim.h"

#include <vector>
#include <math.h>

class ObjectSystem : public BaseSystem
{
public:
	ObjectSystem(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	double distanc(double time);

	void loadmodel();
	int command(int argc, myCONST_SPEC char** argv);
	void display(GLenum mode = GL_RENDER);

	Hermite* hermite;

	Vector position = { 0, 0, 0 };
	double distance = 0;

	double time = 0;

	Vector u, v, w;
	GLdouble m[16] = {
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1
	};

protected:
	GLMmodel m_model;
};



#endif
