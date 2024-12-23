#ifndef MY_HERMITE_H
#define MY_HERMITE_H

#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLModel/GLModel.h>
#include <math.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include "shared/opengl.h"

#define MAX_CONTROL_POINTS 40
#define SAMPLE_POINTS 20

struct ControlPoint {
	Vector point;
	Vector tangent;
	Vector curvature;
};

struct TableEntry {
	double u, s;
	Vector point, tangent, curvature;
};

class Hermite : public BaseSystem
{

public:
	Hermite(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void setPoint(int index, double x, double y, double z);
	void setTangent(int index, double sx, double sy, double sz);
	void addPoint(double x, double y, double z, double sx, double sy, double sz);

	double fPoint(int d, ControlPoint p1, ControlPoint p2, double t);
	double fTangent(int d, ControlPoint p1, ControlPoint p2, double t);
	double fCurvature(int d, ControlPoint p1, ControlPoint p2, double t);

	void catmull();
	double arcLength(double u);
	ControlPoint arcPoint(double u);
	int arcIndex(double u);

	double arcLengthInverse(double s);
	void updateTable();
	void load(std::string filename);

	int command(int argc, myCONST_SPEC char** argv);	
	void drawPoints();
	void drawCurve();
	void display(GLenum mode = GL_RENDER);

	std::vector<ControlPoint> controlPoints;
	std::vector<TableEntry> lookupTable;
};

#endif