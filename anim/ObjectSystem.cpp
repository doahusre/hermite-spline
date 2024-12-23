#include "ObjectSystem.h"
#include <chrono>

ObjectSystem::ObjectSystem(const std::string& name) : BaseSystem(name) {
	hermite = new Hermite("hermite");
	
	loadmodel();
}

void ObjectSystem::getState(double* p) {
	time = p[0];
}

std::chrono::high_resolution_clock::time_point prevRealTime = std::chrono::high_resolution_clock::now();
Vector prevPosition = { 0.0, 0.0, 0.0 }; 

void ObjectSystem::setState(double* p) {
	time = p[0];
	double d = distanc(time);
	if (d > 0.999 || time > 0.999) {
		animTcl::OutputMessage("Simulation finished");
		Reset();
		return;
	}
	double totravel = hermite->arcLength(1) * d;
	double uu = hermite->arcLengthInverse(totravel);
	int tableIndex = hermite->arcIndex(uu);

	Vector point1, point2, interpolatedPoint;
	VecCopy(point1, hermite->lookupTable.at(tableIndex).point);
	VecCopy(point2, hermite->lookupTable.at(tableIndex + 1).point);

	double arcLength1 = hermite->lookupTable.at(tableIndex).s;
	double arcLength2 = hermite->lookupTable.at(tableIndex + 1).s;

	double t = (totravel - arcLength1) / (arcLength2 - arcLength1);  // Interpolation factor based on distance

	for (int i = 0; i < 3; ++i) { 
		interpolatedPoint[i] = point1[i] + t * (point2[i] - point1[i]);
	}

	VecCopy(position, interpolatedPoint);


	// Orientation Calculation:
	Vector up, side;
	Vector tangent1, tangent2, interpolatedTangent;
	VecCopy(tangent1, hermite->lookupTable.at(tableIndex).tangent);
	VecCopy(tangent2, hermite->lookupTable.at(tableIndex + 1).tangent);

	for (int i = 0; i < 3; ++i) { 
		interpolatedTangent[i] = tangent1[i] + t * (tangent2[i] - tangent1[i]);
	}
	VecNormalize(interpolatedTangent);


	VecCopy(u, interpolatedTangent);

	up[0] = 0.0;
	up[1] = 0.0; 
	up[2] = 1.0;

	VecCrossProd(side, up, u);
	VecNormalize(side);

	VecCrossProd(v, u, side);
	VecNormalize(v);

	m[0] = side[0]; m[4] = v[0]; m[8] = u[0];
	m[1] = side[1]; m[5] = v[1]; m[9] = u[1];
	m[2] = side[2]; m[6] = v[2]; m[10] = u[2];
	m[3] = 0;      m[7] = 0;    m[11] = 0;
	m[12] = 0;     m[13] = 0;   m[14] = 0;  m[15] = 1;

	// Print debug information
	//animTcl::OutputMessage("time: %f", time);
	//animTcl::OutputMessage("d: %f", d);
	//animTcl::OutputMessage("to travel: %f", totravel);
	//animTcl::OutputMessage("u: %f", u);
	//animTcl::OutputMessage("table index: %d", tableIndex);

	auto currentTime = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> timeElapsed = currentTime - prevRealTime;

	if (timeElapsed.count() >= 1.0) {
		double dx = position[0] - prevPosition[0];
		double dy = position[1] - prevPosition[1];
		double dz = position[2] - prevPosition[2];
		double distance = sqrt(dx * dx + dy * dy + dz * dz);

		double speed = distance / timeElapsed.count();

		animTcl::OutputMessage("Speed: %f units/second", speed);

		prevRealTime = currentTime;
		VecCopy(prevPosition, position);
	}
}


void ObjectSystem::reset(double time) {
	if (hermite->controlPoints.size() == 0) {
		position[0] = 0;
		position[1] = 0;
		position[2] = 0;
	}
	else {
		position[0] = hermite->controlPoints.at(0).point[0];
		position[1] = hermite->controlPoints.at(0).point[1];
		position[2] = hermite->controlPoints.at(0).point[2];
	}

}

double ObjectSystem::distanc(double time) {
	double result;

	if (time < 0.1) {
		result = 0.5 * (1 - cos(PI * (time / 0.1)));
	}
	else if (time > 0.9) {
		result = 1 - 0.5 * (1 - cos(PI * ((time - 0.9) / 0.1)));
	}
	else {
		result = 0.5 + (time - 0.1) * (0.8);
	}

	return result;
}

void ObjectSystem::loadmodel() {
	m_model.ReadOBJ("data/porsche.obj");
}

int ObjectSystem::command(int argc, myCONST_SPEC char** argv) {
	if (argc != 2) {
		animTcl::OutputMessage("system %s: wrong number of params", m_name.c_str());
		return TCL_ERROR;
	}

	if (strcmp(argv[0], "load") == 0) {
		hermite->load(argv[1]);
		hermite->updateTable();
		animTcl::OutputMessage("arc length: %f", hermite->arcLength(1));
		position[0] = hermite->controlPoints.at(0).point[0];
		position[1] = hermite->controlPoints.at(0).point[1];
		position[2] = hermite->controlPoints.at(0).point[2];
		return TCL_OK;
		}
}

void ObjectSystem::display(GLenum mode) {
	hermite->drawPoints();
	hermite->drawCurve();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslated(position[0], position[1], position[2]);
	glMultMatrixd(m);
	glScaled(.01, .01, .01);
	glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
	glPopMatrix();
}
