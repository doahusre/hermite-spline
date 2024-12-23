#include "Hermite.h"

Hermite::Hermite(const std::string& name) : BaseSystem(name) {

}

void Hermite::getState(double* p) {

}
void Hermite::setState(double* p) {

}
void Hermite::reset(double time) {

}

void Hermite::setPoint(int index, double x, double y, double z) {
	controlPoints.at(index).point[0] = x;
	controlPoints.at(index).point[1] = y;
	controlPoints.at(index).point[2] = z;
}

void Hermite::setTangent(int index, double sx, double sy, double sz) {
	controlPoints.at(index).tangent[0] = sx;
	controlPoints.at(index).tangent[1] = sy;
	controlPoints.at(index).tangent[2] = sz;
}

void Hermite::addPoint(double x, double y, double z, double sx, double sy, double sz) {
	ControlPoint cp;
	cp.point[0] = x;
	cp.point[1] = y;
	cp.point[2] = z;
	cp.tangent[0] = sx;
	cp.tangent[1] = sy;
	cp.tangent[2] = sz;
	animTcl::OutputMessage("in add point");

	
	if (controlPoints.size() >= MAX_CONTROL_POINTS) {
		animTcl::OutputMessage("system %s: max control points", m_name.c_str());
		return;
	}

	controlPoints.push_back(cp);
}

void Hermite::catmull() {
	int size = controlPoints.size();
	if (size < 3) {
		animTcl::OutputMessage("system %s: not enough control points", m_name.c_str());
	}
	for (int i = 0; i < size; i++) {
		if (i == 0) {
			Vector temp1;
			VecSubtract(temp1, controlPoints.at(i + 1).point, controlPoints.at(i).point);
			VecScale(temp1, 2);

			Vector temp2;
			VecSubtract(temp2, controlPoints.at(i + 2).point, controlPoints.at(i).point);
			VecScale(temp2, 0.5);

			Vector tangent;
			VecSubtract(tangent, temp1, temp2);

			VecCopy(controlPoints.at(i).tangent, tangent);
		}
		else if (i == size - 1) {
			Vector temp1;
			VecSubtract(temp1, controlPoints.at(i).point, controlPoints.at(i - 1).point);
			VecScale(temp1, 2);

			Vector temp2;
			VecSubtract(temp2, controlPoints.at(i).point, controlPoints.at(i - 2).point);
			VecScale(temp2, 0.5);

			Vector tangent;
			VecSubtract(tangent, temp1, temp2);

			VecCopy(controlPoints.at(i).tangent, tangent);
		}
		else {
			Vector tangent;
			VecSubtract(tangent, controlPoints.at(i + 1).point, controlPoints.at(i - 1).point);
			VecScale(tangent, 0.5);

			VecCopy(controlPoints.at(i).tangent, tangent);
		}

	}
}

double Hermite::arcLength(double u) {
	int id = (int)(u / (1.0 / (lookupTable.size()-1)) + 0.5);
	return lookupTable[id].s;
}

ControlPoint Hermite::arcPoint(double u) {
	ControlPoint cp;
	int id = (int)(u / (1.0 / (lookupTable.size() - 1)) + 0.5);
	VecCopy(cp.point, lookupTable[id].point);
	VecCopy(cp.tangent, lookupTable[id].tangent);
	VecCopy(cp.curvature, lookupTable[id].curvature);
	return cp;
}

int Hermite::arcIndex(double u) {
	return (int)(u / (1.0 / (lookupTable.size() - 1)) + 0.5);
}



double Hermite::arcLengthInverse(double s) {
	double length = lookupTable.back().s;
	if (s <= 0) return 0;
	if (s >= length) return 1;

	// Bisection search for the value of u corresponding to s
	double low = 0.0;
	double high = 1.0;
	double mid;

	while (high - low > 1e-5) { 
		mid = (low + high) / 2.0;
		double midLength = arcLength(mid);

		if (midLength < s) {
			low = mid; 
		}
		else {
			high = mid; 
		}
	}

	return mid;  // Approximate u value corresponding to s
}

void Hermite::updateTable() {
	lookupTable.clear();
	TableEntry entry;
	double u = 0;
	double s = 0;
	double du = 1.0 / (SAMPLE_POINTS * (controlPoints.size()-1));
	Vector tangent{}, curvature{}, p1{}, p2{};

	for (int i = 0; i < controlPoints.size() - 1; i++) {
		for (int j = 0; j < SAMPLE_POINTS; j++) {
			entry.u = u;
			entry.s = s;

			// calculate arc length and point
			double t1, t2;
			t1 = static_cast<double>(j) / SAMPLE_POINTS;
			//animTcl::OutputMessage("t1: %f", t1);
			t2 = static_cast<double>(j + 1) / SAMPLE_POINTS;
			//animTcl::OutputMessage("t2: %f", t2);
			p1[0] = fPoint(0, controlPoints.at(i), controlPoints.at(i + 1), t1);
			p1[1] = fPoint(1, controlPoints.at(i), controlPoints.at(i + 1), t1);
			p1[2] = fPoint(2, controlPoints.at(i), controlPoints.at(i + 1), t1);
			p2[0] = fPoint(0, controlPoints.at(i), controlPoints.at(i + 1), t2);
			p2[1] = fPoint(1, controlPoints.at(i), controlPoints.at(i + 1), t2);
			p2[2] = fPoint(2, controlPoints.at(i), controlPoints.at(i + 1), t2);
			VecCopy(entry.point, p1);

			// calculate tangent
			tangent[0] = fTangent(0, controlPoints.at(i), controlPoints.at(i + 1), t1);
			tangent[1] = fTangent(1, controlPoints.at(i), controlPoints.at(i + 1), t1);
			tangent[2] = fTangent(2, controlPoints.at(i), controlPoints.at(i + 1), t1);
			VecCopy(entry.tangent, tangent);

			// calculate curvature
			curvature[0] = fCurvature(0, controlPoints.at(i), controlPoints.at(i + 1), t1);
			curvature[1] = fCurvature(1, controlPoints.at(i), controlPoints.at(i + 1), t1);
			curvature[2] = fCurvature(2, controlPoints.at(i), controlPoints.at(i + 1), t1);
			VecCopy(entry.curvature, curvature);

			lookupTable.push_back(entry);
			u += du; // increment u
			s += sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) + pow(p2[2] - p1[2], 2)); // increment s
		}
	}
	entry.u = u;
	entry.s = s;

	//calculate point
	p1[0] = fPoint(0, controlPoints.at(controlPoints.size() - 2), controlPoints.at(controlPoints.size() - 1), 1);
	p1[1] = fPoint(1, controlPoints.at(controlPoints.size() - 2), controlPoints.at(controlPoints.size() - 1), 1);
	p1[2] = fPoint(2, controlPoints.at(controlPoints.size() - 2), controlPoints.at(controlPoints.size() - 1), 1);
	VecCopy(entry.point, p1);

	// calculate tangent
	tangent[0] = fTangent(0, controlPoints.at(controlPoints.size() - 2), controlPoints.at(controlPoints.size() - 1), 1);
	tangent[1] = fTangent(1, controlPoints.at(controlPoints.size() - 2), controlPoints.at(controlPoints.size() - 1), 1);
	tangent[2] = fTangent(2, controlPoints.at(controlPoints.size() - 2), controlPoints.at(controlPoints.size() - 1), 1);
	VecCopy(entry.tangent, tangent);

	// calculate curvature
	curvature[0] = fCurvature(0, controlPoints.at(controlPoints.size() - 2), controlPoints.at(controlPoints.size() - 1), 1);
	curvature[1] = fCurvature(1, controlPoints.at(controlPoints.size() - 2), controlPoints.at(controlPoints.size() - 1), 1);
	curvature[2] = fCurvature(2, controlPoints.at(controlPoints.size() - 2), controlPoints.at(controlPoints.size() - 1), 1);
	VecCopy(entry.curvature, curvature);

	lookupTable.push_back(entry);
	animTcl::OutputMessage("system %s: table updated", m_name.c_str());
	//animTcl::OutputMessage("Table Print Out: ");
	//for (TableEntry e : lookupTable) {
	//	animTcl::OutputMessage("u: %f, s: %f point: %f %f %f tangent: %f %f %f curvature: %f %f %f", e.u, e.s, e.point[0], e.point[1], e.point[2], e.tangent[0], e.tangent[1], e.tangent[2], e.curvature[0], e.curvature[1], e.curvature[2]);
	//}
}

double Hermite::fPoint(int d, ControlPoint p1, ControlPoint p2, double t) {
	double result;
	result = (2 * pow(t, 3) - 3 * pow(t, 2) + 1) * p1.point[d] +
		(pow(t, 3) - 2 * pow(t, 2) + t) * p1.tangent[d] +
		(-2 * pow(t, 3) + 3 * pow(t, 2)) * p2.point[d] +
		(pow(t, 3) - pow(t, 2)) * p2.tangent[d];
	return result;
}

double Hermite::fTangent(int d, ControlPoint p1, ControlPoint p2, double t) {
	double result;
	result = (6 * pow(t, 2) - 6 * t) * p1.point[d] +
		(3 * pow(t, 2) - 4 * t + 1) * p1.tangent[d] +
		(-6 * pow(t, 2) + 6 * t) * p2.point[d] +
		(3 * pow(t, 2) - 2 * t) * p2.tangent[d];
	return result;
}

double Hermite::fCurvature(int d, ControlPoint p1, ControlPoint p2, double t) {
	double result;
	result = (6 * t - 6) * p1.point[d] +
		(3 * t - 4) * p1.tangent[d] +
		(-6 * t + 6) * p2.point[d] +
		(3 * t - 2) * p2.tangent[d];
	return result;
}


void Hermite::load(std::string filename) {

		controlPoints.clear();
	
		std::fstream f;
		f.open(filename);
		if (!f.is_open()) {
			animTcl::OutputMessage("system %s: file not found", m_name.c_str());
			return;
		}

		std::string line;
		std::getline(f, line);

		ControlPoint cp;

		while (std::getline(f, line)) {
			// line format: x y z sx sy sz
			std::istringstream iss(line);
			iss >> cp.point[0] >> cp.point[1] >> cp.point[2] >> cp.tangent[0] >> cp.tangent[1] >> cp.tangent[2];
			controlPoints.push_back(cp);
		}
	}

int Hermite::command(int argc, myCONST_SPEC char** argv) {
	if (argc < 1) {
		animTcl::OutputMessage("system %s: wrong number of params", m_name.c_str());
		animTcl::OutputMessage("Usage: system %s <command> <params>", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "cr") == 0) {
		// Catmull-Rom initialization with second-order accurate boundary conditions
		catmull();
		updateTable();
	}
	else if (strcmp(argv[0], "set") == 0) {
		if (argc != 6) {
			animTcl::OutputMessage("system %s: wrong number of params", m_name.c_str());
			animTcl::OutputMessage("Usage: system %s set <point or tangent> <index> <x> <y> <z>", m_name.c_str());
			return TCL_ERROR;
		}
		if (strcmp(argv[1], "point") == 0) {
			int index = atoi(argv[2]);

			if (index < 0 || index >= MAX_CONTROL_POINTS) {
				animTcl::OutputMessage("system %s: point index out of range", m_name.c_str());
				return TCL_ERROR;
			}

			double x = atof(argv[3]);
			double y = atof(argv[4]);
			double z = atof(argv[5]);

			// update control point of index
			setPoint(index, x, y, z);
		}
		else if (strcmp(argv[1], "tangent") == 0) {
			int index = atoi(argv[2]);

			if (index < 0 || index >= MAX_CONTROL_POINTS) {
				animTcl::OutputMessage("system %s: tangent index out of range", m_name.c_str());
				return TCL_ERROR;
			}

			double x = atof(argv[3]);
			double y = atof(argv[4]);
			double z = atof(argv[5]);

			// update tangent of index
			setTangent(index, x, y, z);
		}
		updateTable();
	}
	else if (strcmp(argv[0], "add") == 0) {
		if (argc != 8) {
			animTcl::OutputMessage("system %s: wrong number of params", m_name.c_str());
			animTcl::OutputMessage("Usage: system %s add point <x> <y> <z> <sx> <sy> <sz>", m_name.c_str());
			return TCL_ERROR;
		}

		if (controlPoints.size() >= MAX_CONTROL_POINTS) {
			animTcl::OutputMessage("system %s: already at maximum control points", m_name.c_str());
			return TCL_ERROR;
		}

		double x = atof(argv[2]);
		double y = atof(argv[3]);
		double z = atof(argv[4]);
		double sx = atof(argv[5]);
		double sy = atof(argv[6]);
		double sz = atof(argv[7]);	

		// add control point and tangent to the end of the list
		addPoint(x, y, z, sx, sy, sz);
		animTcl::OutputMessage("added point");
		if (controlPoints.size() > 1) updateTable();
	}
	else if (strcmp(argv[0], "getArcLength") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("system %s: wrong number of params", m_name.c_str());
			animTcl::OutputMessage("Usage: system %s getArcLength <t>", m_name.c_str());
			return TCL_ERROR;
		}

		double t = atof(argv[1]);
		double a = arcLength(t);
		animTcl::OutputMessage("arc length at t = %f is %f", t, a);
	}
	else if (strcmp(argv[0], "load") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("system %s: wrong number of params", m_name.c_str());
			animTcl::OutputMessage("Usage: system %s load <filename>", m_name.c_str());
			return TCL_ERROR;
		}
		load(argv[1]);
		updateTable();
	}
	else if (strcmp(argv[0], "export") == 0) {
		if (argc != 2) {
			animTcl::OutputMessage("system %s: wrong number of params", m_name.c_str());
			animTcl::OutputMessage("Usage: system %s export <filename>", m_name.c_str());
			return TCL_ERROR;
		}
		animTcl::OutputMessage("Filename: %s", argv[1]);  // Debugging filename

		std::ofstream f;
		// create file with filename
		f.open(argv[1]);

		if (!f.is_open()) {
			animTcl::OutputMessage("Error could not open file");
			return TCL_ERROR;
		}
		
		// first line is the name and number of control points
		f << "hermite " << controlPoints.size() << "\n";
		for (ControlPoint cp : controlPoints) {
			f << cp.point[0] << " " << cp.point[1] << " " << cp.point[2] << " " << cp.tangent[0] << " " << cp.tangent[1] << " " << cp.tangent[2] << "\n";
		}
		f.close();
	}
	else {
		animTcl::OutputMessage("system %s: unknown command %s", m_name.c_str(), argv[0]);
		return TCL_ERROR;

	}
	return 0;
}

void Hermite::drawPoints() {
	glPointSize(5.0);
	glBegin(GL_POINTS);
	for (ControlPoint cp : controlPoints) {
		glVertex3d(cp.point[0], cp.point[1], cp.point[2]);
	}
	glEnd();
}

void Hermite::drawCurve() {
	glLineWidth(2.0);
	glBegin(GL_LINES);
	if (controlPoints.size() < 2) {
		glEnd();
		return;
	}
	for (int i = 0; i < controlPoints.size() - 1; i++) {
		for (int j = 0; j < SAMPLE_POINTS; j++) {
			double t1 = static_cast<double>(j) / SAMPLE_POINTS; 
			double t2 = static_cast<double>(j + 1) / SAMPLE_POINTS; 
			Vector p1{}, p2{};
			p1[0] = fPoint(0, controlPoints.at(i), controlPoints.at(i + 1), t1);
			p1[1] = fPoint(1, controlPoints.at(i), controlPoints.at(i + 1), t1);
			p1[2] = fPoint(2, controlPoints.at(i), controlPoints.at(i + 1), t1);

			p2[0] = fPoint(0, controlPoints.at(i), controlPoints.at(i + 1), t2);
			p2[1] = fPoint(1, controlPoints.at(i), controlPoints.at(i + 1), t2);
			p2[2] = fPoint(2, controlPoints.at(i), controlPoints.at(i + 1), t2);

			glVertex3d(p1[0], p1[1], p1[2]);
			glVertex3d(p2[0], p2[1], p2[2]);
		}
	}
	glEnd();
}

void Hermite::display(GLenum mode) {
	drawPoints();
	drawCurve();
}