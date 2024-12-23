#include "ObjectSimulator.h"

ObjectSimulator::ObjectSimulator(const std::string& name, ObjectSystem* system) : BaseSimulator(name)
{
	m_system = system;
}

int ObjectSimulator::step(double time)
{
	double p[10];
	double t = time / 2000.0;
	//if ((1 - t) < 0.005) return 0;
	p[0] = t;


	m_system->setState(p);

	//make the model move
	return 0;
}

int ObjectSimulator::init(double time)
{
	double p[10];
	p[0] = 0;
	m_system->setState(p);
	return 0;
}

