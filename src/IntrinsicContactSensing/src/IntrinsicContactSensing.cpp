/*
 * Software License Agreement (BSD 3-Clause License)
 *
 *   IntrinsicContactSensing
 *   Copyright (c) 2017-current, Simone Ciotti (simone.ciotti@centropiaggio.unipi.it)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <IntrinsicContactSensing.h>

IntrinsicContactSensing::IntrinsicContactSensing()
{
	/*do nothing*/
}
IntrinsicContactSensing::~IntrinsicContactSensing()
{
/*do nothing*/
}

bool IntrinsicContactSensing::addSensor(std::string id, std::string ip, unsigned short int dataRate, FTSensors::ATI::ForceUnit fu, FTSensors::ATI::TorqueUnit tu, FTSensors::ATI::FilterFrequency ff)
{
	std::shared_ptr<FTSensors::ATI::NetFT > sensor(new FTSensors::ATI::NetFT() );

	sensor->setIP(ip);
	sensor->setDataRate(dataRate);
	sensor->setForceUnit(fu);
	sensor->setTorqueUnit(tu);
	sensor->setFilterFrequency(ff);

	this->sensors.insert( std::pair<std::string, std::shared_ptr<FTSensors::ATI::NetFT > >(id, sensor) );

	return true;
}
bool IntrinsicContactSensing::startSensorDataStream(std::string id, bool calibration)
{
	std::shared_ptr<FTSensors::ATI::NetFT > sensor;

	sensor = this->sensors[id];

	sensor->startDataStream(calibration);

	return true;
}
bool IntrinsicContactSensing::stopSensorDataStream(std::string id)
{
	std::shared_ptr<FTSensors::ATI::NetFT > sensor;

	sensor = this->sensors[id];

	sensor->stopDataStream();

	return true;
}
bool IntrinsicContactSensing::removeSensor(std::string id)
{
	this->stopSensorDataStream(id);
	this->sensors.erase(id);
}

bool IntrinsicContactSensing::addPlane(std::string id, double side, double height)
{
	std::shared_ptr<Surfaces::Ellipsoidal::Plane > plane;

	plane->setSide(side);
	plane->setHeight(height);

	this->surfaces.insert( std::pair<std::string, std::shared_ptr<Surfaces::Ellipsoidal::Plane > >(id, plane) );

	return true;
}
bool IntrinsicContactSensing::addPlane(std::string id, Surfaces::Ellipsoidal::Plane p)
{
	std::shared_ptr<Surfaces::Ellipsoidal::Plane > plane;

	plane = std::make_shared<Surfaces::Ellipsoidal::Plane> (p);

	this->surfaces.insert( std::pair<std::string, std::shared_ptr<Surfaces::Ellipsoidal::Plane > >(id, plane) );

	return true;
}

bool IntrinsicContactSensing::addSphere(std::string id, double radius)
{
	std::shared_ptr<Surfaces::Ellipsoidal::Sphere > sphere;

	sphere->setRadius(radius);

	this->surfaces.insert( std::pair<std::string, std::shared_ptr<Surfaces::Ellipsoidal::Sphere > >(id, sphere) );

	return true;
}
bool IntrinsicContactSensing::addSphere(std::string id, Surfaces::Ellipsoidal::Sphere s)
{
	std::shared_ptr<Surfaces::Ellipsoidal::Sphere > sphere;

	sphere = std::make_shared<Surfaces::Ellipsoidal::Sphere> (s);

	this->surfaces.insert( std::pair<std::string, std::shared_ptr<Surfaces::Ellipsoidal::Sphere > >(id, sphere) );

	return true;
}

bool IntrinsicContactSensing::addCylinder(std::string id, double height, double baseRadius)
{
	std::shared_ptr<Surfaces::Ellipsoidal::Cylinder > cylinder;

	cylinder->setHeight(height);
	cylinder->setBaseRadius(baseRadius);

	this->surfaces.insert( std::pair<std::string, std::shared_ptr<Surfaces::Ellipsoidal::Cylinder > >(id, cylinder) );

	return true;
}
bool IntrinsicContactSensing::addCylinder(std::string id, Surfaces::Ellipsoidal::Cylinder c)
{
	std::shared_ptr<Surfaces::Ellipsoidal::Cylinder > cylinder;

	cylinder = std::make_shared<Surfaces::Ellipsoidal::Cylinder> (c);

	this->surfaces.insert( std::pair<std::string, std::shared_ptr<Surfaces::Ellipsoidal::Cylinder > >(id, cylinder) );

	return true;
}

bool IntrinsicContactSensing::addEllipsoid(std::string id, Eigen::Vector3d principalSemiAxes)
{
	std::shared_ptr<Surfaces::Ellipsoidal::Ellipsoid > ellipsoid;

	ellipsoid->setPrincipalSemiAxes(principalSemiAxes);

	this->surfaces.insert( std::pair<std::string, std::shared_ptr<Surfaces::Ellipsoidal::Ellipsoid > >(id, ellipsoid) );

	return true;
}
bool IntrinsicContactSensing::addEllipsoid(std::string id, Surfaces::Ellipsoidal::Ellipsoid e)
{
	std::shared_ptr<Surfaces::Ellipsoidal::Ellipsoid > ellipsoid;

	ellipsoid = std::make_shared<Surfaces::Ellipsoidal::Ellipsoid> (e);

	this->surfaces.insert( std::pair<std::string, std::shared_ptr<Surfaces::Ellipsoidal::Ellipsoid > >(id, ellipsoid) );

	return true;
}

bool IntrinsicContactSensing::setSurfaceDisplacement(std::string id, double dispX, double dispY, double dispZ)
{
	std::shared_ptr<Surfaces::Surface > surface;

	surface = this->surfaces[id];

	surface->setDisplacement(dispX, dispY, dispZ);

	return true;
}
bool IntrinsicContactSensing::setSurfaceOrientation(std::string id, double alpha, double beta, double gamma)
{
	std::shared_ptr<Surfaces::Surface > surface;

	surface = this->surfaces[id];

	surface->setOrientation(alpha, beta, gamma);

	return true;
}

bool IntrinsicContactSensing::setSurfaceStiffness(std::string id, double K, double r)
{
	std::shared_ptr<Surfaces::Surface > surface;

	surface = this->surfaces[id];

	surface->setStiffness(K, r);

	return true;
}

Surfaces::ContactSensingProblemSolution IntrinsicContactSensing::solveContactSensingProblem(std::string idSensor, std::string idSurface, double forceThreshold)
{
	std::shared_ptr<FTSensors::ATI::NetFT > sensor;
	Eigen::Vector3d f;
	Eigen::Vector3d t;
	std::shared_ptr<Surfaces::Surface > surface;
	Surfaces::ContactSensingProblemSolution csps;

	sensor = this->sensors[idSensor];
	surface = this->surfaces[idSurface];

	sensor->getData(f, t);
	csps = surface->solveContactSensingProblem(f, t, forceThreshold);

	return csps;
}
Surfaces::ContactSensingProblemSolution IntrinsicContactSensing::solveContactSensingProblem(std::string idSurface, Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold)
{
	std::shared_ptr<Surfaces::Surface > surface;
	Surfaces::ContactSensingProblemSolution csps;

	surface = this->surfaces[idSurface];

	csps = surface->solveContactSensingProblem(f, t, forceThreshold);

	return csps;
}
Surfaces::ContactSensingProblemSolution IntrinsicContactSensing::solveContactSensingProblem(std::string idSurface, double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold)
{
	std::shared_ptr<Surfaces::Surface > surface;
	Surfaces::ContactSensingProblemSolution csps;

	surface = this->surfaces[idSurface];

	csps = surface->solveContactSensingProblem(fx, fy, fz, tx, ty, tz, forceThreshold);

	return csps;
}

bool IntrinsicContactSensing::removeSurface(std::string id)
{
	this->surfaces.erase(id);
}
