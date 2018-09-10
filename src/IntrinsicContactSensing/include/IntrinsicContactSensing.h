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
#ifndef ICS_H
#define ICS_H

#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <memory>
#include <map>
#include <ITSurfaces.h>
#include <FTSensors.h>

class IntrinsicContactSensing
{
	private:
		std::map<std::string, std::shared_ptr<FTSensors::ATI::NetFT > > sensors;
		std::map<std::string, std::shared_ptr<Surfaces::Surface > > surfaces;
	public:
		IntrinsicContactSensing();
		~IntrinsicContactSensing();

		bool addSensor(std::string id, std::string ip, unsigned short int dataRate, FTSensors::ATI::ForceUnit fu, FTSensors::ATI::TorqueUnit tu, FTSensors::ATI::FilterFrequency ff);
        bool startSensorDataStream(std::string id, bool calibration = true);
        bool stopSensorDataStream(std::string id);
        bool removeSensor(std::string id);

        bool addPlane(std::string id, double side, double height);
        bool addPlane(std::string id, Surfaces::Ellipsoidal::Plane p);
        
        bool addSphere(std::string id, double radius);
        bool addSphere(std::string id, Surfaces::Ellipsoidal::Sphere s);
        
        bool addCylinder(std::string id, double height, double baseRadius);
        bool addCylinder(std::string id, Surfaces::Ellipsoidal::Cylinder c);
        
        bool addEllipsoid(std::string id, Eigen::Vector3d principalSemiAxes);
        bool addEllipsoid(std::string id, Surfaces::Ellipsoidal::Ellipsoid e);

        bool setSurfaceDisplacement(std::string id, double dispX, double dispY, double dispZ);
        /*
        *   The orientation is achieved by composing 3 elemental rotations
        *   The 3 elemental rotations are INTRINSIC (respect to the current axes)  
        *   Rot = Rz(alpha) * Rx'(beta) * Rz''(gamma)
        */
        bool setSurfaceOrientation(std::string id, double alpha, double beta, double gamma);
        /*
            Two way to describe the surface elastic behaviour
                1. Hooke's law:                 
                                        F = K * Dx --> homogeneous and isotropic material
                                        stiffnessCoefficients(0) = K
                                        stiffnessCoefficients(1) = 1
                2. Power approximation: 
                                        F = K * Dx^r
                                        stiffnessCoefficients(0) = K
                                        stiffnessCoefficients(1) = r
                3. Rigid surface:
                                        stiffnessCoefficients(0) = 0
                                        stiffnessCoefficients(1) = 1
        */
        bool setSurfaceStiffness(std::string id, double K = 0.0, double r = 1.0);
        /*
            Retrieves F/T data from the sensor idSensor and use them to solve the contact sensing problem for the surface idSurface
        */
        Surfaces::ContactSensingProblemSolution solveContactSensingProblem(std::string idSensor, std::string idSurface, double forceThreshold = 0.0);
        /*
            Solves the contact sensing problem for the surface idSurface using F/T data retrieved from an external resource (e.g. from a data file)
        */
        Surfaces::ContactSensingProblemSolution solveContactSensingProblem(std::string idSurface, Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold = 0.0);
        Surfaces::ContactSensingProblemSolution solveContactSensingProblem(std::string idSurface, double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold = 0.0);

        bool removeSurface(std::string id);
};
#endif // ICS_H