/*
 * Software License Agreement (BSD 3-Clause License)
 *
 *   ITSurfaces
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
#ifndef SURFACES_H
#define SURFACES_H

#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>

namespace Surfaces
{
	enum class SurfaceType: unsigned short int {Plane = 1, Sphere = 2, Cylinder = 3, Ellipsoid = 4, NURBS = 5};
    /*
    *   Rigid:      Surface NO DEFORMABLE
    *   Hooke:      F = K * Dx
    *   Quadratic:  F = p2 * Dx ^ 2 + p1 * Dx + p0
    *   Power:      F = a * Dx ^ b
    */
    enum class StiffnessType: unsigned short int {Rigid = 1, Hooke = 2, Quadratic = 3, Power = 4};
	
	std::ostream& operator<<(std::ostream& os, SurfaceType st);
    std::ostream& operator<<(std::ostream& os, StiffnessType st);

	struct ContactSensingProblemSolution
	{
		Eigen::Vector3d F;      //Force retrieved from the sensor expressed in the SENSOR reference frame
        Eigen::Vector3d T; 	 	//Torque retrieved from the sensor expressed in the SENSOR reference frame
        Eigen::Vector3d Fs;     //Force retrieved from the sensor expressed in the SURFACE reference frame
        Eigen::Vector3d Ts;     //Torque retrieved from the sensor expressed in the SURFACE reference frame
        Eigen::Vector3d Tpoc;   //Torque in the point of contact (PoC) expressed in the SURFACE reference frame 
        Eigen::Vector3d PoC;    //Point of Contact respect the sensor frame expressed in the SURFACE reference frame
        Eigen::Vector3d n;	    //The unit vector normal to the surface in the PoC expressed in the SURFACE reference frame
        Eigen::Vector3d Fn;     //The normal force to the surface in the PoC expressed in the SURFACE reference frame
        Eigen::Vector3d Ft;     //The tangential force to the surface in the PoC expressed in the SURFACE reference frame
		double FtFnRatio;       //norm(Ft)/norm(Fn)
        double TnFnRatioLin;    //norm(Tpoc)/norm(Fn)
		double TnFnRatio;       //norm(Tpoc)/(norm(Fn)^(4/3))
	};

    class Surface
    {
        protected:
            SurfaceType surfaceType;
            StiffnessType stiffnessType;
            /* Pose data */
			Eigen::Matrix3d orientation;   //Surface orientation respect the sensor frame
			Eigen::Vector3d displacement; //Surface displacement respect the sensor frame
			std::vector<double> stiffnessCoefficients;

        public:
            SurfaceType getSurfaceType();
            virtual ContactSensingProblemSolution solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold = 0.0) = 0;
            virtual ContactSensingProblemSolution solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold = 0.0) = 0;
            void setDisplacement(Eigen::Vector3d disp);
            void setDisplacement(double dispX, double dispY, double dispZ);
            Eigen::Vector3d getDisplacement();
            void getDisplacement(double& dispX, double& dispY, double& dispZ);
            void setOrientation(Eigen::Matrix3d rot);
            Eigen::Matrix3d getOrientation();
            /*
            *    StiffnessType:
            *                   Rigid:      
            *                               Surface NO DEFORMABLE
            *                               params = empty vector;
            *
            *                   Hooke:      F = K * Dx
            *                               params = K;
            *                               F = params[0] * Dx
            *               
            *                   Quadratic:  F = p2 * Dx ^ 2 + p1 * Dx + p0 
            *                               params = [p0, p1, p2];
            *                               F = params[2] * Dx ^ 2 + params[1] * Dx + params[0] 
            *
            *                   Power:      F = a * Dx ^ b
            *                               params = [a, b];
            *                               F = params[0] * Dx ^ params[1] 
            */
            void setStiffness(StiffnessType st, std::vector<double> params);
			StiffnessType getStiffness(std::vector<double>& params);
    };

    namespace Ellipsoidal
    {
        class Plane:public Surfaces::Surface
        {
            private:
                double side;
                double height;
            public:
                Plane();
                Plane(Plane& p);
				ContactSensingProblemSolution solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold = 0.0);
            	ContactSensingProblemSolution solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold = 0.0);
				void setSide(double s);
                void setHeight(double h);
                double getSide();
                double getHeight();
        };
        class Sphere:public Surfaces::Surface
        {
        	private:
        		double radius;
            public:
                Sphere();
                Sphere(Sphere& s);
                ContactSensingProblemSolution solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold = 0.0);
            	ContactSensingProblemSolution solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold = 0.0);
				void setRadius(double r);
                double getRadius();
        };
        class Cylinder:public Surfaces::Surface
        {
            private:
                double height;
                double baseRadius;
            public:
                Cylinder();
                Cylinder(Cylinder& c);
                ContactSensingProblemSolution solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold = 0.0);
            	ContactSensingProblemSolution solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold = 0.0);
				void setBaseRadius(double br);
				void setHeight(double h);
                double getBaseRadius();
                double getHeight();
        };
        class Ellipsoid:public Surfaces::Surface
        {
            private:
            	Eigen::Vector3d principalSemiAxes;
            public:
                Ellipsoid();
                Ellipsoid(Ellipsoid& e);
                ContactSensingProblemSolution solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold = 0.0);
            	ContactSensingProblemSolution solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold = 0.0);
				void setPrincipalSemiAxes(double psaX, double psaY, double psaZ);
				void setPrincipalSemiAxes(Eigen::Vector3d psa);
                void getPrincipalSemiAxes(double& psaX, double& psaY, double& psaZ);
                Eigen::Vector3d getPrincipalSemiAxes();
        };
    }

    class NURBS:public Surfaces::Surface
    {
        private:
            std::string pointsFile;
        protected:
        	std::string getPointsFile();
        public:
            NURBS();
            NURBS(NURBS& n);
            ContactSensingProblemSolution solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold = 0.0);
        	ContactSensingProblemSolution solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold = 0.0);
            void setPointsFile(std::string pf);
    };
}
#endif // SURFACES_H