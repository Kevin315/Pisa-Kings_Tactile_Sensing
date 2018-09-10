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
#include <ITSurfaces.h>

std::ostream& Surfaces::operator<<(std::ostream& os, Surfaces::SurfaceType st)
{
    switch(st)
    {
        case Surfaces::SurfaceType::Plane 	  : os << "Plane";     break;
        case Surfaces::SurfaceType::Sphere 	  : os << "Sphere";    break;
        case Surfaces::SurfaceType::Cylinder  : os << "Cylinder";  break;
        case Surfaces::SurfaceType::Ellipsoid : os << "Ellipsoid"; break;
        case Surfaces::SurfaceType::NURBS 	  : os << "NURBS";     break;
        default		                          : os.setstate(std::ios_base::failbit);
    }
    return os;
}
std::ostream& Surfaces::operator<<(std::ostream& os, Surfaces::StiffnessType st)
{
    switch(st)
    {
        case Surfaces::StiffnessType::Rigid 	: os << "Rigid";     break;
        case Surfaces::StiffnessType::Hooke   	: os << "Hooke";    break;
        case Surfaces::StiffnessType::Quadratic : os << "Quadratic";  break;
        case Surfaces::StiffnessType::Power 	: os << "Power"; break;
        default		                          	: os.setstate(std::ios_base::failbit);
    }
    return os;
}
/*
	Surfaces
		Surface
*/
Surfaces::SurfaceType Surfaces::Surface::getSurfaceType()
{
	return this->surfaceType;
}
void Surfaces::Surface::setDisplacement(Eigen::Vector3d disp)
{
	this->displacement = disp;
}
void Surfaces::Surface::setDisplacement(double dispX, double dispY, double dispZ)
{
	this->displacement(0) = dispX;
	this->displacement(1) = dispY;
	this->displacement(2) = dispZ;
}
Eigen::Vector3d Surfaces::Surface::getDisplacement()
{
	return this->displacement;
}
void Surfaces::Surface::getDisplacement(double& dispX, double& dispY, double& dispZ)
{
	dispX = this->displacement(0);
	dispY = this->displacement(1);
	dispZ = this->displacement(2);
}
void Surfaces::Surface::setOrientation(Eigen::Matrix3d rot)
{
	this->orientation = rot;
}
Eigen::Matrix3d Surfaces::Surface::getOrientation()
{
	return this->orientation;
}
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
void Surfaces::Surface::setStiffness(Surfaces::StiffnessType st, std::vector<double> params)
{
	this->stiffnessType = st;
	this->stiffnessCoefficients = params;
}
Surfaces::StiffnessType Surfaces::Surface::getStiffness(std::vector<double>& params)
{
	params = this->stiffnessCoefficients;
	return this->stiffnessType;
}
/*
	Surfaces
		Ellipsoidal

    Constant coefficient matrix A
    The elements extern to the main diagonal are equal to zero
    Where 1/a, 1/b and 1/c are the elements of the main diagonal of matrix A
    For general ellipsoids the principal axes of the surface are given by:
    -) 2*a*R (x-axis)
    -) 2*b*R (y-axis)
    -) 2*c*R (z-axis)
    where 1/a, 1/b, 1/c are >0 and <=1, then a,b,c >= 1
    where R is a scale factor used for convenience

    Particular cases:
        Plane: 1/a = 0, 1/b = 0, 1/c = 1 where R is the dimension along z-axis
        Sphere: 1/a = 1, 1/b = 1, 1/c = 1 where R is the radius
        Cylinder: 1/a = 1, 1/b = 1, 1/c = 0  where R is the radius of the circular base	
*/
/*
	Surfaces
		Ellipsodial
			Plane
*/
Surfaces::Ellipsoidal::Plane::Plane()
{
	this->surfaceType = Surfaces::SurfaceType::Plane;
	this->stiffnessType = Surfaces::StiffnessType::Rigid;
	this->orientation = Eigen::Matrix3d::Zero();
	this->displacement = Eigen::Vector3d::Zero();
	this->stiffnessCoefficients.clear();
	this->side = 0.0;
	this->height = 0.0;
}
Surfaces::Ellipsoidal::Plane::Plane(Plane& p)
{
	this->surfaceType = p.getSurfaceType();
	this->orientation = p.getOrientation();
	this->displacement = p.getDisplacement();
	std::vector<double> params;
	this->stiffnessType = p.getStiffness(params);
	this->stiffnessCoefficients = params;
	this->side = p.getSide();
	this->height = p.getHeight();
}
Surfaces::ContactSensingProblemSolution Surfaces::Ellipsoidal::Plane::solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;
	Eigen::Matrix3d A;
	Eigen::Vector3d Fz;
	double K;
	double R;

	csps.F = f;
	csps.T = t;
	csps.Fs = this->orientation.transpose() * f;
	csps.Ts = this->orientation.transpose() * (t - this->displacement.cross(f));
	f = csps.Fs;
	t = csps.Ts;
	csps.PoC = Eigen::Vector3d::Zero();
	csps.n = Eigen::Vector3d::Zero();
	csps.Tpoc = Eigen::Vector3d::Zero();
	csps.Fn = Eigen::Vector3d::Zero();
	csps.Ft = Eigen::Vector3d::Zero();
	csps.FtFnRatio = 0.0;
	csps.TnFnRatioLin = 0.0;
	csps.TnFnRatio = 0.0; 

	if(f.norm() < forceThreshold)
		return csps;

	A << 0, 0, 0,
		 0, 0, 0,
		 0, 0, 1;

	Fz << 0, 0, f(2);
	
	switch(this->stiffnessType)
	{
		case Surfaces::StiffnessType::Hooke:
			R = this->height - (Fz.norm()/this->stiffnessCoefficients[0]);
			break;
		case Surfaces::StiffnessType::Quadratic:
			{
				double p0 = this->stiffnessCoefficients[0];
				double p1 = this->stiffnessCoefficients[1];
				double p2 = this->stiffnessCoefficients[2];
				R = this->height - (-p1 + sqrt(pow(p1,2) - 4*p2*(p0 - Fz.norm())));
			}
			break;
		case Surfaces::StiffnessType::Power:
			R = this->height - pow(Fz.norm()/this->stiffnessCoefficients[0], 1.0/this->stiffnessCoefficients[1]);
			break;
		default:
			R = this->height;
			break;
	}
		
	K = - f.dot(t) / (R * Fz.norm());

	csps.PoC = (1.0 / Fz.squaredNorm()) * (Fz.cross(t) + (R * Fz.norm() * f));
	
	csps.n = A * A * csps.PoC;
	csps.n.normalize();
	
	csps.Tpoc = K * A * A * csps.PoC;
	
	csps.Fn = f.dot(csps.n) * csps.n;
	csps.Ft = f - csps.Fn;
	
	csps.FtFnRatio = csps.Ft.norm() / csps.Fn.norm();
	csps.TnFnRatioLin = csps.Tpoc.norm() / csps.Fn.norm();
	csps.TnFnRatio = csps.Tpoc.norm() / pow(csps.Fn.norm(), 4.0/3.0); 

	return csps;
}
Surfaces::ContactSensingProblemSolution Surfaces::Ellipsoidal::Plane::solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;
	Eigen::Vector3d f;
	Eigen::Vector3d t;

	f << fx, fy, fz;
	t << tx, ty, tz;

	csps = this->solveContactSensingProblem(f, t, forceThreshold);

	return csps;
}
void Surfaces::Ellipsoidal::Plane::setSide(double s)
{
	this->side = s;
}
void Surfaces::Ellipsoidal::Plane::setHeight(double h)
{
	this->height = h;
}
double Surfaces::Ellipsoidal::Plane::getSide()
{
	return this->side;
}
double Surfaces::Ellipsoidal::Plane::getHeight()
{
	return this->height;
}
/*
	Surfaces
		Ellipsodial
			Sphere
*/
Surfaces::Ellipsoidal::Sphere::Sphere()
{
	this->surfaceType = Surfaces::SurfaceType::Sphere;
	this->stiffnessType = Surfaces::StiffnessType::Rigid;
	this->orientation = Eigen::Matrix3d::Zero();
	this->displacement = Eigen::Vector3d::Zero();
	this->stiffnessCoefficients.clear();
	this->radius = 0.0;
}
Surfaces::Ellipsoidal::Sphere::Sphere(Sphere& s)
{
	this->surfaceType = s.getSurfaceType();
	this->orientation = s.getOrientation();
	this->displacement = s.getDisplacement();
	std::vector<double> params;
	this->stiffnessType = s.getStiffness(params);
	this->stiffnessCoefficients = params;
	this->radius = s.getRadius();
}
Surfaces::ContactSensingProblemSolution Surfaces::Ellipsoidal::Sphere::solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;
	Eigen::Matrix3d A;
	double sigma;
	double K;
	double R;

	csps.F = f;
	csps.T = t;
	csps.Fs = this->orientation.transpose() * f;
	csps.Ts = this->orientation.transpose() * (t - this->displacement.cross(f));
	f = csps.Fs;
	t = csps.Ts;
	csps.PoC = Eigen::Vector3d::Zero();
	csps.n = Eigen::Vector3d::Zero();
	csps.Tpoc = Eigen::Vector3d::Zero();
	csps.Fn = Eigen::Vector3d::Zero();
	csps.Ft = Eigen::Vector3d::Zero();
	csps.FtFnRatio = 0.0;
	csps.TnFnRatioLin = 0.0;
	csps.TnFnRatio = 0.0;

	if(f.norm() < forceThreshold)
		return csps;

	A << 1, 0, 0,
		 0, 1, 0,
		 0, 0, 1;

	switch(this->stiffnessType)
	{
		case Surfaces::StiffnessType::Hooke:
			R = this->radius - (f.norm()/this->stiffnessCoefficients[0]);
			break;
		case Surfaces::StiffnessType::Quadratic:
			{
				double p0 = this->stiffnessCoefficients[0];
				double p1 = this->stiffnessCoefficients[1];
				double p2 = this->stiffnessCoefficients[2];
				R = this->radius - (-p1 + sqrt(pow(p1,2) - 4*p2*(p0 - f.norm())));
			}
			break;
		case Surfaces::StiffnessType::Power:
			R = this->radius - pow(f.norm()/this->stiffnessCoefficients[0], 1.0/this->stiffnessCoefficients[1]);
			break;
		default:
			R = this->radius;
			break;
	}		

	sigma = t.squaredNorm() - (R * R * f.squaredNorm());

	K = (-(f.dot(t) / sqrt(f.dot(t) * f.dot(t))) / (sqrt(2.0) * R)) * sqrt(sigma + sqrt((sigma * sigma) + (4.0 * R * R * f.dot(t) * f.dot(t))));

	if(K != 0)
		csps.PoC = (1.0 / (K * ((K * K) + f.squaredNorm()))) * ((K * K * t) + (K * f.cross(t)) + (f.dot(t) * f));
	else
		return csps;//TO DO: Wrench Axis Method
	
	csps.n = A * A * csps.PoC;
	csps.n.normalize();
	
	csps.Tpoc = K * A * A * csps.PoC;
	
	csps.Fn = f.dot(csps.n) * csps.n;
	csps.Ft = f - csps.Fn;
	
	csps.FtFnRatio = csps.Ft.norm() / csps.Fn.norm();
	csps.TnFnRatioLin = csps.Tpoc.norm() / csps.Fn.norm();
	csps.TnFnRatio = csps.Tpoc.norm() / pow(csps.Fn.norm(), 4.0/3.0); 

	return csps;
}
Surfaces::ContactSensingProblemSolution Surfaces::Ellipsoidal::Sphere::solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;
	Eigen::Vector3d f;
	Eigen::Vector3d t;

	f << fx, fy, fz;
	t << tx, ty, tz;

	csps = this->solveContactSensingProblem(f, t, forceThreshold);

	return csps;
}
void Surfaces::Ellipsoidal::Sphere::setRadius(double r)
{
	this->radius = r;
}
double Surfaces::Ellipsoidal::Sphere::getRadius()
{
	return this->radius;
}
/*
	Surfaces
		Ellipsodial
			Cylinder
*/
Surfaces::Ellipsoidal::Cylinder::Cylinder()
{
	this->surfaceType = Surfaces::SurfaceType::Cylinder;
	this->stiffnessType = Surfaces::StiffnessType::Rigid;
	this->orientation = Eigen::Matrix3d::Zero();
	this->displacement = Eigen::Vector3d::Zero();
	this->stiffnessCoefficients.clear();
	this->height = 0.0;
	this->baseRadius = 0.0;
}
Surfaces::Ellipsoidal::Cylinder::Cylinder(Cylinder& c)
{
	this->surfaceType = c.getSurfaceType();
	this->orientation = c.getOrientation();
	this->displacement = c.getDisplacement();
	std::vector<double> params;
	this->stiffnessType = c.getStiffness(params);
	this->stiffnessCoefficients = params;
	this->baseRadius = c.getBaseRadius();
	this->height = c.getHeight();
}
Surfaces::ContactSensingProblemSolution Surfaces::Ellipsoidal::Cylinder::solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;
	Eigen::Matrix3d A;
	Eigen::Vector3d Mz;
	Eigen::Vector3d Fperp;
	double sigma;
	double K;
	double R;

	csps.F = f;
	csps.T = t;
	csps.Fs = this->orientation.transpose() * f;
	csps.Ts = this->orientation.transpose() * (t - this->displacement.cross(f));
	f = csps.Fs;
	t = csps.Ts;
	csps.PoC = Eigen::Vector3d::Zero();
	csps.n = Eigen::Vector3d::Zero();
	csps.Tpoc = Eigen::Vector3d::Zero();
	csps.Fn = Eigen::Vector3d::Zero();
	csps.Ft = Eigen::Vector3d::Zero();
	csps.FtFnRatio = 0.0;
	csps.TnFnRatioLin = 0.0;
	csps.TnFnRatio = 0.0; 

	if(f.norm() < forceThreshold)
		return csps;

	A << 1, 0, 0,
		 0, 1, 0,
		 0, 0, 0;

 	Mz << 0, 0, t(2);

    Fperp << f(0), f(1), 0;

    switch(this->stiffnessType)
	{
		case Surfaces::StiffnessType::Hooke:
			R = this->baseRadius - (Fperp.norm()/this->stiffnessCoefficients[0]);
			break;
		case Surfaces::StiffnessType::Quadratic:
			{
				double p0 = this->stiffnessCoefficients[0];
				double p1 = this->stiffnessCoefficients[1];
				double p2 = this->stiffnessCoefficients[2];
				R = this->baseRadius - (-p1 + sqrt(pow(p1,2) - 4*p2*(p0 - Fperp.norm())));
			}
			break;
		case Surfaces::StiffnessType::Power:
			R = this->baseRadius - pow(Fperp.norm()/this->stiffnessCoefficients[0], 1.0/this->stiffnessCoefficients[1]);
			break;
		default:
			R = this->baseRadius;
			break;
	}

	K = -f.dot(t) / sqrt((R * R * Fperp.squaredNorm()) - Mz.squaredNorm());

	if(K != 0)
		csps.PoC = (1.0 / (K * Fperp.squaredNorm())) * ((K * K * Mz) + (K * Fperp.cross(t)) + (f.dot(t) * f));
	else
		return csps;//TO DO: Wrench Axis Method
	
	csps.n = A * A * csps.PoC;
	csps.n.normalize();
	
	csps.Tpoc = K * A * A * csps.PoC;
	
	csps.Fn = f.dot(csps.n) * csps.n;
	csps.Ft = f - csps.Fn;
	
	csps.FtFnRatio = csps.Ft.norm() / csps.Fn.norm();
	csps.TnFnRatioLin = csps.Tpoc.norm() / csps.Fn.norm();
	csps.TnFnRatio = csps.Tpoc.norm() / pow(csps.Fn.norm(), 4.0/3.0); 

	return csps;
}
Surfaces::ContactSensingProblemSolution Surfaces::Ellipsoidal::Cylinder::solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;
	Eigen::Vector3d f;
	Eigen::Vector3d t;

	f << fx, fy, fz;
	t << tx, ty, tz;

	csps = this->solveContactSensingProblem(f, t, forceThreshold);

	return csps;
}
void Surfaces::Ellipsoidal::Cylinder::setBaseRadius(double br)
{
	this->baseRadius = br;
}
void Surfaces::Ellipsoidal::Cylinder::setHeight(double h)
{
	this->height = h;
}
double Surfaces::Ellipsoidal::Cylinder::getBaseRadius()
{
	return this->baseRadius;
}
double Surfaces::Ellipsoidal::Cylinder::getHeight()
{
	return this->height;
}
/*
	Surfaces
		Ellipsodial
			Ellipsoid
*/
Surfaces::Ellipsoidal::Ellipsoid::Ellipsoid()
{
	this->surfaceType = Surfaces::SurfaceType::Ellipsoid;
	this->stiffnessType = Surfaces::StiffnessType::Rigid;
	this->orientation = Eigen::Matrix3d::Zero();
	this->displacement = Eigen::Vector3d::Zero();
	this->stiffnessCoefficients.clear();
	this->principalSemiAxes = Eigen::Vector3d::Zero();
}
Surfaces::Ellipsoidal::Ellipsoid::Ellipsoid(Ellipsoid& e)
{
	this->surfaceType = e.getSurfaceType();
	this->orientation = e.getOrientation();
	this->displacement = e.getDisplacement();
	std::vector<double> params;
	this->stiffnessType = e.getStiffness(params);
	this->stiffnessCoefficients = params;
	this->principalSemiAxes = e.getPrincipalSemiAxes();
}
Surfaces::ContactSensingProblemSolution Surfaces::Ellipsoidal::Ellipsoid::solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;
	double alpha, beta, gamma, invR, R, D, K, sigma, detG;
	Eigen::Vector3d Af, invAt, AAf, psa;
	Eigen::Matrix3d A;
	
	csps.F = f;
	csps.T = t;
	csps.Fs = this->orientation.transpose() * f;
	csps.Ts = this->orientation.transpose() * (t - this->displacement.cross(f));
	f = csps.Fs;
	t = csps.Ts;
	csps.PoC = Eigen::Vector3d::Zero();
	csps.n = Eigen::Vector3d::Zero();
	csps.Tpoc = Eigen::Vector3d::Zero();
	csps.Fn = Eigen::Vector3d::Zero();
	csps.Ft = Eigen::Vector3d::Zero();
	csps.FtFnRatio = 0.0;
	csps.TnFnRatioLin = 0.0;
	csps.TnFnRatio = 0.0; 

	if(f.norm() < forceThreshold)
		return csps;

	switch(this->stiffnessType)
	{
		case Surfaces::StiffnessType::Hooke:
			psa(0) = this->principalSemiAxes(0) - (fabs(f(0))/this->stiffnessCoefficients[0]);
			psa(1) = this->principalSemiAxes(1) - (fabs(f(1))/this->stiffnessCoefficients[0]);
			psa(2) = this->principalSemiAxes(2) - (fabs(f(2))/this->stiffnessCoefficients[0]);
			break;
		case Surfaces::StiffnessType::Quadratic:
			{
				double p0 = this->stiffnessCoefficients[0];
				double p1 = this->stiffnessCoefficients[1];
				double p2 = this->stiffnessCoefficients[2];
				psa(0) = this->principalSemiAxes(0) - (-p1 + sqrt(pow(p1,2) - 4*p2*(p0 - fabs(f(0)))));
				psa(1) = this->principalSemiAxes(1) - (-p1 + sqrt(pow(p1,2) - 4*p2*(p0 - fabs(f(1)))));
				psa(2) = this->principalSemiAxes(2) - (-p1 + sqrt(pow(p1,2) - 4*p2*(p0 - fabs(f(2)))));
			}
			break;
		case Surfaces::StiffnessType::Power:
			psa(0) = this->principalSemiAxes(0) - pow(fabs(f(0))/this->stiffnessCoefficients[0], 1.0/this->stiffnessCoefficients[1]);
			psa(1) = this->principalSemiAxes(1) - pow(fabs(f(1))/this->stiffnessCoefficients[0], 1.0/this->stiffnessCoefficients[1]);
			psa(2) = this->principalSemiAxes(2) - pow(fabs(f(2))/this->stiffnessCoefficients[0], 1.0/this->stiffnessCoefficients[1]);
			break;
		default:
			psa = this->principalSemiAxes;
			break;
	}

	alpha = psa(0);
	beta = psa(1);
	gamma = psa(2);
	invR = 1;

	while((alpha < 1.0) || (beta < 1.0) || (gamma < 1.0))
	{
		invR *= 10.0;
		alpha = this->principalSemiAxes(0)*invR;
		beta = this->principalSemiAxes(1)*invR;
		gamma = this->principalSemiAxes(2)*invR;
	}

	R = 1.0/invR;
	A.row(0) = Eigen::Vector3d(1.0/alpha, 0.0, 0.0);
    A.row(1) = Eigen::Vector3d(0.0, 1.0/beta, 0.0);
    A.row(2) = Eigen::Vector3d(0.0, 0.0, 1.0/gamma);

    Af = A * f;
    AAf = A * A * f;
    invAt = A.inverse() * t;
    D = A.determinant();
    sigma = (D * D * invAt.squaredNorm()) - (R * R * Af.squaredNorm()); 
    K = (-(f.dot(t) / sqrt(f.dot(t) * f.dot(t))) / (sqrt(2.0) * R * D)) * sqrt(sigma + sqrt((sigma * sigma) + (4.0 * D * D * R * R * f.dot(t) * f.dot(t))));
    detG = K * ((K * K * D * D) + Af.squaredNorm());

    if(K != 0)
    	csps.PoC = (1.0 / detG) * ((K * K * D * D * A.inverse() * A.inverse() * t) + (K * AAf.cross(t)) + (f.dot(t) * f));
	else
		return csps;//TO DO: Wrench Axis Method
	
	csps.n = A * A * csps.PoC;
	csps.n.normalize();
	
	csps.Tpoc = K * A * A * csps.PoC;
	
	csps.Fn = f.dot(csps.n) * csps.n;
	csps.Ft = f - csps.Fn;
	
	csps.FtFnRatio = csps.Ft.norm() / csps.Fn.norm();
	csps.TnFnRatioLin = csps.Tpoc.norm() / csps.Fn.norm();
	csps.TnFnRatio = csps.Tpoc.norm() / pow(csps.Fn.norm(), 4.0/3.0); 

	return csps;
}
Surfaces::ContactSensingProblemSolution Surfaces::Ellipsoidal::Ellipsoid::solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;
	Eigen::Vector3d f;
	Eigen::Vector3d t;

	f << fx, fy, fz;
	t << tx, ty, tz;

	csps = this->solveContactSensingProblem(f, t, forceThreshold);

	return csps;
}
void Surfaces::Ellipsoidal::Ellipsoid::setPrincipalSemiAxes(double psaX, double psaY, double psaZ)
{
	this->principalSemiAxes << psaX, psaY, psaZ;
}
void Surfaces::Ellipsoidal::Ellipsoid::setPrincipalSemiAxes(Eigen::Vector3d psa)
{
	this->principalSemiAxes = psa;
}
void Surfaces::Ellipsoidal::Ellipsoid::getPrincipalSemiAxes(double& psaX, double& psaY, double& psaZ)
{
	psaX = this->principalSemiAxes(0);
	psaY = this->principalSemiAxes(1);
	psaZ = this->principalSemiAxes(2);
}
Eigen::Vector3d Surfaces::Ellipsoidal::Ellipsoid::getPrincipalSemiAxes()
{
	return this->principalSemiAxes;
}
/*
	Surfaces
		NURBS
*/
Surfaces::NURBS::NURBS()
{
	this->surfaceType = Surfaces::SurfaceType::NURBS;
	this->stiffnessType = Surfaces::StiffnessType::Rigid;
	this->orientation = Eigen::Matrix3d::Zero();
	this->displacement = Eigen::Vector3d::Zero();
	this->stiffnessCoefficients.clear();
	this->pointsFile = "";
}
Surfaces::NURBS::NURBS(NURBS& n)
{
	this->surfaceType = n.getSurfaceType();
	this->orientation = n.getOrientation();
	this->displacement = n.getDisplacement();
	std::vector<double> params;
	this->stiffnessType = n.getStiffness(params);
	this->stiffnessCoefficients = params;
	this->pointsFile = n.getPointsFile();
}
Surfaces::ContactSensingProblemSolution Surfaces::NURBS::solveContactSensingProblem(Eigen::Vector3d f, Eigen::Vector3d t, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;

	csps.F = f;
	csps.T = t;
	csps.Fs = this->orientation.transpose() * f;
	csps.Ts = this->orientation.transpose() * (t - this->displacement.cross(f));
	f = csps.Fs;
	t = csps.Ts;
	csps.PoC = Eigen::Vector3d::Zero();
	csps.n = Eigen::Vector3d::Zero();
	csps.Tpoc = Eigen::Vector3d::Zero();
	csps.Fn = Eigen::Vector3d::Zero();
	csps.Ft = Eigen::Vector3d::Zero();
	csps.FtFnRatio = 0.0;
	csps.TnFnRatioLin = 0.0;
	csps.TnFnRatio = 0.0; 

	if(f.norm() < forceThreshold)
		return csps;

	//TO DO

	return csps;
}
Surfaces::ContactSensingProblemSolution Surfaces::NURBS::solveContactSensingProblem(double fx, double fy, double fz, double tx, double ty, double tz, double forceThreshold)
{
	Surfaces::ContactSensingProblemSolution csps;
	Eigen::Vector3d f;
	Eigen::Vector3d t;

	f << fx, fy, fz;
	t << tx, ty, tz;

	csps = this->solveContactSensingProblem(f, t, forceThreshold);

	return csps;
}
void Surfaces::NURBS::setPointsFile(std::string pf)
{
	this->pointsFile = pf;
}
std::string Surfaces::NURBS::getPointsFile()
{
	return this->pointsFile;
}