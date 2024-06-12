#ifndef FSITE_GENERATOR_H
#define FSITE_GENERATOR_H
#include "vec.h"
#include <vector>
#include <DirectXCollision.h>
#include <ctime>
enum RandomType
{
	NORMAL,
	GAUSSION,
};
class FBoundingBox;	
inline FFLOAT RandomNumber(FFLOAT min, FFLOAT max)
{	
	//srand(time(0));
	return min + FFLOAT(rand()) / RAND_MAX * (max - min);
}	
inline FFLOAT GaussianRandom(FFLOAT mu, FFLOAT sigma)
	{
		//const double epsilon = (std::numeric_limits<double>::min)();
		const FFLOAT two_pi = 2.0 * 3.14159265358979323846;

		static FFLOAT z0, z1;
		static bool generate;
		generate = !generate;

		if (!generate)
			return z1 * sigma + mu;

		double u1, u2;
		do
		{
			u1 = rand() * (1.0 / RAND_MAX);
			u2 = rand() * (1.0 / RAND_MAX);
		} while (u1 <= FLOAT_EPSILON);

		z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
		z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
		return z0 * sigma + mu;
	}
class FSiteGenerator
{
public:
	static void ImpactAABBoxDamage(FBoundingBox&box, int num, std::vector<FVec3>&sites,RandomType type=NORMAL,FVec3 transform= FVec3(0, 0, 0));
	static void ImpactSphereDamage(FVec3& pos, FFLOAT radius, int num, std::vector<FVec3>& sites, RandomType type = NORMAL,FVec3 transform=FVec3(0, 0, 0));
	static void ImpactPlaneDamage(DirectX::BoundingBox& box,FVec3& pos, FVec3& normals, int num, std::vector<FVec3>& sites,FVec3 transform= FVec3(0, 0, 0));



};

#endif//FSITE_GENERATOR_H
