#include "FSiteGenerator.h"
#include <random>
#include <time.h>
#include "FBoundingBox.h"
void FSiteGenerator::ImpactAABBoxDamage(FBoundingBox& box, int num, std::vector<FVec3>& sites, RandomType type, FVec3 transform)
{

	FVec3& min = box.m_Min;
	FVec3& max = box.m_Max;
	FVec3 center = (max + min) * 0.5;
	FVec3 newSite;
	for (int i = 0; i < num; i++) {

		switch (type)
		{
		case NORMAL:
			newSite = FVec3(RandomNumber(min.X, max.X), RandomNumber(min.Y, max.Y), RandomNumber(min.Z, max.Z));
		case GAUSSION:
			newSite = FVec3(GaussianRandom(center.X, box.m_Size.X / 4), GaussianRandom(center.Y, box.m_Size.Y / 4), GaussianRandom(center.Z, box.m_Size.Z / 4));
		default:
			break;
		}
		sites.push_back(newSite+transform);
	}
}

void FSiteGenerator::ImpactSphereDamage(FVec3& pos,FFLOAT radius, int num, std::vector<FVec3> &output, RandomType type, FVec3 transform)
{
	FVec3 min(pos.X - radius, pos.Y - radius, pos.Z - radius);
	FVec3 max(pos.X + radius, pos.Y + radius, pos.Z + radius);
	FVec3 newSite;
	for (int i = 0; i < num; ) {

		switch (type)
		{
		case NORMAL:
			newSite=FVec3(RandomNumber(min.X, max.X), RandomNumber(min.Y, max.Y), RandomNumber(min.Z, max.Z));
		case GAUSSION:
			newSite=FVec3(GaussianRandom(pos.X, radius / 4), GaussianRandom(pos.Y, radius / 4), GaussianRandom(pos.Z, radius / 4));
		default:
			break;
		}
		
		
		double disq = newSite.DistanceSqr(pos);
		if (disq <= radius * radius) {
			//printf("%f,%f,%f\n", newSite.X, newSite.Y, newSite.Z);
			output.push_back(newSite-transform);
			i++;
		}
	}
}

void FSiteGenerator::ImpactPlaneDamage(DirectX::BoundingBox&box,FVec3& pos, FVec3& normals,  int num, std::vector<FVec3>& sites, FVec3 transform)
{
	FVec3 min(box.Center.x - box.Extents.x, box.Center.y - box.Extents.y, box.Center.z - box.Extents.z);
	FVec3 max(box.Center.x + box.Extents.x, box.Center.y + box.Extents.y, box.Center.z + box.Extents.z);

	FVec3 newSite;
	for (int i = 0; i < num; ) {

		newSite = FVec3(RandomNumber(min.X, max.X), RandomNumber(min.Y, max.Y), RandomNumber(min.Z, max.Z));

		FVec3 temp = newSite - pos + transform;
		double dot = normals.Dot(temp);
		if (dot <= 0) {
			sites.push_back(newSite);
			i++;
		}

	}
}

