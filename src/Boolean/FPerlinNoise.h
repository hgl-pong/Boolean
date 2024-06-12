#ifndef FPERLIN_NOISE_H
#define FPERLIN_NOISE_H
#include "vec.h"

#define SAMPLE_SIZE 1024

struct NoiseParams
{
	int oxtavesNumbers;
	float frequency;
	float amplitude;
	int seed;
};

class FPerlinNoise
{
public:
	FPerlinNoise(NoiseParams&params);
	~FPerlinNoise();
	
	float Get(const FVec2& v);
	float Get(const FVec3& v);
private:
	void _Init();
	float _PerlinNoise3D(const FVec3& v);
	float _PerlinNoise2D(const FVec2& v);

	float _Noise1(float arg);
	float _Noise2(const FVec2& v);
	float _Noise3(const FVec3& v);
	
private:
	int m_Octaves;
	float m_Frequency;
	float m_Amplitude;
	int m_Seed;

	int p[SAMPLE_SIZE + SAMPLE_SIZE + 2];
	FVec3 g3[SAMPLE_SIZE + SAMPLE_SIZE + 2];
	FVec2 g2[SAMPLE_SIZE + SAMPLE_SIZE + 2];
	float g1[SAMPLE_SIZE + SAMPLE_SIZE + 2];
	bool mStart;
};


#endif // !FPERLIN_NOISE_H
