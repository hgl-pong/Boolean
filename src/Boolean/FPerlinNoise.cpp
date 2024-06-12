#include "FPerlinNoise.h"

#define B SAMPLE_SIZE
#define BM (SAMPLE_SIZE-1)

#define N 0x1000
#define NP 12   /* 2^N */
#define NM 0xfff

#define Curve(t) (t * t * (3.0f - 2.0f * t))
#define FLerp(t, a, b) (a + t * (b - a))

#define at2(rx,ry) (rx * q.X + ry * q.Y)
#define at3(rx,ry,rz) ( rx * q.X + ry * q.Y + rz * q.Z )

FPerlinNoise::FPerlinNoise(NoiseParams& params)
{
	m_Octaves = params.oxtavesNumbers;
	m_Frequency = params.frequency;
	m_Amplitude = params.amplitude;
	m_Seed = params.seed;
}

FPerlinNoise::~FPerlinNoise()
{
}

void FPerlinNoise::_Init()
{
	int i, j, k;

	for (i = 0; i < B; i++) {
		p[i] = i;
		g1[i] = (float)((rand() % (B + B)) - B) / B;
		g2[i].X = (float)((rand() % (B + B)) - B) / B;
		g2[i].Y = (float)((rand() % (B + B)) - B) / B;

		g2[i].Normalize();

		g3[i].X = (float)((rand() % (B + B)) - B) / B;
		g3[i].Y = (float)((rand() % (B + B)) - B) / B;
		g3[i].Z = (float)((rand() % (B + B)) - B) / B;

		g3[i].Normalize();
	}

	while (--i) {
		k = p[i];
		p[i] = p[j = rand() % B];
		p[j] = k;
	}

	for (i = 0; i < B + 2; i++) {
		p[B + i] = p[i];
		g1[B + i] = g1[i];

		g2[B + i] = g2[i];
		g3[B + i] = g3[i];

	}
}

float FPerlinNoise::Get(const FVec2& v)
{
	return _PerlinNoise2D(v);
}

float FPerlinNoise::Get(const FVec3& v)
{
	return _PerlinNoise3D(v);
}

float FPerlinNoise::_PerlinNoise3D(const FVec3& v)
{
	int terms = m_Octaves;
	float freq = m_Frequency;
	float result = 0.0f;
	float amp = m_Amplitude;

	FVec3 temp = v * m_Frequency;

	for (int i = 0; i < terms; i++) {
		result += _Noise3(temp) * amp;
		temp *= 2.0f;
		amp *= 0.5f;
	}

	return result;
}

float FPerlinNoise::_PerlinNoise2D(const FVec2& v)
{
	int terms = m_Octaves;
	float freq = m_Frequency;
	float result = 0.0f;
	float amp = m_Amplitude;

	FVec2 temp(v.X * m_Frequency, v.Y * m_Frequency);

	for (int i = 0; i < terms; i++) {
		result += _Noise2(temp) * amp;
		temp.X *= 2.0f;
		temp.Y *= 2.0f;
		amp *= 0.5f;
	}

	return result;
}

float FPerlinNoise::_Noise1(float arg)
{
	int bx0, bx1;
	float rx0, rx1, sx, t, u, v, vec[1];


	if (mStart) {
		srand(m_Seed);
		mStart = false;
		_Init();
	}

	t = arg + N; 
	bx0 = ((int)t) & BM; 
	bx1 = (bx0 + 1) & BM; 
	rx0 = t - (int)t; 
	rx1 = rx0 - 1.0f;

	sx = Curve(rx0);

	u = rx0 * g1[p[bx0]];
	v = rx1 * g1[p[bx1]];

	return FLerp(sx, u, v);
}

float FPerlinNoise::_Noise2(const FVec2& vec)
{
	int bx0, bx1, by0=0, by1=0, b00, b10, b01, b11;
	float rx0, rx1, ry0=0, ry1=0, sx, sy, a, b, t, u, v;
	FVec2 q;
	int i, j;

	if (mStart) {
		srand(m_Seed);
		mStart = false;
		_Init();
	}

	t = vec.X + N;
	bx0 = ((int)t) & BM;
	bx1 = (bx0 + 1) & BM;
	rx0 = t - (int)t;
	rx1 = rx0 - 1.0f;

	t = vec.Y + N;
	bx0 = ((int)t) & BM;
	bx1 = (bx0 + 1) & BM;
	rx0 = t - (int)t;
	rx1 = rx0 - 1.0f;

	i = p[bx0];
	j = p[bx1];

	b00 = p[i + by0];
	b10 = p[j + by0];
	b01 = p[i + by1];
	b11 = p[j + by1];

	sx = Curve(rx0);
	sy = Curve(ry0);

	q = g2[b00];
	u = at2(rx0, ry0);
	q = g2[b10];
	v = at2(rx1, ry0);
	a = FLerp(sx, u, v);

	q = g2[b01];
	u = at2(rx0, ry1);
	q = g2[b11];
	v = at2(rx1, ry1);
	b = FLerp(sx, u, v);

	return FLerp(sy, a, b);
}

float FPerlinNoise::_Noise3(const FVec3& vec)
{
	int bx0, bx1, by0=0, by1=0, bz0=0, bz1=0, b00, b10, b01, b11;
	float rx0, rx1, ry0=0, ry1=0, rz0=0, rz1=0, sy, sz, a, b, c, d, t, u, v;
	FVec3 q;
	int i, j;

	if (mStart) {
		srand(m_Seed);
		mStart = false;
		_Init();
	}

	t = vec.X + N;
	bx0 = ((int)t) & BM;
	bx1 = (bx0 + 1) & BM;
	rx0 = t - (int)t;
	rx1 = rx0 - 1.0f;

	t = vec.Y + N;
	bx0 = ((int)t) & BM;
	bx1 = (bx0 + 1) & BM;
	rx0 = t - (int)t;
	rx1 = rx0 - 1.0f;

	t = vec.Z + N;
	bx0 = ((int)t) & BM;
	bx1 = (bx0 + 1) & BM;
	rx0 = t - (int)t;
	rx1 = rx0 - 1.0f;

	i = p[bx0];
	j = p[bx1];

	b00 = p[i + by0];
	b10 = p[j + by0];
	b01 = p[i + by1];
	b11 = p[j + by1];

	t = Curve(rx0);
	sy = Curve(ry0);
	sz = Curve(rz0);

	q = g3[b00 + bz0]; u = at3(rx0, ry0, rz0);
	q = g3[b10 + bz0]; v = at3(rx1, ry0, rz0);
	a = FLerp(t, u, v);

	q = g3[b01 + bz0]; u = at3(rx0, ry1, rz0);
	q = g3[b11 + bz0]; v = at3(rx1, ry1, rz0);
	b = FLerp(t, u, v);

	c = FLerp(sy, a, b);

	q = g3[b00 + bz1]; u = at3(rx0, ry0, rz1);
	q = g3[b10 + bz1]; v = at3(rx1, ry0, rz1);
	a = FLerp(t, u, v);

	q = g3[b01 + bz1]; u = at3(rx0, ry1, rz1);
	q = g3[b11 + bz1]; v = at3(rx1, ry1, rz1);
	b = FLerp(t, u, v);

	d = FLerp(sy, a, b);

	return FLerp(sz, c, d);
}
