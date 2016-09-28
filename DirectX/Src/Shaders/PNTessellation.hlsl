#define REGISTER(b) : register(b)
#include "ShadowMapEffectCBuffer.hlsli"
// Input control point
#include "ShadowMapEffectStructures.hlsli"

// Output patch constant data.
struct HS_CONSTANT_DATA_OUTPUT
{
	float EdgeTessFactor[3] : SV_TessFactor; // e.g. would be [4] for a quad domain
	float InsideTessFactor : SV_InsideTessFactor; // e.g. would be Inside[2] for a quad domain

	// PN triangle control points
	float3 b210;
	float3 b120;
	float3 b021;
	float3 b012;
	float3 b102;
	float3 b201;
	float3 b111;
	float3 n110;
	float3 n011;
	float3 n101;
};

#define NUM_CONTROL_POINTS 3

inline float _wij(PSInputOneLightTex vi, PSInputOneLightTex vj)
{
	return (dot(vj.pos - vi.pos, vi.normal));
}
 
inline float _vij(PSInputOneLightTex vi, PSInputOneLightTex vj)
{
	float3 Pj_minus_Pi = vi.pos - vj.pos;
	float3 Ni_plus_Nj = vi.normal + vj.normal;
	return 2.0 * dot(Pj_minus_Pi, Ni_plus_Nj) / dot(Pj_minus_Pi, Pj_minus_Pi);
}

#define wij(I, J) _wij(ip[I], ip[J])
#define vij(I, J) _vij(ip[I], ip[J])

// Patch Constant Function
HS_CONSTANT_DATA_OUTPUT CalcHSPatchConstants(
	InputPatch<PSInputOneLightTex, NUM_CONTROL_POINTS> ip,
	uint PatchID : SV_PrimitiveID)
{
	HS_CONSTANT_DATA_OUTPUT Output;

	// Insert code to compute Output here
	Output.EdgeTessFactor[0] =
	Output.EdgeTessFactor[1] =
	Output.EdgeTessFactor[2] =
	Output.InsideTessFactor = 15; // e.g. could calculate dynamic tessellation factors instead

	// set base 
	float3 P0 = ip[0].pos;
	float3 P1 = ip[1].pos;
	float3 P2 = ip[2].pos;
	float3 N0 = ip[0].normal;
	float3 N1 = ip[1].normal;
	float3 N2 = ip[2].normal;
 
 // compute control points
	Output.b210 = (2.0 * P0 + P1 - wij(0, 1) * N0) / 3.0;
	Output.b120 = (2.0 * P1 + P0 - wij(1, 0) * N1) / 3.0;
	Output.b021 = (2.0 * P1 + P2 - wij(1, 2) * N1) / 3.0;
	Output.b012 = (2.0 * P2 + P1 - wij(2, 1) * N2) / 3.0;
	Output.b102 = (2.0 * P2 + P0 - wij(2, 0) * N2) / 3.0;
	Output.b201 = (2.0 * P0 + P2 - wij(0, 2) * N0) / 3.0;
	float E = (Output.b210
		   + Output.b120
		   + Output.b021
		   + Output.b012
		   + Output.b102
		   + Output.b201) / 6.0;
	float V = (P0 + P1 + P2) / 3.0;
	Output.b111 = E + (E - V) * 0.5;
	Output.n110 = N0 + N1 - vij(0, 1) * (P1 - P0);
	Output.n011 = N1 + N2 - vij(1, 2) * (P2 - P1);
	Output.n101 = N2 + N0 - vij(2, 0) * (P0 - P2);
	return Output;
}

[domain("tri")]
[partitioning("fractional_odd")]
[outputtopology("triangle_cw")]
[outputcontrolpoints(3)]
[patchconstantfunc("CalcHSPatchConstants")]
PSInputOneLightTex main(
	InputPatch<PSInputOneLightTex, NUM_CONTROL_POINTS> ip,
	uint i : SV_OutputControlPointID,
	uint PatchID : SV_PrimitiveID)
{
	PSInputOneLightTex Output;

	// Insert code to compute Output here
	Output.pos = ip[i].pos;
	Output.normal = ip[i].normal;
	Output.uv = ip[i].uv;

	return Output;
}

[domain("tri")]
PSInputOneLightTex PN_DS(
	HS_CONSTANT_DATA_OUTPUT input,
	float3 uvw : SV_DomainLocation,
	const OutputPatch<PSInputOneLightTex, NUM_CONTROL_POINTS> patch)
{
	PSInputOneLightTex Output;

	float3 uvw2 = uvw * uvw; // uvw squared
	float3 uvw3 = uvw2 * uvw; // uvw cubic
	float3 n200 = patch[0].normal;
	float3 n020 = patch[1].normal;
	float3 n002 = patch[2].normal;
	float3 b300 = patch[0].pos;
	float3 b030 = patch[1].pos;
	float3 b003 = patch[2].pos;

	// normal
	float3 barNormal = uvw[2] * patch[0].normal
				+ uvw[0] * patch[1].normal
				+ uvw[1] * patch[2].normal;
	float3 pnNormal = n200 * uvw2[2]
				+ n020 * uvw2[0]
				+ n002 * uvw2[1]
				+ input.n110 * uvw[2] * uvw[0]
				+ input.n011 * uvw[0] * uvw[1]
				+ input.n101 * uvw[2] * uvw[1];
	Output.normal = lerp(barNormal, pnNormal, TessellationAlpha);
 
	// compute interpolated pos
	float3 barPos = uvw[2] * b300
				  + uvw[0] * b030
				  + uvw[1] * b003;
 
	// save some computations
	uvw2 *= 3.0;
 
	// compute PN position
	float3 pnPos = b300 * uvw3[2]
			 + b030 * uvw3[0]
			 + b003 * uvw3[1]
			 + input.b210 * uvw2[2] * uvw[0]
			 + input.b120 * uvw2[0] * uvw[2]
			 + input.b201 * uvw2[2] * uvw[1]
			 + input.b021 * uvw2[0] * uvw[1]
			 + input.b102 * uvw2[1] * uvw[2]
			 + input.b012 * uvw2[1] * uvw[0]
			 + input.b111 * 6.0 * uvw[0] * uvw[1] * uvw[2];
 
	// final position and normal
	float3 finalPos = lerp(barPos, pnPos, TessellationAlpha);

	Output.pos = mul(float4(finalPos, 1.0), ViewProjection);

	Output.uv = uvw[2] * patch[0].uv + uvw[0] * patch[1].uv + uvw[1] * patch[2].uv;
	Output.lightUv[0] = uvw[2] * patch[0].lightUv[0] + uvw[0] * patch[1].lightUv[0] + uvw[1] * patch[2].lightUv[0];

	return Output;
}
