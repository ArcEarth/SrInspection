#define REGISTER(b) :register(b)

#include "ShadowMapEffectCBuffer.hlsli"
// Input control point
#include "ShadowMapEffectStructures.hlsli"

#include "Common.hlsli"
// Output patch constant data.
struct HS_CONSTANT_DATA_OUTPUT
{
	float EdgeTessFactor[3] : SV_TessFactor; // e.g. would be [4] for a quad domain
	float InsideTessFactor : SV_InsideTessFactor; // e.g. would be Inside[2] for a quad domain

	// PN triangle control points
	float3 b210 : CONTROLPOINTS0;
    float3 b120 : CONTROLPOINTS1;
    float3 b021 : CONTROLPOINTS2;
    float3 b012 : CONTROLPOINTS3;
    float3 b102 : CONTROLPOINTS4;
    float3 b201 : CONTROLPOINTS5;
    float3 b111 : CONTROLPOINTS6;
    float3 n110 : CONTROLPOINTS7;
    float3 n011 : CONTROLPOINTS8;
    float3 n101 : CONTROLPOINTS9;
};

#define NUM_CONTROL_POINTS 3

inline float _wij(HSControlPointTex vi, HSControlPointTex vj)
{
	return dot(vj.Position - vi.Position, vi.Normal);
}
 
inline float _vij(HSControlPointTex vi, HSControlPointTex vj)
{
	float3 Pj_minus_Pi = vi.Position - vj.Position;
	float3 Ni_plus_Nj = vi.Normal + vj.Normal;
	return 2.0 * dot(Pj_minus_Pi, Ni_plus_Nj) / dot(Pj_minus_Pi, Pj_minus_Pi);
}

#define wij(I, J) _wij(ip[I], ip[J])
#define vij(I, J) _vij(ip[I], ip[J])

//void CaculatePNControlPoints(inout HS_CONSTANT_DATA_OUTPUT Output, float3 P0, float3 P1, float3 P2, float3 N0, float3 N1, float3 N2){}
// Patch Constant Function
HS_CONSTANT_DATA_OUTPUT CalcHSPatchConstants(
	InputPatch<HSControlPointTex, NUM_CONTROL_POINTS> ip,
	uint PatchID : SV_PrimitiveID)
{
	HS_CONSTANT_DATA_OUTPUT Output;

	// set base 
	float3 P0 = ip[0].Position;
	float3 P1 = ip[1].Position;
	float3 P2 = ip[2].Position;
	float3 N0 = normalize(ip[0].Normal);
	float3 N1 = normalize(ip[1].Normal);
	float3 N2 = normalize(ip[2].Normal);

	float4 sp0 = mul(float4(P0, 1.0f), ViewProjection);
	float4 sp1 = mul(float4(P1, 1.0f), ViewProjection);
	float4 sp2 = mul(float4(P2, 1.0f), ViewProjection);
	sp0 /= sp0.w;
	sp1 /= sp1.w;
	sp2 /= sp2.w;
	
	// Caculate tessellation factor based on screen-space distance
	Output.EdgeTessFactor[0] = min(TessellationFactor, dot(abs(sp1.xy - sp2.xy), TessellationScreenFactor));
	Output.EdgeTessFactor[1] = min(TessellationFactor, dot(abs(sp2.xy - sp0.xy), TessellationScreenFactor));
	Output.EdgeTessFactor[2] = min(TessellationFactor, dot(abs(sp0.xy - sp1.xy), TessellationScreenFactor));
	Output.InsideTessFactor = (Output.EdgeTessFactor[0] + Output.EdgeTessFactor[1] + Output.EdgeTessFactor[2])/3.0f;
 
	// compute control points
	Output.b210 = (2.0f * P0 + P1 - wij(0, 1) * N0) / 3.0f;
	Output.b120 = (2.0f * P1 + P0 - wij(1, 0) * N1) / 3.0f;
	Output.b021 = (2.0f * P1 + P2 - wij(1, 2) * N1) / 3.0f;
	Output.b012 = (2.0f * P2 + P1 - wij(2, 1) * N2) / 3.0f;
	Output.b102 = (2.0f * P2 + P0 - wij(2, 0) * N2) / 3.0f;
	Output.b201 = (2.0f * P0 + P2 - wij(0, 2) * N0) / 3.0f;
	float3 E = (Output.b210 + Output.b120 + Output.b021 + Output.b012 + Output.b102 + Output.b201) / 6.0f;
	float3 V = (P0 + P1 + P2) / 3.0f;
	Output.b111 = E + (E - V) * 0.5f;
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
HSControlPointTex PN_HS_Tex(
	InputPatch<HSControlPointTex, NUM_CONTROL_POINTS> ip,
	uint i : SV_OutputControlPointID,
	uint PatchID : SV_PrimitiveID)
{
	HSControlPointTex Output;

	// Insert code to compute Output here
	Output.Position = ip[i].Position;
	Output.Normal = ip[i].Normal;
	Output.TexCoord = ip[i].TexCoord;

	return Output;
}

[domain("tri")]
PSInputOneLightTex PN_DS_OneLightTex(
	HS_CONSTANT_DATA_OUTPUT input,
	float3 uvw : SV_DomainLocation,
	const OutputPatch<HSControlPointTex, NUM_CONTROL_POINTS> patch)
{
	PSInputOneLightTex vout;

	float3 uvw2 = uvw * uvw; // uvw squared
	float3 uvw3 = uvw2 * uvw; // uvw cubic
	float3 n200 = patch[0].Normal;
	float3 n020 = patch[1].Normal;
	float3 n002 = patch[2].Normal;
	float3 b300 = patch[0].Position;
	float3 b030 = patch[1].Position;
	float3 b003 = patch[2].Position;

	// normal
	float3 barNormal = uvw[2] * patch[0].Normal
				+ uvw[0] * patch[1].Normal
				+ uvw[1] * patch[2].Normal;
	float3 pnNormal = n200 * uvw2[2]
				+ n020 * uvw2[0]
				+ n002 * uvw2[1]
				+ input.n110 * uvw[2] * uvw[0]
				+ input.n011 * uvw[0] * uvw[1]
				+ input.n101 * uvw[2] * uvw[1];
	vout.normal = barNormal; //lerp(barNormal, pnNormal, TessellationAlpha);
 
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
	float4 posWorld = float4(lerp(barPos, pnPos, TessellationAlpha),1.0f);

    vout.pos = mul(posWorld, ViewProjection);

    vout.uv = uvw[2] * patch[0].TexCoord + uvw[0] * patch[1].TexCoord + uvw[1] * patch[2].TexCoord;
 
    vout.pos_ws = posWorld.xyz;

    SetLightUVOne;

    return vout;
}
