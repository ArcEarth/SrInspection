#define MAX_LIGHTS 4
#define MAX_BONES 72

#ifndef REGISTER
#define REGISTER(b)
#endif

cbuffer ShadowMapEffectCBuffer REGISTER(b0)
{
	float4x4 World;
	float4x4 ViewProjection;
	float4x4 UVTransform; // 12
	float3   EyePosition;
	float	 Bias;	//13
	
	float	 TessellationAlpha;
	float	 TessellationFactor;  // Max of tessellation
	float2	 TessellationScreenFactor; // 14

	float4   FogColor;
	float4   FogVector;

	float4	 MaterialAmbient;
	float4	 MaterialDiffuse;
	float3   MaterialSpecular;
	float    MaterialSpecularPower;
	float4   MaterialEmissive; //20

	float4	 AmbientLight; //21
	float4   LightColor[MAX_LIGHTS];
	float4	 LightAttenuation[MAX_LIGHTS];
	float4	 LightPosition[MAX_LIGHTS]; //33
	float4x4 LightViewProjection[MAX_LIGHTS]; //49
	float4	 LightDirection[MAX_LIGHTS]; //53
	float    LightBias[MAX_LIGHTS]; //54
	uint     IsPointLight[MAX_LIGHTS]; //55
}; // 880 Bytes in total
