struct HSControlPointNoTex
{
	float3 Position : WORLDPOS;
	float3 Normal   : NORMAL;
};

struct HSControlPointTex
{
	float3 Position : WORLDPOS;
	float3 Normal   : NORMAL;
	float2 TexCoord : TEXCOORD0;
};

struct HSInputTexBump
{
	float3 Position : WORLDPOS;
	float3 Normal   : NORMAL;
	float3 Tangent  : TANGENT;
	float3 Binormal : BINORMAL;
	float2 TexCoord : TEXCOORD0;
};

struct PSInputOneLightNoTex
{
	float4 pos : SV_POSITION;
	float3 normal : NORMAL;
	float3 toEye : TEXCOORD1;
	float4 lightUv[1] : TEXCOORD2;
};

struct PSInputTwoLightNoTex
{
	float4 pos : SV_POSITION;
	float3 normal : NORMAL;
	float3 toEye : TEXCOORD1;
	float4 lightUv[2] : TEXCOORD2;
};

struct PSInputThreeLightNoTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : NORMAL;
	float3 toEye : TEXCOORD2;
	float4 lightUv[3] : TEXCOORD2;
};

struct PSInputFourLightNoTex
{
	float4 pos : SV_POSITION;
	float3 normal : NORMAL;
	float3 toEye : TEXCOORD1;
	float4 lightUv[4] : TEXCOORD2;
};

struct PSInputOneLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : NORMAL;
	float3 toEye : TEXCOORD1;
	float4 lightUv[1] : TEXCOORD2;
};

struct PSInputOneLightTexBump
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : NORMAL;
	float3 toEye : TEXCOORD2;
	float4 lightUv[1] : TEXCOORD3;
	float3 tangent : TANGENT;
	float3 binormal : BINORMAL;
};


struct PSInputTwoLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : NORMAL;
	float3 tangent : TANGENT;
	float3 binormal : BINORMAL;
	float3 toEye : TEXCOORD1;
	float4 lightUv[2] : TEXCOORD2;
};

struct PSInputThreeLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : NORMAL;
	float3 tangent : TANGENT;
	float3 binormal : BINORMAL;
	float3 toEye : TEXCOORD1;
	float4 lightUv[3] : TEXCOORD2;
};

struct PSInputFourLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : NORMAL;
	float3 tangent : TANGENT;
	float3 binormal : BINORMAL;
	float3 toEye : TEXCOORD1;
	float4 lightUv[4] : TEXCOORD2;
};

struct PSInputBinaryOneLightNoTex
{
	float4 pos : SV_POSITION;
	float4 lightUv[1] : TEXCOORD0;
};

struct PSInputBinaryTwoLightNoTex
{
	float4 pos : SV_POSITION;
	float4 lightUv[2] : TEXCOORD0;
};

struct PSInputBinaryThreeLightNoTex
{
	float4 pos : SV_POSITION;
	float4 lightUv[3] : TEXCOORD0;
};

struct PSInputBinaryFourLightNoTex
{
	float4 pos : SV_POSITION;
	float4 lightUv[4] : TEXCOORD0;
};


struct PSInputBinaryOneLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 lightUv[1] : TEXCOORD1;
};

struct PSInputBinaryTwoLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 lightUv[2] : TEXCOORD1;
};

struct PSInputBinaryThreeLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 lightUv[3] : TEXCOORD1;
};

struct PSInputBinaryFourLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 lightUv[4] : TEXCOORD0;
};

struct PSInputScreenSpaceNoTex
{
	float4 pos : SV_POSITION;
	float4 posUV : TEXCOORD0;
	float3 normal : NORMAL;
	float3 toEye : TEXCOORD1;
};

struct PSInputScreenSpaceTex
{
	float4 pos			: SV_POSITION;
	float2 uv			: TEXCOORD0;
	float4 posUV        : TEXCOORD1;
	float3 normal		: NORMAL;
	float3 toEye        : TEXCOORD3;
	float3 tangent      : TANGENT;
	float3 binormal     : BINORMAL;
};
