#include "Basic.hlsli"

static float theta = 0.0f;

// 顶点着色器(3D)
VertexPosHWNormalTex VS_3D(VertexPosNormalTex vIn)
{
    VertexPosHWNormalTex vOut;
    matrix viewProj = mul(g_View, g_Proj);
    float4 posW = mul(float4(vIn.PosL, 1.0f), g_World);

    vOut.PosH = mul(posW, viewProj);
    vOut.PosW = posW.xyz;
    vOut.NormalW = mul(vIn.NormalL, (float3x3) g_WorldInvTranspose);
    float2x2 rotate =
    {
        cos(theta), -sin(theta),
        sin(theta), cos(theta)
    };
    //vOut.Tex = mul(vIn.Tex - 0.5f, rotate)+0.5f;
    //vOut.Tex = vIn.Tex;
    vOut.Tex = mul(vIn.Tex - 0.5f, rotate)+0.5f;
    return vOut;
}
