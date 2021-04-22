#include "Basic.hlsli"
static int index = 0;

// 像素着色器(2D)
float4 PS_2D(VertexPosHTex pIn) : SV_Target
{
    //texs.Sample(g_SamLinear, pIn.Tex);
    return texs.Sample(g_SamLinear, float3(pIn.Tex,2));
    //return g_Tex1.Sample(g_SamLinear,pIn.Tex);

}