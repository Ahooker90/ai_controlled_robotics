Shader "Custom/DepthGrayscaleShader"
{
    Properties
    {
        _DepthScale ("Depth Scale", Float) = 1.0
        _DepthOffset ("Depth Offset", Float) = 0.0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }

        Pass
        {
            ZTest Always Cull Off ZWrite Off

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            sampler2D _CameraDepthTexture;
            float _DepthScale;
            float _DepthOffset;

            struct v2f
            {
                float4 pos : SV_POSITION;
                float2 uv  : TEXCOORD0;
            };

            v2f vert(appdata_full v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv  = v.texcoord;
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                // Sample the depth texture
                float rawDepth = SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.uv);

                // Linearize the depth using the built-in function
                float linearDepth = LinearEyeDepth(rawDepth);

                // Normalize depth to 0-1 range and apply scale and offset
                float depth = saturate((linearDepth + _DepthOffset) * _DepthScale);

                // Invert depth so closer objects are brighter
                float intensity = 1.0 - depth;

                // Output the grayscale color
                return float4(intensity, intensity, intensity, 1.0);
            }
            ENDCG
        }
    }
    FallBack Off
}

