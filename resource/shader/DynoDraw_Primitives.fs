#version 330 core

out vec4 Out_FragColor;

in vec3 VS_Color;
in vec3 VS_Normal;

uniform bool IsTransparent = false;

void main()
{
    float Intensity = 0.2;

    Intensity += (0.8 *
                  max(dot(-vec3( -0.577, -0.577, -0.577),
                          VS_Normal), 0.0));

    float Alpha = 1.0;
    if (IsTransparent)
    {
        Alpha = 0.4;
    }
    Out_FragColor = vec4(Intensity * VS_Color, Alpha);
}

