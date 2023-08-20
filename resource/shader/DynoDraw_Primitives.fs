#version 330 core

out vec4 Out_FragColor;

in vec3 VS_Color;
in vec3 VS_Normal;

void main()
{
    float Intensity = 0.2;

    Intensity += (0.8 *
                  max(dot(-vec3( -0.577, -0.577, -0.577),
                          VS_Normal), 0.0));

    Out_FragColor = vec4(Intensity * VS_Color, 1.0);
}

