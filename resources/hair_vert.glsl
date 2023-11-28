#version 120

attribute vec3 aPos;
uniform mat4 P;
uniform mat4 MV;
varying vec3 color;

void main()
{
    gl_Position = P * MV * vec4(aPos, 1);
    color = gl_Color.rgb;
}
