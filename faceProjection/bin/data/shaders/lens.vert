#version 120

uniform float k1;
uniform float k2;
uniform float p1;
uniform float p2;
uniform float k3;

void main() {
    vec4 pos = gl_ModelViewMatrix * gl_Vertex;
    pos.x = pos.x / pos.z;
    pos.y = pos.y / pos.z;
    vec4 distorted = pos;
    float r2 = dot(pos.xy, pos.xy);
    float coeff = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    distorted.x = pos.x * coeff + 2 * p1 * pos.x * pos.y + p2 * (r2 + 2 * pos.x * pos.x);
    distorted.y = pos.y * coeff + 2 * p2 * pos.x * pos.y + p1 * (r2 + 2 * pos.y * pos.y);
    distorted.x *= pos.z;
    distorted.y *= pos.z;

    gl_Position = gl_ProjectionMatrix * distorted;
    vec4 col = gl_Color;
    gl_FrontColor = col;
//    gl_TexCoord[0] = gl_MultiTexCoord0;
}
