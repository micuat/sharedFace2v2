#version 120

uniform sampler2DRect texture1;

void main(){
    vec2 pos = gl_TexCoord[0].st;

    gl_FragColor = texture2DRect(texture1, gl_TexCoord[0].st);
}
