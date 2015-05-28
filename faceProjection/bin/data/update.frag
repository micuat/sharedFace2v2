#version 120
#extension GL_ARB_texture_rectangle : enable

// ping pong inputs
uniform sampler2DRect particles0;
uniform sampler2DRect particles1;
uniform sampler2DRect particles2;

uniform vec3 mouse;
uniform float radiusSquared;
uniform float elapsed;

void main()
{
    vec3 pos = texture2DRect(particles0, gl_TexCoord[0].st).xyz;
    vec3 vel = texture2DRect(particles1, gl_TexCoord[0].st).xyz;
    vec3 orgPos = texture2DRect(particles2, gl_TexCoord[0].st).xyz;

    // mouse attraction
    vec3 direction = mouse - pos.xyz;
    float distSquared = dot(direction, direction);
    float magnitude = 500 * (1.0 - distSquared / radiusSquared);
    vec3 force = step(distSquared, radiusSquared) * magnitude * normalize(direction);
    
    // gravity
//    force += vec3(0.0, -0.5, 0.0);

	// go back
	force += 20 * (orgPos.xyz - pos.xyz);
    
    // accelerate
    vel += elapsed * force;
    
    // bounce off the sides
//    vel.x *= step(abs(pos.x), 512.0) * 2.0 - 1.0;
//    vel.y *= step(abs(pos.y), 384.0) * 2.0 - 1.0;
    
    // damping
    vel *= 0.9;
    
    // move
    pos += elapsed * vel;
    
    gl_FragData[0] = vec4(pos, 1.0);
    gl_FragData[1] = vec4(vel, 0.0);
    gl_FragData[2] = vec4(orgPos, 0.0);
}