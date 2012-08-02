varying vec3 v_position;

void main(void)
{
    vec4 v = gl_ModelViewMatrix * gl_Vertex;
    v_position = v.xyz;
    gl_Position = gl_ProjectionMatrix * v;
}

