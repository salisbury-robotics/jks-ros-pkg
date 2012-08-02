uniform float g_scale;
varying vec3  v_position;

void main(void)
{
    float d = length(v_position) * g_scale;
    gl_FragColor = vec4(d, d, g_scale, 1.0);
}

