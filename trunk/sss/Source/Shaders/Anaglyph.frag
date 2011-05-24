// --------------------------------------------------------------------------
// Stanford BioRobotics Simulation Suite
//
// GLSL fragment program that combines a left and right stereo pair to form
// an anaglyphic 3D image.
//
// Author:  Sonny Chan
// Date:    October 2010
// --------------------------------------------------------------------------

uniform sampler2D   g_stereoLeft;	// left image
uniform sampler2D   g_stereoRight;  // right image
uniform float       g_parallax;     // maximum parallax
uniform vec4        g_viewport;     // x, y, w, h of the current viewport

const mat3 Bl = mat3( 0.456,-0.040,-0.015,
                      0.500,-0.378,-0.021,
                      0.176,-0.016,-0.005 );
const mat3 Br = mat3(-0.043, 0.378,-0.072,
                     -0.088, 0.734,-0.113,
                     -0.002,-0.018, 1.226 );

void main(void)
{
    vec2 fcoord = (gl_FragCoord.xy - g_viewport.xy) / g_viewport.pq;
    vec2 shift  = vec2(0.5 * g_parallax, 0.0);
    
    vec3 lcolor = texture2D(g_stereoLeft, fcoord + shift).rgb;
    vec3 rcolor = texture2D(g_stereoRight, fcoord - shift).rgb;
    
//    vec3 blended = Bl * lcolor + Br * rcolor;
//	gl_FragColor = vec4(blended, 1.0);    
	
	gl_FragColor = vec4(0.7 * lcolor.g + 0.3 * lcolor.b, rcolor.g, rcolor.b, 1.0);
	gl_FragColor = vec4(lcolor.r, rcolor.g, rcolor.b, 1.0);
}

// --------------------------------------------------------------------------

