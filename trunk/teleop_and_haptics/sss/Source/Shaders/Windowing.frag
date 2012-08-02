// --------------------------------------------------------------------------
// Stanford BioRobotics Simulation Suite
//
// GLSL fragment program that performs windowing on medical images.
//
// Author:  Sonny Chan
// Date:    March 2010
// --------------------------------------------------------------------------

uniform float windowCenter;	// window center in [0,1] range
uniform float windowWidth;	// window width, should be > 0

uniform sampler2D image;	// the image to window, as a texture

void main(void)
{
	vec4 value = texture2D(image, gl_TexCoord[0].st);
	float start = windowCenter - 0.5 * windowWidth;
	gl_FragColor = vec4((value.rgb - start) / windowWidth, value.a);
}

// --------------------------------------------------------------------------

