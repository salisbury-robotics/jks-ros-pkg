// --------------------------------------------------------------------------
// Stanford BioRobotics Simulation Suite
//
// GLSL fragment program that renders a labelled (segmented) image using
// a 1D colour lookup table.
//
// Author:  Sonny Chan
// Date:    June 2010
// --------------------------------------------------------------------------

uniform sampler2D image;	// the image to window, as a texture
uniform sampler1D lookup;   // the colour lookup table

void main(void)
{
	float value = texture2D(image, gl_TexCoord[0].st).r;
	vec4 colour = texture1D(lookup, value);
	gl_FragColor = colour;
}

// --------------------------------------------------------------------------

