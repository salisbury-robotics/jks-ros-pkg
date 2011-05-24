// -------------------------------------------------------------------------- //
// STANFORD BIOROBOTICS SURGICAL SIMULATION SUITE
//  Volume Ray Casting Fragment Program
//
// Authors: Sonny Chan, Joseph Lee
// Date:    2008-2010
//
// Notes:
//      The ray casting is set up in world space within the 0-1 cube, but
//  the texture is stored compactly without padding, and there exists an
//  anisotropic scaling between world coordinates and texture coordinates.
//  Before sampling a volume texture, simply multiply by the scale factor
//  uniform variable.  The gradient delta vector has already been multiplied.
//      The ray and rayStep variables are kept in texture space as well, but
//  this may change if needed (ie. if we introduce point light sources, or
//  need world coordinates for some other reason along the ray.
//
// -------------------------------------------------------------------------- //

// --- Global Constants & Uniforms ------------------------------------------ //

// primary volume
uniform sampler3D   g_volumePrimary;    // volume data
uniform sampler1D   g_transferPrimary;  // mip-mapped transfer function

// secondary volume
uniform sampler3D   g_volumeSecondary;  // volume data
uniform sampler1D   g_transferSecondary;// mip-mapped transfer function

// labels volume
uniform sampler3D   g_volumeLabels;     // luminance-alpha (LUT index, edge @.5)
uniform sampler1D   g_transferLabels;   // colour lookup table

// mask volume
uniform sampler3D   g_volumeMask;

// isosurface parameters
uniform float       g_isosurfaceValue;  // threshold in (0,1) range
uniform vec4        g_isosurfaceColor;  // RGBA colour to render the surface

// auxiliary information
uniform sampler2D   g_distances;        // s = dist to start, t = dist to terminate,
                                        // scale s,t by 1/p to obtain actual units
uniform float       g_step;             // ray integration step size
uniform vec3        g_textureScale;     // multiply to get to texture space

uniform vec3        g_viewer;           // position of viewer
uniform vec4        g_viewport;         // x, y, w, h of the current viewport

// opacity limit for early ray termination
const float         k_opacityLimit = 0.97;

// delta step for approximating the gradient
uniform float g_delta;
vec3 dx = vec3(g_delta, 0.0, 0.0) * g_textureScale;
vec3 dy = vec3(0.0, g_delta, 0.0) * g_textureScale;
vec3 dz = vec3(0.0, 0.0, g_delta) * g_textureScale;

// --- Intensity Sampling --------------------------------------------------- //

float intensity(sampler3D volume, vec3 p)
{
    // this version is a straight sample of the primary volume
//    return texture3D(volume, p).r;

    // this version multiplies the sample by the mask value at same location
    return texture3D(volume, p).r * texture3D(g_volumeMask, p).r;
}

// --- Gradient Estimation -------------------------------------------------- //

// assumes p is in texture space, returns gradient magnitude in the w channel
vec4 gradient(sampler3D volume, vec3 p)
{
    float gx = intensity(volume, p+dx) - intensity(volume, p-dx);
    float gy = intensity(volume, p+dy) - intensity(volume, p-dy);
    float gz = intensity(volume, p+dz) - intensity(volume, p-dz);
    vec3 g = vec3(gx, gy, gz);
    float l = length(g);
    return vec4(g / (l+0.001), l);
}

// --- Material Classification ---------------------------------------------- //

// approximates a pre-integrated transfer function lookup by taking a sample
// from a mipmapped transfer function
vec4 classify(vec2 intensities, sampler1D transfer)
{
    float avg = (intensities.s + intensities.t) * 0.5;
    float lod = log2(abs(intensities.s - intensities.t) * 4096.0 + 1.0);
    vec4 colour = texture1DLod(transfer, avg, lod);

    // transfer function is premultiplied, so divide out by the opacity
    return vec4(colour.rgb / (colour.a + 0.001), colour.a);
}

void adjust_alpha(inout vec4 colour)
{
    colour.a = 1.0 - pow(1.0 - colour.a, g_step * 100.0);
}

// --- Isosurface Determination --------------------------------------------- //

// perform interval bisection to find the isosurface
// direction is -1.0 for outside to in, and 1.0 for inside to out
void refine(inout vec4 ray, vec4 rstep, float direction, sampler3D volume)
{
    vec4 bstep = rstep * 0.5;
    ray -= bstep;
    for (int i = 0; i < 8; ++i) {
        float sample = intensity(volume, ray.xyz);
        bstep *= 0.5;
        float f = step(g_isosurfaceValue, sample) * 2.0 * direction + 1.0;
        ray += bstep * f;
    }
}

// --- Lighting & Shading --------------------------------------------------- //

vec4 phong(vec3 n, vec3 v, vec3 l, vec4 c, float a, float d, float s, float m)
{
    vec3 h = normalize(l+v);
    vec3 colour = c.rgb * (a + abs(dot(n,l)) * d)           // ambient & diffuse
                + vec3(1.0) * pow(abs(dot(n,h)), m) * s;    // white specular
    return vec4(colour * c.a, c.a);
}

// --- Program Entry Point -------------------------------------------------- //

void main(void)
{
    vec2 fcoord = (gl_FragCoord.xy - g_viewport.xy) / g_viewport.pq;
    vec4 dist = texture2D(g_distances, fcoord);
    dist.st /= dist.p;
    float rayLength = (dist.t - dist.s);
    vec3 rayDirection = normalize(gl_TexCoord[0].xyz - g_viewer);
    
    // set up the ray and ray step in world space
    // (w component stores the distance or length travelled in world space)
    vec4 rayStep = vec4(rayDirection, 1.0) * g_step;
    vec4 ray = vec4(g_viewer + rayDirection * dist.s, dist.s);

    // convert the ray and ray step to texture space for convenience/speed
    rayStep.xyz *= g_textureScale;
    ray.xyz     *= g_textureScale;

    // set up initial accumulation variables
    vec4 sum = vec4(0.0);

    // take one initial sample before entering the main loop
    float sample = intensity(g_volumePrimary, ray.xyz);
    bool inside = sample >= g_isosurfaceValue;

    while (ray.w < dist.t && sum.a < k_opacityLimit)
    {
        // find first hit on isosurface
        while (!inside && ray.w < dist.t)
        {
            ray += rayStep;
            sample = intensity(g_volumePrimary, ray.xyz);
            inside = sample >= g_isosurfaceValue;
        }

        if (inside)
        {
            // refine the isosurface hit point
            refine(ray, rayStep, -1.0, g_volumePrimary);

            if (ray.w < dist.t) {
                vec3 normal = gradient(g_volumePrimary, ray.xyz).xyz;
                vec4 lit = phong(normal, rayDirection, rayDirection,
                                 g_isosurfaceColor, 0.2, 0.6, 0.3, 20.0);
                sum += (1.0 - sum.a) * lit;
            }
        }

        // sample then advance the ray doing direct volume rendering
        sample = intensity(g_volumePrimary, ray.xyz);
        do {
            float previous = sample;
            ray += rayStep;
            sample = intensity(g_volumePrimary, ray.xyz);
            inside = sample >= g_isosurfaceValue;

            // check for ray termination
            if (ray.w >= dist.t) break;

            // classify the intensity
            vec4 colour = classify(vec2(previous, sample), g_transferPrimary);
            adjust_alpha(colour);
            vec4 grad = gradient( g_volumePrimary, (ray - 0.5*rayStep).xyz );
            vec4 lit = phong(grad.xyz, rayDirection, rayDirection,
                             colour, 0.2, 0.6, 0.3, 20.0);
            sum += (1.0 - sum.a) * lit;
        } while (inside && sum.a < k_opacityLimit);

        // find the back (exiting surface)
        if (!inside)
        {
            // refine the isosurface hit point
            refine(ray, rayStep, 1.0, g_volumePrimary);

            if (ray.w < dist.t) {
                vec3 normal = gradient(g_volumePrimary, ray.xyz).xyz;
                vec4 lit = phong(normal, rayDirection, rayDirection,
                                 g_isosurfaceColor, 0.2, 0.3, 0.0, 0.0);
                sum += (1.0 - sum.a) * lit;
            }
        }
    }

    // summed output has pre-multiplied alpha
    gl_FragColor = sum;


    // debug outputs
//    gl_FragColor = vec4(gl_TexCoord[0].xyz, 1.0);
//    gl_FragColor = vec4(rayLength, 0.0, 0.0, 1.0);
//    gl_FragColor = vec4(ray.xyz, 1.0);
//    gl_FragColor = texture1DLod(g_transferPrimary, fcoord.x, fcoord.y * 12.01);
}

// -------------------------------------------------------------------------- //
