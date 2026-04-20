// Color ramp functions for heatmap visualization

export const viridisRamp = `
vec3 viridis(float t) {
  const vec3 c0 = vec3(0.2777273272234177, 0.005407344544966578, 0.3340998053353061);
  const vec3 c1 = vec3(0.1050930431085774, 1.404613529898575, 1.384590162594685);
  const vec3 c2 = vec3(-0.3308618287255563, 0.214847559468213, 0.09509516302823659);
  const vec3 c3 = vec3(-4.634230498983486, -5.799100973351585, -19.33244095627987);
  const vec3 c4 = vec3(6.228269936347081, 14.17993336680509, 56.69055260068105);
  const vec3 c5 = vec3(4.776384997670288, -13.74514537774601, -65.35303263337234);
  const vec3 c6 = vec3(-5.435455855934631, 4.645852612178535, 26.3124352495832);
  
  return c0+t*(c1+t*(c2+t*(c3+t*(c4+t*(c5+t*c6)))));
}
`;

export const plasmaRamp = `
vec3 plasma(float t) {
  const vec3 c0 = vec3(0.05873234392399702, 0.02333670892565664, 0.5433401826748754);
  const vec3 c1 = vec3(2.176514634195958, 0.2383834171260182, -0.7539773609990764);
  const vec3 c2 = vec3(-2.689460476458034, -7.455851135738909, 3.110799939717086);
  const vec3 c3 = vec3(6.130348345893603, 42.3461881477227, -28.51885465332158);
  const vec3 c4 = vec3(-11.10743619062271, -82.66631109428045, 60.13984767418263);
  const vec3 c5 = vec3(10.02306557647065, 71.41361770095349, -54.07218655560067);
  const vec3 c6 = vec3(-3.658713842777788, -22.93153465461149, 18.19190778539828);
  
  return c0+t*(c1+t*(c2+t*(c3+t*(c4+t*(c5+t*c6)))));
}
`;

export const magmaRamp = `
vec3 magma(float t) {
  const vec3 c0 = vec3(-0.002136485053939582, -0.000749655052795221, -0.005386127855323933);
  const vec3 c1 = vec3(0.2516605407371642, 0.6775232436837668, 2.494026599312351);
  const vec3 c2 = vec3(8.353717279216625, -3.577719514958484, 0.3144679030132573);
  const vec3 c3 = vec3(-27.66873308576866, 14.26473078096533, -13.64921318813922);
  const vec3 c4 = vec3(52.17613981234068, -27.94360607168351, 12.94416944238394);
  const vec3 c5 = vec3(-50.76852536473588, 29.04658282127291, 4.23415299384598);
  const vec3 c6 = vec3(18.65570506591883, -11.48977351997711, -5.601961508734096);
  
  return c0+t*(c1+t*(c2+t*(c3+t*(c4+t*(c5+t*c6)))));
}
`;

export const infernoRamp = `
vec3 inferno(float t) {
  const vec3 c0 = vec3(0.0002189403691192265, 0.001651004631001012, -0.01948089843709184);
  const vec3 c1 = vec3(0.1065134194856116, 0.5639564367884091, 3.932712388889277);
  const vec3 c2 = vec3(11.60249308247187, -3.972853965665698, -15.9423941062914);
  const vec3 c3 = vec3(-41.70399613139459, 17.43639888205313, 44.35414519872813);
  const vec3 c4 = vec3(77.162935699498, -33.40235894210092, -81.80730925738993);
  const vec3 c5 = vec3(-71.31942824499214, 32.62606426397723, 73.20951985803202);
  const vec3 c6 = vec3(25.13112622477341, -12.24266895238567, -23.07032500287172);
  
  return c0+t*(c1+t*(c2+t*(c3+t*(c4+t*(c5+t*c6)))));
}
`;

export const jetRamp = `
vec3 jet(float t) {
  return clamp(vec3(
    1.5 - abs(4.0 * t - 3.0),
    1.5 - abs(4.0 * t - 2.0),
    1.5 - abs(4.0 * t - 1.0)
  ), 0.0, 1.0);
}
`;

export const grayscaleRamp = `
vec3 grayscale(float t) {
  return vec3(t);
}
`;

export const heatmapVertexShader = `
uniform float uPointSize;
attribute float intensity;

varying vec3 vPosition;
varying vec3 vColor;
varying float vIntensity;

void main() {
  vPosition = position;
  vColor = color;
  vIntensity = intensity;
  vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
  gl_Position = projectionMatrix * mvPosition;
  // Size attenuation: size * (scale / -z)
  // 300.0 is an approximation of the viewport height factor for typical FOV
  gl_PointSize = uPointSize * (300.0 / -mvPosition.z);
}
`;

export const heatmapFragmentShader = `
uniform int uMode; // 0=none, 1=height, 2=distance, 3=intensity
uniform float uMin;
uniform float uMax;
uniform int uColorScheme; // 0=viridis, 1=plasma, 2=magma, 3=inferno, 4=jet, 5=grayscale
uniform vec3 uCameraPosition;
uniform float uOpacity;

varying vec3 vPosition;
varying vec3 vColor;
varying float vIntensity;

${viridisRamp}
${plasmaRamp}
${magmaRamp}
${infernoRamp}
${jetRamp}
${grayscaleRamp}

void main() {
  vec3 color = vColor;
  
  if (uMode > 0) {
    float value = 0.0;
    
    if (uMode == 1) {
      // Height mode (Z coordinate)
      value = vPosition.z;
    } else if (uMode == 2) {
      // Distance from camera
      value = length(vPosition - uCameraPosition);
    } else if (uMode == 3) {
      // Intensity (use dedicated intensity attribute, fallback to color.r if not available)
      value = vIntensity > 0.0 ? vIntensity : vColor.r;
    }
    
    // Normalize to 0-1
    float t = clamp((value - uMin) / (uMax - uMin), 0.0, 1.0);
    
    // Apply color ramp
    if (uColorScheme == 0) {
      color = viridis(t);
    } else if (uColorScheme == 1) {
      color = plasma(t);
    } else if (uColorScheme == 2) {
      color = magma(t);
    } else if (uColorScheme == 3) {
      color = inferno(t);
    } else if (uColorScheme == 4) {
      color = jet(t);
    } else {
      color = grayscale(t);
    }
  }
  
  gl_FragColor = vec4(color, uOpacity);
}
`;
