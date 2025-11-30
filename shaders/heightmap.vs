#version 330

// Input vertex attributes
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in vec4 vertexColor;

// Input uniform values
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
uniform sampler2D heightmapTexture;
uniform float heightScale;

// Output vertex attributes (to fragment shader)
out vec3 fragPosition;
out vec2 fragTexCoord;
out vec4 fragColor;
out vec3 fragNormal;
out float fragHeight;

void main()
{
    // Sample the heightmap texture
    float height = texture(heightmapTexture, vertexTexCoord).r;
    fragHeight = height;
    
    // Displace vertex along its normal based on heightmap
    vec3 displacedPosition = vertexPosition + vertexNormal * height * heightScale;
    
    // Calculate normals by sampling neighboring heights for proper lighting
    float texelSize = 0.01; // Adjust based on mesh resolution
    float heightL = texture(heightmapTexture, vertexTexCoord + vec2(-texelSize, 0.0)).r;
    float heightR = texture(heightmapTexture, vertexTexCoord + vec2(texelSize, 0.0)).r;
    float heightD = texture(heightmapTexture, vertexTexCoord + vec2(0.0, -texelSize)).r;
    float heightU = texture(heightmapTexture, vertexTexCoord + vec2(0.0, texelSize)).r;
    
    // Calculate tangent vectors
    vec3 tangentX = vec3(2.0 * texelSize * heightScale, heightR - heightL, 0.0);
    vec3 tangentZ = vec3(0.0, heightU - heightD, 2.0 * texelSize * heightScale);
    
    // Calculate normal using cross product
    vec3 calculatedNormal = normalize(cross(tangentZ, tangentX));
    
    // Send vertex attributes to fragment shader
    fragPosition = vec3(matModel * vec4(displacedPosition, 1.0));
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    fragNormal = normalize(vec3(matNormal * vec4(calculatedNormal, 0.0)));
    
    // Calculate final vertex position
    gl_Position = mvp * vec4(displacedPosition, 1.0);
}
