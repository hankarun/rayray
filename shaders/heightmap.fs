#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;
in float fragHeight;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Directional light uniforms
uniform vec3 lightDirection;
uniform vec3 lightColor;
uniform vec3 ambientColor;
uniform vec3 viewPos;

// Material properties
uniform float ambientStrength;
uniform float specularStrength;
uniform float shininess;

// Output fragment color
out vec4 finalColor;

void main()
{
    // Normalize the normal vector
    vec3 normal = normalize(fragNormal);
    
    // Normalize the light direction
    vec3 lightDir = normalize(-lightDirection);
    
    // === Ambient lighting ===
    vec3 ambient = ambientStrength * ambientColor;
    
    // === Diffuse lighting ===
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    
    // === Specular lighting (Blinn-Phong) ===
    vec3 viewDir = normalize(viewPos - fragPosition);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), shininess);
    vec3 specular = specularStrength * spec * lightColor;
    
    // === Combine lighting components ===
    vec3 lighting = ambient + diffuse + specular;
    
    // Height-based coloring (gradient from low to high)
    vec3 lowColor = vec3(0.2, 0.4, 0.1);   // Dark green for valleys
    vec3 midColor = vec3(0.4, 0.6, 0.2);   // Mid green
    vec3 highColor = vec3(0.9, 0.9, 0.9);  // White for peaks
    
    vec3 terrainColor;
    if (fragHeight < 0.5) {
        terrainColor = mix(lowColor, midColor, fragHeight * 2.0);
    } else {
        terrainColor = mix(midColor, highColor, (fragHeight - 0.5) * 2.0);
    }
    
    // Calculate final color with terrain coloring
    finalColor = vec4(lighting * terrainColor, 1.0) * colDiffuse * fragColor;
}
