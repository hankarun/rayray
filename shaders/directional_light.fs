#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Directional light uniforms
uniform vec3 lightDirection;  // Direction of the light (normalized)
uniform vec3 lightColor;      // Color of the light
uniform vec3 ambientColor;    // Ambient light color
uniform vec3 viewPos;         // Camera position

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
    
    // Normalize the light direction (should already be normalized, but just to be safe)
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
    
    // Sample texture color
    vec4 texelColor = texture(texture0, fragTexCoord);
    
    // Calculate final color
    finalColor = vec4(lighting, 1.0) * texelColor * colDiffuse * fragColor;
}
