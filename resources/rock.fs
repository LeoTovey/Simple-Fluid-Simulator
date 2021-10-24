#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D texture_diffuse1;

void main()
{
    //FragColor = texture(texture_diffuse1, TexCoords);
    FragColor = vec4(0.62, 0.85, 0.96, 1.0);
}