#version 400
uniform vec3 light_direction;
uniform mat4 projection;
uniform mat4 modelview;
uniform mat3 modelview_inverse_transpose;
layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_normal;
out vec3 v_normal;
out vec3 v_eye_to_position;
out vec3 v_light_direction;

void main( void )
{
	v_light_direction = vec3( modelview * vec4( light_direction , 0. ) );
	v_eye_to_position = vec3( modelview * vec4( vertex_position , 1. ) );
	v_normal = modelview_inverse_transpose * vertex_normal;
	gl_Position = projection * modelview * vec4( vertex_position , 1. );
}