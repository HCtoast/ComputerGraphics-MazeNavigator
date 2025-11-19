#include <vec.h>
#include <mat.h>

mat4 myLookAt(vec3 eye, vec3 at, vec3 up)
{
	mat4 V(1.0f);
	vec3 n = at-eye;
	n /= length(n);

	float a = dot(up, n);
	vec3 v = up - a*n;
	v /= length(v);

	vec3 w = cross(n, v);

	mat4 Rw(1.0f);

	Rw[0][0] = w.x;	Rw[0][1] = v.x; Rw[0][2] = -n.x;
	Rw[1][0] = w.y;	Rw[1][1] = v.y; Rw[1][2] = -n.y;
	Rw[2][0] = w.z;	Rw[2][1] = v.z; Rw[2][2] = -n.z;

	mat4 Rc;
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			Rc[i][j] = Rw[j][i];

	mat4 Tc = Translate(-eye.x, -eye.y, -eye.z);

	V = Rc*Tc;
		
	return V;
}

mat4 myOrtho(float l, float r, float b, float t, float zNear, float zFar)
{
	mat4 V(1.0f);
	mat4 T = Translate(-(l+r)/2,-(b+t)/2, zNear);
	mat4 S = Scale(2/(r-l), 2/(t-b), 1/(zFar-zNear));

	V = S*T;
	return V;
}

mat4 myPerspective(float fovy, float aspectRatio, float zNear, float zFar)
{
	mat4 P(1.0f);
	
	float sz = 1/zFar;
	float angle = fovy/2.0*3.141592/180.0f;
	float halfHeight = zFar*tan(angle);
	float halfWidth = halfHeight*aspectRatio;
	float sy = 1/halfHeight;
	float sx = 1/halfWidth;
	mat4 S = Scale(sx, sy, sz);

	mat4 H(1.0f);
	float c = -zNear/zFar;
	H[2][2] = 1/(c+1);		H[2][3] = -c/(c+1);
	H[3][2] = -1;			H[3][3] = 0;
	
	P = H*S;
		
	return P;
}