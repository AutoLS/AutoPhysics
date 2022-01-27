#ifndef AUTO_MATH_H
#define AUTO_MATH_H

//#include "raymath.h"

#define PI 3.141593f
const float ms_in_sec = 1.0f / 1000.0f;

inline float randf(float low, float high)
{
	if(low == high) return low;
	float result = low + (float)(rand() / (float)(RAND_MAX/(high - low)));
	return result;
}

float clamp(float n, float low, float high)
{
	if(n < low) n = low;
	if(n > high) n = high;

	return n;
}

float radians(float theta)
{
	return theta * (PI / 180);
}

union Vector2
{
	struct
	{
		float x, y;
	};

	float E[2];
};

inline Vector2 operator+(Vector2 a, Vector2 b)
{
    Vector2 result = {a.x + b.x, a.y + b.y};
    return result;
}

inline Vector2 operator-(Vector2 a, Vector2 b)
{
    Vector2 result = {a.x - b.x, a.y - b.y};
    return result;
}

inline Vector2 operator-(Vector2 a)
{
	Vector2 result = {-a.x, -a.y};
	return result;
}

inline Vector2 operator*(Vector2 a, float k)
{
    Vector2 result = {a.x * k, a.y * k};
    return result;
}

inline Vector2 operator*(float k, Vector2 a)
{
    Vector2 result = {a.x * k, a.y * k};
    return result;
}

inline Vector2 V2(float x = 0, float y = 0)
{
    Vector2 result = {x, y};
    return result;
}

inline float dot(Vector2 a, Vector2 b)
{
	return a.x*b.x + a.y*b.y;
}

inline Vector2 perp(Vector2 a)
{
	return V2(-a.y, a.x);
}

inline Vector2 reverse_perp(Vector2 a)
{
	return V2(a.y, -a.x);
}

union Vector3
{
	struct
	{
		float x, y, z;
	};

	struct
	{
		Vector2 xy;
		float z;
	};

	struct
	{
		Vector2 yz;
		float x;
	};

	struct
	{
		Vector2 xz;
		float y;
	};
};

inline Vector3 V3(float x = 0, float y = 0, float z = 0)
{
    Vector3 result = {x, y, z};
    return result;
}

inline Vector3 V3(Vector2 a, float z = 0)
{
    Vector3 result = {a.x, a.y, z};
    return result;
}

inline Vector3 operator+(Vector3 a, Vector3 b)
{
    Vector3 result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

inline Vector3 operator-(Vector3 a, Vector3 b)
{
    Vector3 result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

inline Vector3 operator-(Vector3 a)
{
	Vector3 result = {-a.x, -a.y, -a.z};
	return result;
}

inline Vector3 operator*(Vector3 a, float k)
{
    Vector3 result = {a.x * k, a.y * k, a.z * k};
    return result;
}

inline Vector3 operator*(float k, Vector3 a)
{
    Vector3 result = {a.x * k, a.y * k, a.z * k};
    return result;
}

inline Vector3 operator/(Vector3 a, float k)
{
	float inverse = 1.0f / k;
	Vector3 result = {a.x * inverse, a.y * inverse, a.z * inverse};
	return result;
}

inline Vector3 operator+=(Vector3& a, Vector3 b)
{
	a = a + b;
	return a;
}

inline Vector3 operator-=(Vector3& a, Vector3 b)
{
	a = a - b;
	return a;
}

inline bool operator==(Vector3 a, Vector3 b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline float dot(Vector3 a, Vector3 b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline float length(Vector3 a)
{
	return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
}

inline Vector3 normalize(Vector3 a)
{
	if(a == V3())
	{
		return a;
	}

	Vector3 result = a / length(a);
	return result;
}

inline Vector3 cross(Vector3 a, Vector3 b)
{
	Vector3 n = {};
	n.x = a.y * b.z - a.z * b.y;
    n.y = a.z * b.x - a.x * b.z;
    n.z = a.x * b.y - a.y * b.x;

	return n;
}

inline Vector3 triple_product(Vector3 a, Vector3 b, Vector3 c)
{
	float ac = dot(a.xy, c.xy);
	float bc = dot(b.xy, c.xy);

	Vector3 result = V3(b.x * ac - a.x * bc, b.y * ac - a.y * bc, 0);

	return result;
}

struct Vector4
{
	struct
	{
		float x, y, z, w;
	};
	struct
	{
		float r, g, b, a;
	};
};

inline Vector4 V4(float x = 0, float y = 0, float z = 0, float w = 0)
{
	Vector4 result = {x, y, z, w};
	return result;
}

inline Vector4 V4(Vector3 a, float w = 0)
{
	Vector4 result = {a.x, a.y, a.z, w};
	return result;
}

//NOTE: Below are matrix and quaternion implementations
//		Matrix is in row major

union Mat3
{
	struct 
	{
		float _11, _12, _13;
		float _21, _22, _23;
		float _31, _32, _33;
	};
	struct 
	{
		float E[3][3];
	};
	struct 
	{
		float m[9];
	};
	struct
	{
		Vector3 x;
		Vector3 y;
		Vector3 z;
	};
};

Mat3 mat3_identity()
{
	Mat3 result = {};
	for(int r = 0; r < 3; ++r)
	{
		result.E[r][r] = 1;
	}
	
	return result;
}

Mat3 operator*(Mat3 a, Mat3 b)
{
	Mat3 out;

	out._11 = a._11 * b._11 + a._12 * b._21 + a._13 * b._31;
	out._12 = a._11 * b._12 + a._12 * b._22 + a._13 * b._32;
	out._13 = a._11 * b._13 + a._12 * b._23 + a._13 * b._33;

	out._21 = a._21 * b._11 + a._22 * b._21 + a._23 * b._31;
	out._22 = a._21 * b._12 + a._22 * b._22 + a._23 * b._32;
	out._23 = a._21 * b._13 + a._22 * b._23 + a._23 * b._33;

	out._31 = a._31 * b._11 + a._32 * b._21 + a._33 * b._31;
	out._32 = a._31 * b._12 + a._32 * b._22 + a._33 * b._32;
	out._33 = a._31 * b._13 + a._32 * b._23 + a._33 * b._33;

	return out;
}

Vector3 operator*(Mat3& a, Vector3 b)
{
	Vector3 out;

	out.x = a._11 * b.x
		+ a._21 * b.y
		+ a._31 * b.z;

	out.y = a._12 * b.x
		+ a._22 * b.y
		+ a._32 * b.z;

	out.z = a._13 * b.x
		+ a._23 * b.y
		+ a._33 * b.z;

	return out;
}

Vector3 operator*(Vector3 b, Mat3& a)
{
	Vector3 out;

	out.x = a._11 * b.x
		+ a._21 * b.y
		+ a._31 * b.z;

	out.y = a._12 * b.x
		+ a._22 * b.y
		+ a._32 * b.z;

	out.z = a._13 * b.x
		+ a._23 * b.y
		+ a._33 * b.z;

	return out;
}

Mat3 transpose(Mat3 A)
{
	Mat3 m;

	m._11 = A._11;
	m._21 = A._12;
	m._31 = A._13;

	m._12 = A._21;
	m._22 = A._22;
	m._32 = A._23;

	m._13 = A._31;
	m._23 = A._32;
	m._33 = A._33;

	return m;
}

union Mat4
{
	struct
	{
		float E[4][4];
	};
	
	struct 
	{
		float m[16];
	};
	
	struct 
	{
		Vector4 x;
		Vector4 y;
		Vector4 z;
		Vector4 w;
	};
};

Mat4 mat4_identity()
{
	Mat4 Result = {};
	for(int r = 0; r < 4; ++r)
	{
		Result.E[r][r] = 1;
	}
	
	return Result;
}

Mat4 operator*(Mat4 A, Mat4 B)
{
	Mat4 Result = {};
	for(int r = 0; r < 4; ++r)
	{
		for(int c = 0; c < 4; ++c)
		{
			Result.E[r][c] = A.E[r][0] * B.E[0][c] +
							 A.E[r][1] * B.E[1][c] +
							 A.E[r][2] * B.E[2][c] +
							 A.E[r][3] * B.E[3][c];
		}
	}
	return Result;
}

Vector4 operator*(Mat4 &A, Vector4 B)
{
	Vector4 Result = {};
	float* PtrResult = &Result.x;
	for(int i = 0; i < 4; ++i)
	{
		*(PtrResult + i) = A.E[i][0] * B.x +
						   A.E[i][1] * B.y +
						   A.E[i][2] * B.z +
						   A.E[i][3] * B.w;
	}
	return Result;
}

Vector4 operator*(Vector4 B, Mat4 &A)
{
	Vector4 Result = {};
	float* PtrResult = &Result.x;
	for(int i = 0; i < 4; ++i)
	{
		*(PtrResult + i) = A.E[i][0] * B.x +
						   A.E[i][1] * B.y +
						   A.E[i][2] * B.z +
						   A.E[i][3] * B.w;
	}
	return Result;
}

Mat4 mat4_scale(Mat4& A, Vector3 K)
{
	Mat4 Result = A;
	float* Ptr = &K.x;
	for(int r = 0; r < 3; ++r)
	{
		Result.E[r][r] *= *(Ptr + r);
	}
	
	return Result;
}

Mat4 mat4_translate(Mat4 A, Vector3 T)
{
	Mat4 Result = A;
	
	Result.E[0][3] += T.x;
	Result.E[1][3] += T.y;
	Result.E[2][3] += T.z;

	return Result;
}

Mat4 mat4_rotate(Mat4 A, Vector3 Axis, float Theta)
{
	if(length(Axis))
	Axis = normalize(Axis);
	
	Mat4 Result = mat4_identity();
	
	//p'
	Result.E[0][0] = cosf(Theta) + ((Axis.x * Axis.x) * (1 - cosf(Theta)));
	Result.E[0][1] = (Axis.x * Axis.y * (1 - cosf(Theta))) - (Axis.z * sinf(Theta));
	Result.E[0][2] = (Axis.x * Axis.z * (1 - cosf(Theta))) + (Axis.y * sinf(Theta));
	
	//q'
	Result.E[1][0] = (Axis.y * Axis.x * (1 - cosf(Theta))) + (Axis.z * sinf(Theta));
	Result.E[1][1] = cosf(Theta) + ((Axis.y * Axis.y) * (1 - cosf(Theta)));
	Result.E[1][2] = (Axis.z * Axis.y * (1 - cosf(Theta))) - (Axis.x * sinf(Theta));
	
	//r'
	Result.E[2][0] = (Axis.z * Axis.x * (1 - cosf(Theta))) - (Axis.y * sinf(Theta));
	Result.E[2][1] = (Axis.z * Axis.y * (1 - cosf(Theta))) + (Axis.x * sinf(Theta));
	Result.E[2][2] = cosf(Theta) + ((Axis.z * Axis.z) * (1 - cosf(Theta)));
	
	Result = Result * A;
	
	return Result;
}

Mat4 mat4_ortho(float Left, float Right, 
		        float Bottom, float Top, 
		        float Near, float Far)
{
	Mat4 Result = mat4_identity();
	Result.E[0][0] = 2 / (Right - Left);
	Result.E[1][1] = 2 / (Top - Bottom);
	Result.E[2][2] = -2 / (Far - Near);
	
	Result.E[0][3] = -(Right + Left) / (Right - Left);
	Result.E[1][3] = -(Top + Bottom) / (Top - Bottom);
	Result.E[2][3] = -(Far + Near) / (Far - Near);
	
	return Result;
}

struct Quaternion 
{
	union 
	{
		struct
		{
			float x, y, z;
		};
		Vector3 xyz;
	};
	float w = 1.0f;
};

Quaternion make_quaternion(Vector3 v, float w)
{
	Quaternion q = {v.x, v.y, v.z, w};
	return q;
}

Quaternion make_quaternion(float x, float y, float z, float w)
{
	Quaternion q = {x, y, z, w};
	return q;
}

Quaternion operator+(Quaternion q1, Quaternion q2)
{
	Quaternion q = {q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w};
	return q;
} 

float dot(Quaternion a, Quaternion b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}

Quaternion normalize(Quaternion q)
{
	Quaternion result = q;
	float magnitude = sqrt(dot(q, q));
	if(magnitude > 0)
	{
		float t = 1.0f / magnitude;
		result.x *= t;
		result.y *= t;
		result.z *= t;
		result.w *= t;
	}
	else
	{
		result.x = 0;
		result.y = 0;
		result.z = 0;
		result.w = 1;
	}

	return result;
}

Mat3 to_mat3(Quaternion q)
{
	Mat3 mat = mat3_identity();

	float yy = q.y*q.y;
	float zz = q.z*q.z;
	float xy = q.x*q.y;
	float zw = q.z*q.w;
	float xz = q.x*q.z;
	float yw = q.y*q.w;
	float xx = q.x*q.x;
	float yz = q.y*q.z;
	float xw = q.x*q.w;

	mat._11 = 1.0f - 2.0f * (yy + zz);
	mat._12 = 2.0f * (xy + zw);
	mat._13 = 2.0f * (xz - yw);

	mat._21 = 2.0f * (xy - zw);
	mat._22 = 1.0f - 2.0f * (xx + zz);
	mat._23 = 2.0f * (yz + xw);

	mat._31 = 2.0f * (xz + yw);
	mat._32 = 2.0f * (yz - xw);
	mat._33 = 1.0f - 2.0f * (xx + yy);

	return mat;
}

Mat4 to_mat4(Quaternion q)
{
	Mat4 mat = mat4_identity();

	float yy = q.y*q.y;
	float zz = q.z*q.z;
	float xy = q.x*q.y;
	float zw = q.z*q.w;
	float xz = q.x*q.z;
	float yw = q.y*q.w;
	float xx = q.x*q.x;
	float yz = q.y*q.z;
	float xw = q.x*q.w;

	mat.m[0] = 1.0f - 2.0f * (yy + zz);
	mat.m[1] = 2.0f * (xy + zw);
	mat.m[2] = 2.0f * (xz - yw);

	mat.m[4] = 2.0f * (xy - zw);
	mat.m[5] = 1.0f - 2.0f * (xx + zz);
	mat.m[6] = 2.0f * (yz + xw);

	mat.m[8] = 2.0f * (xz + yw);
	mat.m[9] = 2.0f * (yz - xw);
	mat.m[10] = 1.0f - 2.0f * (xx + yy);

	return mat;
}

#endif