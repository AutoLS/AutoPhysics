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

inline Vector2 cross(Vector2 a, float k)
{
	return V2(-1 * a.y * k, a.x * k);
}

inline float cross(Vector2 a, Vector2 b)
{
	return a.x * b.y - a.y * b.x;
}

inline Vector2 rotate(Vector2 a, float theta)
{
	Vector2 result = V2(cos(theta) * a.x - sin(theta)*a.y, sin(theta)*a.x + cos(theta)*a.y);
	return  result;
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

inline Vector3 operator*(Vector3 a, Vector3 b)
{
    Vector3 result = {a.x * b.x, a.y * b.y, a.z * b.z};
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

Vector3 angle_vec(float theta)
{
	Vector3 a = {cosf(theta), sinf(theta), 0};
	return a;
}

float to_angle(Vector3 a)
{
	return atan2(a.y, a.x);
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
	Mat3 result;

	result._11 = a._11 * b._11 + a._12 * b._21 + a._13 * b._31;
	result._12 = a._11 * b._12 + a._12 * b._22 + a._13 * b._32;
	result._13 = a._11 * b._13 + a._12 * b._23 + a._13 * b._33;

	result._21 = a._21 * b._11 + a._22 * b._21 + a._23 * b._31;
	result._22 = a._21 * b._12 + a._22 * b._22 + a._23 * b._32;
	result._23 = a._21 * b._13 + a._22 * b._23 + a._23 * b._33;

	result._31 = a._31 * b._11 + a._32 * b._21 + a._33 * b._31;
	result._32 = a._31 * b._12 + a._32 * b._22 + a._33 * b._32;
	result._33 = a._31 * b._13 + a._32 * b._23 + a._33 * b._33;

	return result;
}

Vector3 operator*(Mat3& a, Vector3 b)
{
	Vector3 result;

	result.x = a._11 * b.x
		+ a._21 * b.y
		+ a._31 * b.z;

	result.y = a._12 * b.x
		+ a._22 * b.y
		+ a._32 * b.z;

	result.z = a._13 * b.x
		+ a._23 * b.y
		+ a._33 * b.z;

	return result;
}

Vector3 operator*(Vector3 b, Mat3& a)
{
	Vector3 result;

	result.x = a._11 * b.x
		+ a._21 * b.y
		+ a._31 * b.z;

	result.y = a._12 * b.x
		+ a._22 * b.y
		+ a._32 * b.z;

	result.z = a._13 * b.x
		+ a._23 * b.y
		+ a._33 * b.z;

	return result;
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

float determinant(Mat3 m)
{
	float d = m._11*(m._22*m._33 - m._32*m._23) - m._12*(m._21*m._33 - m._23*m._31) + m._13*(m._21*m._32 - m._22*m._31);

	return d;
}

Mat3 inverse(Mat3 m)
{
	Mat3 ret;
	float det = determinant(m);
	if (det != 0)
	{
		float invdet = 1.0f / det;
		ret._11 = (m._22*m._33 - m._32*m._23) * invdet;
		ret._12 = -(m._12*m._33 - m._13*m._32) * invdet;
		ret._13 = (m._12*m._23 - m._13*m._22) * invdet;

		ret._21 = -(m._21*m._33 - m._23*m._31) * invdet;
		ret._22 = (m._11*m._33 - m._13*m._31) * invdet;
		ret._23 = -(m._11*m._23 - m._21*m._13) * invdet;

		ret._31 = (m._21*m._32 - m._31*m._22) * invdet;
		ret._32 = -(m._11*m._32 - m._31*m._12) * invdet;
		ret._33 = (m._11*m._22 - m._21*m._12) * invdet;
	}

	return ret;
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
	Mat4 result = {};
	for(int r = 0; r < 4; ++r)
	{
		result.E[r][r] = 1;
	}
	
	return result;
}

Mat4 operator*(Mat4 A, Mat4 B)
{
	Mat4 result = {};
	for(int r = 0; r < 4; ++r)
	{
		for(int c = 0; c < 4; ++c)
		{
			result.E[r][c] = A.E[r][0] * B.E[0][c] +
							 A.E[r][1] * B.E[1][c] +
							 A.E[r][2] * B.E[2][c] +
							 A.E[r][3] * B.E[3][c];
		}
	}
	return result;
}

Vector4 operator*(Mat4 &A, Vector4 B)
{
	Vector4 result = {};
	float* ptr_result = &result.x;
	for(int i = 0; i < 4; ++i)
	{
		*(ptr_result + i) = A.E[i][0] * B.x +
						   A.E[i][1] * B.y +
						   A.E[i][2] * B.z +
						   A.E[i][3] * B.w;
	}
	return result;
}

Vector4 operator*(Vector4 B, Mat4 &A)
{
	Vector4 result = {};
	float* ptr_result = &result.x;
	for(int i = 0; i < 4; ++i)
	{
		*(ptr_result + i) = A.E[i][0] * B.x +
						   A.E[i][1] * B.y +
						   A.E[i][2] * B.z +
						   A.E[i][3] * B.w;
	}
	return result;
}

Mat4 mat4_scale(Mat4& A, Vector3 K)
{
	Mat4 result = A;
	float* ptr = &K.x;
	for(int r = 0; r < 3; ++r)
	{
		result.E[r][r] *= *(ptr + r);
	}
	
	return result;
}

Mat4 mat4_translate(Mat4 A, Vector3 T)
{
	Mat4 result = A;
	
	result.E[0][3] += T.x;
	result.E[1][3] += T.y;
	result.E[2][3] += T.z;

	return result;
}

Mat4 mat4_rotate(Mat4 A, Vector3 axis, float theta)
{
	if(length(axis))
	axis = normalize(axis);
	
	Mat4 result = mat4_identity();
	
	//p'
	result.E[0][0] = cosf(theta) + ((axis.x * axis.x) * (1 - cosf(theta)));
	result.E[0][1] = (axis.x * axis.y * (1 - cosf(theta))) - (axis.z * sinf(theta));
	result.E[0][2] = (axis.x * axis.z * (1 - cosf(theta))) + (axis.y * sinf(theta));
	
	//q'
	result.E[1][0] = (axis.y * axis.x * (1 - cosf(theta))) + (axis.z * sinf(theta));
	result.E[1][1] = cosf(theta) + ((axis.y * axis.y) * (1 - cosf(theta)));
	result.E[1][2] = (axis.z * axis.y * (1 - cosf(theta))) - (axis.x * sinf(theta));
	
	//r'
	result.E[2][0] = (axis.z * axis.x * (1 - cosf(theta))) - (axis.y * sinf(theta));
	result.E[2][1] = (axis.z * axis.y * (1 - cosf(theta))) + (axis.x * sinf(theta));
	result.E[2][2] = cosf(theta) + ((axis.z * axis.z) * (1 - cosf(theta)));
	
	result = result * A;
	
	return result;
}

Mat4 mat4_ortho(float left, float right, 
		        float bottom, float top, 
		        float near_, float far_)
{
	Mat4 result = mat4_identity();
	result.E[0][0] = 2 / (right - left);
	result.E[1][1] = 2 / (top - bottom);
	result.E[2][2] = -2 / (far_ - near_);
	
	result.E[0][3] = -(right + left) / (right - left);
	result.E[1][3] = -(top + bottom) / (top - bottom);
	result.E[2][3] = -(far_ + near_) / (far_ - near_);
	
	return result;
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

Quaternion operator*(Quaternion a, Quaternion b)
{
	Quaternion q;

	q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
	q.y = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z;
	q.z = a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x;

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