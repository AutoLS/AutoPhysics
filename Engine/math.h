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

Mat3 operator*(Mat3 A, Mat3 B)
{
	Mat4 Result = {};
	for(int r = 0; r < 3; ++r)
	{
		for(int c = 0; c < 3; ++c)
		{
			Result.E[r][c] = A.E[r][0] * B.E[0][c] +
							 A.E[r][1] * B.E[1][c] +
							 A.E[r][2] * B.E[2][c];
		}
	}
	return Result;
}

Vector3 operator*(Mat3 &A, Vector3 B)
{
	Vector3 Result = {};
	float* PtrResult = &Result.x;
	for(int i = 0; i < 3; ++i)
	{
		*(PtrResult + i) = A.E[i][0] * B.x +
						   A.E[i][1] * B.y +
						   A.E[i][2] * B.z;
	}
	return Result;
}

Vector3 operator*(Vector3 B, Mat3 &A)
{
	Vector3 Result = {};
	float* PtrResult = &Result.x;
	for(int i = 0; i < 3; ++i)
	{
		*(PtrResult + i) = A.E[i][0] * B.x +
						   A.E[i][1] * B.y +
						   A.E[i][2] * B.z;
	}
	return Result;
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
	float w;
};

#endif