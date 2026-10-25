#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

// 基本数学常数
#define M_E         2.7182818284590452354f    // 自然对数底数 e
#define M_PI        3.14159265358979323846f    // 圆周率 π
#define M_PI_2      1.57079632679489661923f    // π/2 (90度)
#define M_PI_4      0.78539816339744830962f    // π/4 (45度)

// 对数相关常数
#define M_LOG2E     1.4426950408889634074f     // log₂(e)
#define M_LOG10E    0.43429448190325182765f    // log₁₀(e)
#define M_LN2       0.69314718055994530942f    // ln(2)
#define M_LN10      2.30258509299404568402f    // ln(10)

// 倒数和根号常数
#define M_1_PI      0.31830988618379067154f    // 1/π
#define M_2_PI      0.63661977236758134308f    // 2/π
#define M_2_SQRTPI  1.12837916709551257390f    // 2/√π
#define M_SQRT2     1.41421356237309504880f    // √2
#define M_SQRT1_2   0.70710678118654752440f    // 1/√2

#define GRAVITY     9.8f                       // 重力加速度


typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif
