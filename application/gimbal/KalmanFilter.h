#ifndef _KalmanFilter_h_
#define _KalmanFilter_h_

// 2x2矩阵结构体
typedef struct
{
    double data[2][2];
} Matrix2x2;

// 2维向量结构体
typedef struct
{
    double data[2];
} Vector2;

// 卡尔曼滤波器结构体
typedef struct
{
    Matrix2x2 A; // 状态转移矩阵
    Matrix2x2 H; // 观测矩阵
    Matrix2x2 Q; // 过程噪声协方差
    Matrix2x2 R; // 观测噪声协方差
    Matrix2x2 P; // 估计误差协方差
    Matrix2x2 K; // 卡尔曼增益
    Vector2 x;   // 状态向量
} KalmanFilter;

Matrix2x2 matrix_init_feng(double a11, double a12, double a21, double a22);
Vector2 vector_init(double x1, double x2);


KalmanFilter kalman_init(Matrix2x2 A, Matrix2x2 H, Matrix2x2 Q, Matrix2x2 R, Vector2 initial_state);

void kalman_predict(KalmanFilter *kf);

void kalman_update(KalmanFilter *kf, Vector2 measurement);
#endif // !_KalmanFilter_h_