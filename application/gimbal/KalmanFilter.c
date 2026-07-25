#include <KalmanFilter.h>

// 初始化2x2矩阵
Matrix2x2 matrix_init_feng(double a11, double a12, double a21, double a22)
{
    Matrix2x2 m;
    m.data[0][0] = a11;
    m.data[0][1] = a12;
    m.data[1][0] = a21;
    m.data[1][1] = a22;
    return m;
}

// 初始化向量
Vector2 vector_init(double x1, double x2)
{
    Vector2 v;
    v.data[0] = x1;
    v.data[1] = x2;
    return v;
}

// 矩阵乘法 C = A * B
Matrix2x2 matrix_multiply(Matrix2x2 A, Matrix2x2 B)
{
    Matrix2x2 C;
    C.data[0][0] = A.data[0][0] * B.data[0][0] + A.data[0][1] * B.data[1][0];
    C.data[0][1] = A.data[0][0] * B.data[0][1] + A.data[0][1] * B.data[1][1];
    C.data[1][0] = A.data[1][0] * B.data[0][0] + A.data[1][1] * B.data[1][0];
    C.data[1][1] = A.data[1][0] * B.data[0][1] + A.data[1][1] * B.data[1][1];
    return C;
}

// 矩阵转置
Matrix2x2 matrix_transpose(Matrix2x2 A)
{
    Matrix2x2 T;
    T.data[0][0] = A.data[0][0];
    T.data[0][1] = A.data[1][0];
    T.data[1][0] = A.data[0][1];
    T.data[1][1] = A.data[1][1];
    return T;
}

// 矩阵加法 C = A + B
Matrix2x2 matrix_add_feng(Matrix2x2 A, Matrix2x2 B)
{
    Matrix2x2 C;
    C.data[0][0] = A.data[0][0] + B.data[0][0];
    C.data[0][1] = A.data[0][1] + B.data[0][1];
    C.data[1][0] = A.data[1][0] + B.data[1][0];
    C.data[1][1] = A.data[1][1] + B.data[1][1];
    return C;
}

// 矩阵减法 C = A - B
Matrix2x2 matrix_subtract(Matrix2x2 A, Matrix2x2 B)
{
    Matrix2x2 C;
    C.data[0][0] = A.data[0][0] - B.data[0][0];
    C.data[0][1] = A.data[0][1] - B.data[0][1];
    C.data[1][0] = A.data[1][0] - B.data[1][0];
    C.data[1][1] = A.data[1][1] - B.data[1][1];
    return C;
}

// 矩阵求逆（2x2专用）
Matrix2x2 matrix_inverse(Matrix2x2 A)
{
    Matrix2x2 inv;
    double det = A.data[0][0] * A.data[1][1] - A.data[0][1] * A.data[1][0];

    // if (fabs(det) < 1e-10)
    // {
    //     printf("Warning: Matrix is singular, cannot invert.\n");
    //     return matrix_init(1, 0, 0, 1); // 返回单位矩阵
    // }

    double inv_det = 1.0 / det;
    inv.data[0][0] = A.data[1][1] * inv_det;
    inv.data[0][1] = -A.data[0][1] * inv_det;
    inv.data[1][0] = -A.data[1][0] * inv_det;
    inv.data[1][1] = A.data[0][0] * inv_det;

    return inv;
}

// 矩阵乘以向量
Vector2 matrix_vector_multiply(Matrix2x2 A, Vector2 v)
{
    Vector2 result;
    result.data[0] = A.data[0][0] * v.data[0] + A.data[0][1] * v.data[1];
    result.data[1] = A.data[1][0] * v.data[0] + A.data[1][1] * v.data[1];
    return result;
}

// 向量加法
Vector2 vector_add(Vector2 a, Vector2 b)
{
    Vector2 result;
    result.data[0] = a.data[0] + b.data[0];
    result.data[1] = a.data[1] + b.data[1];
    return result;
}

// 向量减法
Vector2 vector_subtract(Vector2 a, Vector2 b)
{
    Vector2 result;
    result.data[0] = a.data[0] - b.data[0];
    result.data[1] = a.data[1] - b.data[1];
    return result;
}

// 初始化卡尔曼滤波器
KalmanFilter kalman_init(Matrix2x2 A, Matrix2x2 H, Matrix2x2 Q, Matrix2x2 R, Vector2 initial_state)
{
    KalmanFilter kf;

    kf.A = A;
    kf.H = H;
    kf.Q = Q;
    kf.R = R;
    kf.x = initial_state;

    // 初始估计误差协方差，通常设为较大的值
    kf.P = matrix_init_feng(1.0, 0.0, 0.0, 1.0);

    return kf;
}

// 预测步骤
void kalman_predict(KalmanFilter *kf)
{
    // 状态预测: x = A * x
    kf->x = matrix_vector_multiply(kf->A, kf->x);

    // 协方差预测: P = A * P * A^T + Q
    Matrix2x2 AP = matrix_multiply(kf->A, kf->P);
    Matrix2x2 APAT = matrix_multiply(AP, matrix_transpose(kf->A));
    kf->P = matrix_add_feng(APAT, kf->Q);
}

// 更新步骤
void kalman_update(KalmanFilter *kf, Vector2 measurement)
{
    // 计算卡尔曼增益: K = P * H^T * (H * P * H^T + R)^(-1)
    Matrix2x2 H_T = matrix_transpose(kf->H);
    Matrix2x2 PH_T = matrix_multiply(kf->P, H_T);
    Matrix2x2 HPH_T = matrix_multiply(kf->H, PH_T);
    Matrix2x2 S = matrix_add_feng(HPH_T, kf->R);
    Matrix2x2 S_inv = matrix_inverse(S);
    kf->K = matrix_multiply(PH_T, S_inv);

    // 计算测量残差: y = z - H * x
    Vector2 Hx = matrix_vector_multiply(kf->H, kf->x);
    Vector2 y = vector_subtract(measurement, Hx);

    // 状态更新: x = x + K * y
    Vector2 Ky = matrix_vector_multiply(kf->K, y);
    kf->x = vector_add(kf->x, Ky);

    // 协方差更新: P = (I - K * H) * P
    Matrix2x2 KH = matrix_multiply(kf->K, kf->H);
    Matrix2x2 I = matrix_init_feng(1.0, 0.0, 0.0, 1.0);
    Matrix2x2 I_KH = matrix_subtract(I, KH);
    kf->P = matrix_multiply(I_KH, kf->P);
}

// // 示例使用
// int main() {
//     // 定义系统矩阵
//     // 状态转移矩阵 A (假设系统是匀速运动)
//     Matrix2x2 A = matrix_init_feng(1.0, delta_time,  // 位置 + 速度*0.1
//                               0.0, 1.0); // 速度保持不变

//     // 观测矩阵 H (假设我们只能观测到位置)
//     Matrix2x2 H = matrix_init_feng(1.0, 0.0,  // 观测位置
//                               0.0, 1.0); // 观测速度

//     // 过程噪声协方差 Q
//     Matrix2x2 Q = matrix_init_feng(0.01, 0.0,  // 位置噪声较小
//                               0.0, 0.1);  // 速度噪声较大

//     // 观测噪声协方差 R
//     Matrix2x2 R = matrix_init_feng(0.1, 0.0,   // 位置观测噪声
//                               0.0, 0.2);  // 速度观测噪声

//     // 初始状态 [位置, 速度]
//     Vector2 initial_state = vector_init(0.0, 0.0);

//     // 初始化卡尔曼滤波器
//     KalmanFilter kf = kalman_init(A, H, Q, R, initial_state);

//     // 模拟一些测量数据
//     Vector2 measurements[] = {
//         vector_init(0.9, 1.2),
//         vector_init(2.1, 0.8),
//         vector_init(2.8, 1.1),
//         vector_init(4.2, 0.9),
//         vector_init(4.9, 1.0)
//     };

//     int num_measurements = sizeof(measurements) / sizeof(measurements[0]);

//     for (int i = 0; i < num_measurements; i++) {

//         // 预测步骤
//         kalman_predict(&kf);

//         // 更新步骤
//         kalman_update(&kf, measurements[i]);
//     }

// }