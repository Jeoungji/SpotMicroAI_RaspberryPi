#pragma once
#ifndef _MATRIX
#define _MATRIX

#define N 4

class Matrix {
public:
    void dot(double result[][4], double A[][4], double B[][4])
    {
        int i, j, k;
        for (i = 0; i < 4; i++) {
            for (j = 0; j < 4; j++) {
                result[i][j] = 0;   // C 행렬의 (i,j) 번째 원소 초기화
                for (k = 0; k < 4; k++) {
                    result[i][j] += A[i][k] * B[k][j];   // 행렬 내적 계산식
                }
            }
        }
    }
    void dot(double result[4], double A[][4], double B[4])
    {
        int i, k;
        for (i = 0; i < 4; i++) {
            result[i] = 0;   // C 행렬의 (i,j) 번째 원소 초기화
            for (k = 0; k < 4; k++) {
                result[i] += A[i][k] * B[k];   // 행렬 내적 계산식
            }
        }
    }

    double determinant(double matrix[N][N], int n) {
        double det = 0;
        double submatrix[N][N];

        if (n == 2) {
            return ((matrix[0][0] * matrix[1][1]) - (matrix[1][0] * matrix[0][1]));
        }
        else {
            for (int x = 0; x < n; x++) {
                int sub_i = 0;
                for (int i = 1; i < n; i++) {
                    int sub_j = 0;
                    for (int j = 0; j < n; j++) {
                        if (j == x) continue;
                        submatrix[sub_i][sub_j] = matrix[i][j];
                        sub_j++;
                    }
                    sub_i++;
                }
                det = det + (matrix[0][x] * ((x % 2 == 0) ? 1 : -1) * determinant(submatrix, n - 1));
            }
        }
        return det;
    }

    double determinant(double matrix[N - 1][N - 1], int n) {
        double det = 0;
        double submatrix[N - 1][N - 1];

        if (n == 2) {
            return ((matrix[0][0] * matrix[1][1]) - (matrix[1][0] * matrix[0][1]));
        }
        else {
            for (int x = 0; x < n; x++) {
                int sub_i = 0;
                for (int i = 1; i < n; i++) {
                    int sub_j = 0;
                    for (int j = 0; j < n; j++) {
                        if (j == x) continue;
                        submatrix[sub_i][sub_j] = matrix[i][j];
                        sub_j++;
                    }
                    sub_i++;
                }
                det = det + (matrix[0][x] * ((x % 2 == 0) ? 1 : -1) * determinant(submatrix, n - 1));
            }
        }
        return det;
    }

    int inv(double inverse[N][N], double matrix[N][N]) {
        
        double det = determinant(matrix, N);
        if (det == 0) {
            return 0;
        }
        double adj_matrix[N][N];

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                int sign = ((i + j) % 2 == 0) ? 1 : -1;
                int sub_i = 0;
                int sub_j = 0;
                double submatrix[N - 1][N - 1];
                for (int x = 0; x < N; x++) {
                    if (x == i) continue;
                    sub_j = 0;
                    for (int y = 0; y < N; y++) {
                        if (y == j) continue;
                        submatrix[sub_i][sub_j] = matrix[x][y];
                        sub_j++;
                    }
                    sub_i++;
                }
                adj_matrix[i][j] = sign * determinant(submatrix, N - 1);
            }
        }

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                inverse[i][j] = adj_matrix[j][i] / det;
            }
        }
        return 1;
    }

    void add(double result[N][N], double A[4][4], double B[4][4]) {
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                result[i][j] = A[i][j] + B[i][j];
    }
};

#endif