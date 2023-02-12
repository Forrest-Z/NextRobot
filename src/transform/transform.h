//
// Created by waxz on 22-10-11.
//

#ifndef SCAN_REPUBLISHER_TRANSFORM_H
#define SCAN_REPUBLISHER_TRANSFORM_H

#include <cmath>
#include <vector>
#include <iostream>

namespace transform{

    struct Transform2d{

        std::array<std::array<float,3>,3> matrix;
        Transform2d(float x=0.0, float y=0.0,float yaw=0.0){
            set(x,y,yaw);
        }
        void set(float x=0.0, float y=0.0,float yaw=0.0){
            // Calculate rotation about z axis
            /*
                     cos(yaw),   -sin(yaw),      0,
                     sin(yaw),   cos(yaw),       0,
                     0,          0,              1
                 */
            matrix[0][0] = cos(yaw);
            matrix[0][1]  = -sin(yaw);
            matrix[1][0] = sin(yaw);
            matrix[1][1]  = cos(yaw);

            matrix[0][2]  = x;
            matrix[1][2]  = y;

            matrix[2][0]  = 0.0;
            matrix[2][1]  = 0.0;
            matrix[2][2]  = 1.0;

        }

        float x() const {
            return matrix[0][2];
        }
        float y() const {
            return matrix[1][2];
        }
        float yaw() const {
            return atan2(matrix[1][0],matrix[0][0]);
        }


        Transform2d mul(const Transform2d& rhv)const {
            Transform2d result;

            auto& a = this->matrix;
            auto& b = rhv.matrix;
            auto& c = result.matrix;
            // Calculate the j-th column of the result in-place (in B) using the helper array

//            std::cout << "check matrix mul:\n";
            for(int i=0 ; i<3 ; i++)
            {
                for(int j=0 ; j<3 ; j++)
                {

                    c[i][j]=0;
                    for(int k=0 ; k<3 ; k++)
                    {
//                        std::cout <<" [ " <<  a[i][k] << " * " << b[k][j] << " ] " ;
                        c[i][j]+=a[i][k]*b[k][j];
                        //--^-- should be k
                    }
//                    std::cout << " = " << c[i][j] << "\n";

                }
            }
            return result;

        }

        Transform2d operator*(const Transform2d& rhv)const{
            Transform2d result;

            auto& a = this->matrix;
            auto& b = rhv.matrix;
            auto& c = result.matrix;
            // Calculate the j-th column of the result in-place (in B) using the helper array

//            std::cout << "check matrix multiply:\n";
            for(int i=0 ; i<3 ; i++)
            {
                for(int j=0 ; j<3 ; j++)
                {

                    c[i][j]=0;
                    for(int k=0 ; k<3 ; k++)
                    {
//                        std::cout <<" [ " <<  a[i][k] << " * " << b[k][j] << " ] " ;
                        c[i][j]+=a[i][k]*b[k][j];
                        //--^-- should be k
                    }
//                    std::cout << " = " << c[i][j] << "\n";

                }
            }
            return result;

        }
        void mul(const std::vector<float>& points, std::vector<float>& result){

            /*
         r00 r01 r02 tx     x0        x1
         r10 r11 r12 ty  X  y0   =>   y1
         r20 r21 r22 tz     z0        z1
         0   0   0   1      1         1
        */

            if(points.size()%2 != 0 ){
                std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

            }
            result.resize(points.size());

            float r00 = this->matrix[0][0];
            float r01 = this->matrix[0][1];

            float r10 = this->matrix[1][0];
            float r11 = this->matrix[1][1];

            float tx = this->matrix[0][2];
            float ty = this->matrix[1][2];

            int n_dim = points.size()/2;

            const float *p_data_x = &(points[0]);
//            const float *p_data_y = p_data_x + n_dim;

            float *p_x = &(result[0]);
//            float *p_y = p_x + n_dim;
//            std::cout << "tf, tx = " << tx << ", ty = " << ty << std::endl;

//            std::cout << "tf, r00 = " << r00 << ", r01 = " << r01 << ", r10 = " << r10  << ", r11 = " << r11 << std::endl;


            for (int i = 0; i < n_dim; i++) {
                p_x[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
                p_x[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
//                std::cout << "== i = " << i << std::endl;
//                std::cout << "== tf p_x = " << p_data_x[i+i] << ", p_y = " << p_data_x[i + i + 1] << std::endl;
//                std::cout << "== tf d_x = " << r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx << ", d_y = " << r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty << std::endl;
            }

        }
        void mul(const std::vector<float>& points, size_t n_dim, std::vector<float>& result){

            /*
         r00 r01 r02 tx     x0        x1
         r10 r11 r12 ty  X  y0   =>   y1
         r20 r21 r22 tz     z0        z1
         0   0   0   1      1         1
        */

            if(points.size() < (n_dim+n_dim) ){
                std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

            }
            result.resize(n_dim+n_dim);

            float r00 = this->matrix[0][0];
            float r01 = this->matrix[0][1];

            float r10 = this->matrix[1][0];
            float r11 = this->matrix[1][1];

            float tx = this->matrix[0][2];
            float ty = this->matrix[1][2];


            const float *p_data_x = &(points[0]);
//            const float *p_data_y = p_data_x + n_dim;

            float *p_x = &(result[0]);
//            float *p_y = p_x + n_dim;
//            std::cout << "tf, tx = " << tx << ", ty = " << ty << std::endl;

//            std::cout << "tf, r00 = " << r00 << ", r01 = " << r01 << ", r10 = " << r10  << ", r11 = " << r11 << std::endl;


            for (size_t i = 0; i < n_dim; i++) {
                p_x[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
                p_x[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
//                std::cout << "== i = " << i << std::endl;
//                std::cout << "== tf p_x = " << p_data_x[i+i] << ", p_y = " << p_data_x[i + i + 1] << std::endl;
//                std::cout << "== tf d_x = " << r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx << ", d_y = " << r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty << std::endl;
            }

        }
        void mul( float* points, int n_dim, float*  result){

            /*
         r00 r01 r02 tx     x0        x1
         r10 r11 r12 ty  X  y0   =>   y1
         r20 r21 r22 tz     z0        z1
         0   0   0   1      1         1
        */


            float r00 = this->matrix[0][0];
            float r01 = this->matrix[0][1];

            float r10 = this->matrix[1][0];
            float r11 = this->matrix[1][1];

            float tx = this->matrix[0][2];
            float ty = this->matrix[1][2];


            const float *p_data_x = points;
//            const float *p_data_y = p_data_x + n_dim;

            float *p_x = result;
//            float *p_y = p_x + n_dim;
//            std::cout << "tf, tx = " << tx << ", ty = " << ty << std::endl;

//            std::cout << "tf, r00 = " << r00 << ", r01 = " << r01 << ", r10 = " << r10  << ", r11 = " << r11 << std::endl;


            for (int i = 0; i < n_dim; i++) {
                p_x[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
                p_x[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
//                std::cout << "== i = " << i << std::endl;
//                std::cout << "== tf p_x = " << p_data_x[i+i] << ", p_y = " << p_data_x[i + i + 1] << std::endl;
//                std::cout << "== tf d_x = " << r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx << ", d_y = " << r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty << std::endl;
            }

        }

        Transform2d inverse() const{
            Transform2d transform_inv;
            float determinant = 0;

            auto & mat  =this->matrix;
            //finding determinant
            for(int i = 0; i < 3; i++)
                determinant += (mat[0][i] * (mat[1][(i+1)%3] * mat[2][(i+2)%3] - mat[1][(i+2)%3] * mat[2][(i+1)%3]));

            auto & mat_inv  =transform_inv.matrix;
            float determinant_inv = 1.0f/determinant;

            for(int i = 0; i < 3; i++){
                for(int j = 0; j < 3; j++)
                    mat_inv[i][j]= ((mat[(j+1)%3][(i+1)%3] * mat[(j+2)%3][(i+2)%3]) - (mat[(j+1)%3][(i+2)%3] * mat[(j+2)%3][(i+1)%3]))* determinant_inv ;

            }

            return transform_inv;

        }

    };
    std::ostream& operator <<(std::ostream& out,const Transform2d& rhv ){
        out << "Transform2d:\n";
//    out.unsetf ( std::ios::floatfield );                // floatfield not set
        out.precision(5);
        out.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed

        out << "x-y-yaw:\n[ " << rhv.x() << ", " << rhv.y() << ", " << rhv.yaw() << " ]\n";

        out << "matrix:\n[" << rhv.matrix[0][0] << ", " << rhv.matrix[0][1] << ", " << rhv.matrix[0][2]<<"\n"
            << " " << rhv.matrix[1][0] << ", " << rhv.matrix[1][1] << ", " << rhv.matrix[1][2]<<"\n"
            <<" " << rhv.matrix[2][0] << ", " << rhv.matrix[2][1] << ", " << rhv.matrix[2][2]<<"]\n"
            << std::endl;
        out.unsetf ( std::ios::floatfield );                // floatfield not set

        return out;
    }


    template<typename FloatType>
    struct MatrixSE2{
        std::array<std::array<FloatType,3>,3> matrix;
        MatrixSE2(FloatType x = 0.0, FloatType y = 0.0, FloatType yaw = 0.0){
            set(x,y,yaw);
        }

        MatrixSE2(const Transform2d& rhv){
            set(rhv.x(),rhv.y(),rhv.yaw());
        }

        FloatType x() const {
            return matrix[0][2];
        }
        FloatType y() const {
            return matrix[1][2];
        }
        FloatType yaw() const {
            return atan2(matrix[1][0],matrix[0][0]);
        }
        void set(FloatType x, FloatType y, FloatType yaw){
            matrix[0][0] = cos(yaw);
            matrix[0][1]  = -sin(yaw);
            matrix[1][0] = sin(yaw);
            matrix[1][1]  = cos(yaw);

            matrix[0][2]  = x;
            matrix[1][2]  = y;

            matrix[2][0]  = 0.0;
            matrix[2][1]  = 0.0;
            matrix[2][2]  = 1.0;
        }

        MatrixSE2<FloatType> operator*(const MatrixSE2<FloatType>& rhv)const{
            MatrixSE2<FloatType> result;

            auto& a = this->matrix;
            auto& b = rhv.matrix;
            auto& c = result.matrix;
            // Calculate the j-th column of the result in-place (in B) using the helper array

//            std::cout << "check matrix mul:\n";
            for(int i=0 ; i<3 ; i++)
            {
                for(int j=0 ; j<3 ; j++)
                {

                    c[i][j]=0;
                    for(int k=0 ; k<3 ; k++)
                    {
//                        std::cout <<" [ " <<  a[i][k] << " * " << b[k][j] << " ] " ;
                        c[i][j]+=a[i][k]*b[k][j];
                        //--^-- should be k
                    }
//                    std::cout << " = " << c[i][j] << "\n";

                }
            }
            return result;
        }
    };

}
#endif //SCAN_REPUBLISHER_TRANSFORM_H
