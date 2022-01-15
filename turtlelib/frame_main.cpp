#include <iostream>
#include <string>
#include <cmath>

struct Transform{
    double theta;
    double x;
    double y;
}
Tab, Tbc;

struct Transform matrix_compose(double A[3][3], double B[3][3]);
double * makeTmat(double theta, double x, double y);

int main(){
    std::string extra;
    std::cout << "Enter transform T_{a,b}: \n";
    std::cin >> extra >> Tab.theta >> extra >> Tab.x >> extra >> Tab.y;
    std::cout << "Enter transform T_{b,c}: \n";
    std::cin >> extra >> Tbc.theta >> extra >> Tbc.x >> extra >> Tbc.y;

    // Create matrices
    double *Tab_mat, *Tbc_mat;
    Tab_mat = makeTmat(Tab.theta, Tab.x, Tab.y);
    Tbc_mat = makeTmat(Tbc.theta, Tbc.x, Tbc.y);

    std::cout << "Tab_mat: " << Tab_mat << "Tab_mat: " << Tbc_mat << std::endl;
    
    // Compute Tab, Tba, Tbc, Tcb, Tac, and Tca
    // Transform Tac = matrix_compose(*Tab_mat, *Tbc_mat);




    // Display Tab, Tba, Tbc, Tcb, Tac, and Tca

}

double * makeTmat(double theta, double x, double y) {
    double T[3][3];
    T[0][0] = cos(theta); T[0][1] =-sin(theta); T[0][2] = x;
    T[1][0] = sin(theta); T[1][1] = cos(theta); T[1][2] = y;
    T[2][0] = 0;          T[2][1] = 0;          T[2][2] = 1;

    std::cout << T[0][0] << T[0][1] << T[0][2] << std::endl << T[1][0] << T[1][1] << T[1][2] << std::endl << T[2][0] << T[2][1] << T[2][2] << std::endl;

    return *T;
}

struct Transform matrix_compose(double A[][3], double B[][3]){

    Transform T;
    // int C[3][3] = {
    //     {A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0], A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1], A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2]},
    //     {A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0], A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1], A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2]}, 
    //     {A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0], A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1], A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2]}
    // };

    T.theta = acos(A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0]);
    T.x = A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2];
    T.y = A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2];

    return T;
}
