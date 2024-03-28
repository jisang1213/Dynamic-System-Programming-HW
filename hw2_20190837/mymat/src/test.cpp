#include "../include/mymat.hpp"

int main()
{
    // Test any functions you want. This code will not be evaluated.

    double elems[9] = {3.0, 1.0, 4.0, 1.0, 5.0, 9.0, 2.0, 6.0, 5.0};
    
    Mat33 mat_a, mat_inv, mat_eye;
    for (int row = 0; row < 3; row++) for (int col = 0; col < 3; col++) mat_a.set_elem(row, col, elems[row * 3 + col]);

    mat_inv = mat_a.inverse();
    mat_eye = mat_a * mat_inv;

    std::cout << "This is A" << std::endl;
    mat_a.display();
    std::cout << "This is inv(A)" << std::endl;
    mat_inv.display();
    std::cout << "This is A * inv(A)" << std::endl;
    mat_eye.display();


    Vec3 vec1, vec2, vec3;
    vec1.set_elem(0, 0, 1);
    vec1.set_elem(1, 0, 2);
    vec1.set_elem(2, 0, 4);

    vec2.set_elem(0, 0, 4);
    vec2.set_elem(1, 0, 1);
    vec2.set_elem(2, 0, 0);

    vec3 = vec1.cross(vec2);

    std::cout << "This is vec1 and vec2" << std::endl;
    vec1.display();
    vec2.display();
    std::cout << "This is vec3" << std::endl;
    vec3.display();
    std::cout << "norm of vec3:" << vec3.norm() <<std::endl;
    vec3 = vec3.normalized();
    std::cout << "This is normalized vec3" << std::endl;
    vec3.display();


    // vec1.set_elem(0, 0, 4);
    // vec1.set_elem(1, 0, 2);
    // vec1.set_elem(2, 0, 47);
    // Mat33 I(2,3,1);
    // Vec3 result = (I * vec1);
    // result.display();

    return 0;
}