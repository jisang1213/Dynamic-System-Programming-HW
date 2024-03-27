// Matrix library source code for KAIST ME454 Dynamics System Programming course
// 2024 March, Gunwoo Park

#include "../include/mymat.hpp"

// MatBase CLASS STARTS HERE

// Constructor of the MatBase class
// Initializes [rows] x [cols] zero matrix
// Throws error if [rows] or [cols] are bigger than the maximum value
MatBase::MatBase(int rows, int cols)
{
    if (rows > max_size) throw std::runtime_error("Error : rows exceeds the limit\n");
    if (cols > max_size) throw std::runtime_error("Error : cols exceeds the limit\n");

    m_rows = rows;
    m_cols = cols;

    // TODO : Implement zero matrix initialization
    matrix.resize(m_rows, std::vector<double>(m_cols,0.0));
}

// Destructor of the MatBase class
// Implement it if you need 
MatBase::~MatBase()
{
}

// Set an element in ([row], [col]) to [elem] of the matrix
void MatBase::set_elem(int row, int col, double elem)
{
    if (row >= m_rows) throw std::runtime_error("Error : row out of the range\n");
    if (col >= m_cols) throw std::runtime_error("Error : row out of the range\n");
    // TODO : Set [elem] to the container
    matrix[row][col] = elem;
}

// Returns an element in ([row], [col]) of the matrix
double MatBase::get_elem (int row, int col) const
{
    // TODO : Get [elem] from the container
    return matrix[row][col];
}

// Matrix visualization
// Already implemented
void MatBase::display ()
{
    std::cout << "Matrix : " << std::endl;
    for (int i_row = 0; i_row < m_rows; i_row++)
    {
        for (int i_col = 0; i_col < m_cols; i_col++) std::cout << get_elem(i_row, i_col) << "\t";
        std::cout << "\n";
    }
    std::cout << std::endl;
} 
// Assignment operator (=)
// Assigns the matrix in [mat] to [this] matrix
// Syntax : [this] = [mat]
// Already implemented
void MatBase::operator= (const MatBase &mat) 
{
    if (mat.m_rows != m_rows) throw std::runtime_error("Error : rows are not the same\n");
    if (mat.m_cols != m_cols) throw std::runtime_error("Error : cols are not the same\n");

    for (int i_row = 0; i_row < m_rows; i_row++) for (int i_col = 0; i_col < m_cols; i_col++)
        set_elem(i_row, i_col, mat.get_elem(i_row, i_col));
}

// Addition operator (+)
// Returns the [sum] of [this] and [mat]
// Syntax : [sum] = [this] + [mat]
MatBase MatBase::operator+ (const MatBase &mat) 
{
    if (mat.m_rows != m_rows) throw std::runtime_error("Error : rows are not the same\n");
    if (mat.m_cols != m_cols) throw std::runtime_error("Error : cols are not the same\n");

    MatBase sum(m_rows, m_cols);
    // TODO : Implement addition
    
    for (int i_row = 0; i_row < m_rows; i_row++) for (int i_col = 0; i_col < m_cols; i_col++)
        sum.set_elem(i_row, i_col, get_elem(i_row, i_col) + mat.get_elem(i_row, i_col));
    return sum;
}

// Subtraction operator (-)
// Returns the difference [diff] of [this] and [mat]
// Syntax : [diff] = [this] - [mat]
MatBase MatBase::operator- (const MatBase &mat) 
{
    if (mat.m_rows != m_rows) throw std::runtime_error("Error : rows are not the same\n");
    if (mat.m_cols != m_cols) throw std::runtime_error("Error : cols are not the same\n");

    MatBase diff(m_rows, m_cols);
    // TODO : Implement subtraction
    for (int i_row = 0; i_row < m_rows; i_row++) for (int i_col = 0; i_col < m_cols; i_col++)
        diff.set_elem(i_row, i_col, get_elem(i_row, i_col) - mat.get_elem(i_row, i_col));
    return diff;
}

// Matrix Multiplication operator (*) -> polymorphism
// Returns the product [prod] of [this] and [mat]
// Syntax : [prod] = [this] * [mat]
MatBase MatBase::operator* (const MatBase &mat) 
{
    if (mat.m_rows != m_cols) throw std::runtime_error("Error : the first rows and second cols are not the same\n");

    MatBase prod(m_rows, mat.m_cols);
    double sum;
    // TODO : Implement matrix multiplication
    for (int i_row = 0; i_row < m_rows; i_row++){
        for (int i_col = 0; i_col < mat.m_cols; i_col++){
            sum=0;
            for (int j = 0; j < m_cols; j++){
                sum+= get_elem(i_row, j) * mat.get_elem(j, i_col);
            }
            prod.set_elem(i_row, i_col, sum);
        }
    }
    return prod;
}

// Scalar Multiplication operator (*) -> polymorphism
// Returns the product of [this] and [val]
// Syntax : [prod] = [this] * [val]
MatBase MatBase::operator* (double val) 
{
    MatBase prod(m_rows, m_cols);
    // TODO : Implement scalar multiplication
    for (int i_row = 0; i_row < m_rows; i_row++) for (int i_col = 0; i_col < m_cols; i_col++)
        prod.set_elem(i_row, i_col, get_elem(i_row, i_col)*val);
    return prod;
}


// Mat33 CLASS STARTS HERE

// Constructor of the Mat33 class
// Initializes 3x3 zero matrix
Mat33::Mat33()
{
    m_rows = 3;
    m_cols = 3;
    // TODO : Implement zero matrix initialization
    matrix.resize(m_rows, std::vector<double>(m_cols,0.0));

}

// Constructor of the Mat33 class
// Initializes 3x3 diagonal matrix with [eii]
Mat33::Mat33(double e00, double e11, double e22)
{
    m_rows = 3;
    m_cols = 3;
    // TODO : Implement diagonal matrix initialization
    matrix.resize(m_rows, std::vector<double>(m_cols,0.0));
    matrix[0][0] = e00;
    matrix[1][1] = e11;
    matrix[2][2] = e22;
}

// Constructor of the Mat33 class
// Initializes 3x3 matrix with [eij]
Mat33::Mat33(  double e00, double e01, double e02,
                double e10, double e11, double e12,
                double e20, double e21, double e22  )
{
    m_rows = 3;
    m_cols = 3;
    // Implement matrix initialization (optional, this will not be evaluated.)
    matrix.resize(m_rows, std::vector<double>(m_cols,0.0));
    matrix[0][0] = e00;
    matrix[0][1] = e01;
    matrix[0][2] = e02;
    matrix[1][0] = e10;
    matrix[1][1] = e11;
    matrix[1][2] = e12;
    matrix[2][0] = e20;
    matrix[2][1] = e21;
    matrix[2][2] = e22;
}

// Transpose matrix calculation
// Returns the transpose of [this]
Mat33 Mat33::transpose()
{
    Mat33 trans;
    // TODO : Implement transpose
    for (int i_row = 0; i_row < m_rows; i_row++) for (int i_col = 0; i_col < m_cols; i_col++)
        trans.set_elem(i_col, i_row, get_elem(i_row, i_col));

    return trans;
}

// Determinant calculation (Optional)
// Returns the determinant of [this]
double Mat33::det()
{
    // Implement determinant calculation (optional, this will not be evaluated.)
    double a = matrix[0][0];
    double b = matrix[0][1];
    double c = matrix[0][2];
    double d = matrix[1][0];
    double e = matrix[1][1];
    double f = matrix[1][2];
    double g = matrix[2][0];
    double h = matrix[2][1];
    double i = matrix[2][2];

    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

// Inverse matrix calculation
// Returns the inverse of [this]
Mat33 Mat33::inverse()
{
    Mat33 inv;
    double dt = det();
    double a = matrix[0][0];
    double b = matrix[0][1];
    double c = matrix[0][2];
    double d = matrix[1][0];
    double e = matrix[1][1];
    double f = matrix[1][2];
    double g = matrix[2][0];
    double h = matrix[2][1];
    double i = matrix[2][2];
    // TODO : Implement inverse matrix calculation
    if(dt==0){
        std::cout << "matrix is singular" << std::endl;
        exit(1);
    }
    inv.matrix[0][0] = e*i-f*h;
    inv.matrix[0][1] = c*h-b*i;
    inv.matrix[0][2] = b*f-c*e;
    inv.matrix[1][0] = f*g-d*i;
    inv.matrix[1][1] = a*i-c*g;
    inv.matrix[1][2] = c*d-a*f;
    inv.matrix[2][0] = d*h-e*g;
    inv.matrix[2][1] = b*g-a*h;
    inv.matrix[2][2] = a*e-b*d;

    inv = inv*(1/dt);
    
    return inv;
}

// Vec3 CLASS STARTS HERE

// Constructor of the Vec3 class
// Initializes 3d zero vector (same as 3x1 matrix)
Vec3::Vec3()
{
    m_rows = 3;
    m_cols = 1;
    // TODO : Implement zero vector initialization
    matrix.resize(m_rows, std::vector<double>(m_cols,0.0));
}

// Constructor of the Vec3 class
// Initializes 3d vector with [e0, e1, e2]
Vec3::Vec3(double e0, double e1, double e2)
{
    m_rows = 3;
    m_cols = 1;
    // TODO : Implement vector initialization
    matrix.resize(m_rows, std::vector<double>(m_cols,0.0));
    matrix[0][0] = e0;
    matrix[1][0] = e1;
    matrix[2][0] = e2;
}

// Dot product calculation
// Returns the dot product of [this] and [vec]
double Vec3::dot(const Vec3 &vec)
{
    double prod = 0.0;
    // TODO : Implement dot product
    for(int i=0; i<3; i++){
        prod+=matrix[i][0]*vec.matrix[i][0];
    }

    return prod;
}

// Cross product calculation
// Returns the cross product of [this] and [vec]
Vec3 Vec3::cross(const Vec3 &vec)
{
    Vec3 prod;
    // TODO : Implement cross product
    prod.matrix[0][0] = matrix[1][0]*vec.matrix[2][0] - matrix[2][0]*vec.matrix[1][0];
    prod.matrix[1][0] = matrix[2][0]*vec.matrix[0][0] - matrix[0][0]*vec.matrix[2][0];
    prod.matrix[2][0] = matrix[0][0]*vec.matrix[1][0] - matrix[1][0]*vec.matrix[0][0];

    return prod;
}

// Skew symmetric matrix calculation
// Returns the skew symmetric matrix of [this]
Mat33 Vec3::skew()
{
    Mat33 mat;
    // TODO : Implement skew symmetric matrix calculation
    mat.set_elem(0,0, 0);
    mat.set_elem(0,1, -get_elem(2,0));
    mat.set_elem(0,2, get_elem(1,0));
    mat.set_elem(1,0, get_elem(2,0));
    mat.set_elem(1,1, 0);
    mat.set_elem(1,2, -get_elem(0,0));
    mat.set_elem(2,0, -get_elem(1,0));
    mat.set_elem(2,1, get_elem(0,0));
    mat.set_elem(2,2, 0);

    return mat;
}

// Squared norm calculation
// Returns the squared norm of [this]
double Vec3::squarednorm()
{
    double val = 0.0;
    // Implement squared norm calculation (optional, this will not be evaluated.)
    for(int i=0; i<3; i++){
        val+=matrix[i][0]*matrix[i][0];
    }

    return val;
}

// norm calculation
// Returns the norm of [this]
double Vec3::norm()
{
    double val = squarednorm();

    return std::sqrt(val);
}

void Vec3::normalize()
{
    double nm = norm();
    for(int i=0; i<3; i++){
        matrix[i][0] /= nm;
    }
}

Vec3 Vec3::normalized()
{
    Vec3 normalized_vec;
    double nm = norm();
    for(int i=0; i<3; i++){
        normalized_vec.set_elem(i,0, matrix[i][0] / nm);
    }
    return normalized_vec;
}

// Vector visualization
// Already implemented
void Vec3::display()
{
    std::cout << "Vector : " << std::endl;
    for (int i_row = 0; i_row < m_rows; i_row++)
    {
        for (int i_col = 0; i_col < m_cols; i_col++) std::cout << get_elem(i_row, i_col) << "\t";
        std::cout << "\n";
    }
    std::cout << std::endl;
} 