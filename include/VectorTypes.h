/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/self-tuning
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)
 ***************************************************************************/

/**
 * @file VectorTypes.h
 * @author Tim Pfeifer
 * @date 19.05.2020
 * @brief Derived types for vectors, matrices and 2D/3D rotations.
 * @copyright GNU Public License.
 *
 */

#ifndef VECTORTYPES_H
#define VECTORTYPES_H

#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace libRSF
{
   /** index for matrix types */
  typedef Eigen::Index Index;
  const Index Dynamic = Eigen::Dynamic;

  /** generic matrix */
  template <typename T, int Row, int Col>
  using MatrixT = Eigen::Matrix<T, Row, Col, ((Col == 1) ? Eigen::ColMajor : Eigen::RowMajor)>;

  template <typename T, int Row>
  using VectorT = MatrixT<T, Row, 1>;

  /** double static template types */
  template <int Row, int Col>
  using MatrixStatic = MatrixT<double, Row, Col>;

  template <int Row>
  using VectorStatic = VectorT<double, Row>;

  /** double dynamic types */
  typedef VectorStatic<Dynamic> Vector;
  typedef MatrixStatic<Dynamic, Dynamic> Matrix;

  /** double static fixed types */
  typedef VectorStatic<1> Vector1;
  typedef VectorStatic<2> Vector2;
  typedef VectorStatic<3> Vector3;
  typedef VectorStatic<4> Vector4;
  typedef VectorStatic<6> Vector6;
  typedef VectorStatic<7> Vector7;
  typedef VectorStatic<8> Vector8;
  typedef VectorStatic<9> Vector9;
  typedef VectorStatic<12> Vector12;
  typedef VectorStatic<15> Vector15;

  typedef MatrixStatic<1, 1> Matrix11;
  typedef MatrixStatic<2, 2> Matrix22;
  typedef MatrixStatic<3, 3> Matrix33;
  typedef MatrixStatic<3, 4> Matrix34;
  typedef MatrixStatic<4, 3> Matrix43;
  typedef MatrixStatic<4, 4> Matrix44;
  typedef MatrixStatic<6, 6> Matrix66;
  typedef MatrixStatic<7, 7> Matrix77;
  typedef MatrixStatic<8, 8> Matrix88;
  typedef MatrixStatic<9, 9> Matrix99;
  typedef MatrixStatic<10, 10> Matrix1010;

  typedef MatrixStatic<2, Dynamic> Matrix2X;
  typedef MatrixStatic<3, Dynamic> Matrix3X;

  /** reference wrappers */
  template <typename T, int Row, int Col>
  using MatrixRef = Eigen::Map<MatrixT<T, Row, Col>>;

  template <typename T, int Row>
  using VectorRef = Eigen::Map<VectorT<T, Row>>;

  /** constant reference wrappers */
  template <typename T, int Row, int Col>
  using MatrixRefConst = Eigen::Map<const MatrixT<T, Row, Col>>;

  template <typename T, int Row>
  using VectorRefConst = Eigen::Map<const VectorT<T, Row>>;

  /** 2D rotation matrix */
  template <typename T>
  using Rotation2DT = Eigen::Rotation2D<T>;

  typedef Rotation2DT<double> Rotation2D;

  /** 3D quaternions */
  template <typename T>
  using QuaternionT = Eigen::Quaternion<T>;

  template <typename T>
  using QuaternionRef = Eigen::Map<QuaternionT<T>>;

  template <typename T>
  using QuaternionRefConst = Eigen::Map<const QuaternionT<T>>;

  typedef QuaternionT<double> Quaternion;

  template <typename T>
  QuaternionT<T> VectorToQuaternion(const VectorT<T,4> &Vec)
  {
    QuaternionT<T> Quat(Vec(3), Vec(0), Vec(1), Vec(2)); /**< storage order is different from constructor! */
    return Quat;
  }

  /** 3D angle-axis representation */
  template <typename T>
  using AngleAxisT = Eigen::AngleAxis<T>;

  typedef AngleAxisT<double> AngleAxis;

  /** safe STL contatianer */
  template <int Row, int Col>
  using MatrixVectorSTL = std::vector<MatrixT<double, Row, Col>, Eigen::aligned_allocator<MatrixT<double, Row, Col>>>;

  template <int Row>
  using VectorVectorSTL = MatrixVectorSTL<Row, 1>;
}

#endif // VECTORTYPES_H
