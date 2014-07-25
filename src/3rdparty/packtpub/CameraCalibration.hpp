/*****************************************************************************
*   CameraCalibration.hpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef Example_MarkerBasedAR_CameraCalibration_hpp
#define Example_MarkerBasedAR_CameraCalibration_hpp

////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"

/**
 * A camera calibraiton class that stores intrinsic matrix
 * and distorsion vector.
 */
class CameraCalibration
{
public:
  CameraCalibration();
  CameraCalibration(float fx, float fy, float cx, float cy);
  CameraCalibration(float fx, float fy, float cx, float cy, float distorsionCoeff[4]);
  
  void getMatrix34(float cparam[3][4]) const;

  const Matrix33& getIntrinsic() const;
  const Vector4&  getDistorsion() const;
  
private:
  Matrix33 m_intrinsic;
  Vector4  m_distorsion;
};

CameraCalibration::CameraCalibration()
{
  
}

CameraCalibration::CameraCalibration(float fx, float fy, float cx, float cy)
{
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      m_intrinsic.mat[i][j] = 0;
  
  m_intrinsic.mat[0][0] = fx;
  m_intrinsic.mat[1][1] = fy;
  m_intrinsic.mat[0][2] = cx;
  m_intrinsic.mat[1][2] = cy;
  
  for (int i=0; i<4; i++)
    m_distorsion.data[i] = 0;
}


CameraCalibration::CameraCalibration(float fx, float fy, float cx, float cy, float distorsionCoeff[4])
{
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      m_intrinsic.mat[i][j] = 0;
  
  m_intrinsic.mat[0][0] = fx;
  m_intrinsic.mat[1][1] = fy;
  m_intrinsic.mat[0][2] = cx;
  m_intrinsic.mat[1][2] = cy;
  
  for (int i=0; i<4; i++)
    m_distorsion.data[i] = distorsionCoeff[i];
}

void CameraCalibration::getMatrix34(float cparam[3][4]) const
{
  for (int j=0; j<3; j++)
    for (int i=0; i<3; i++)
      cparam[i][j] = m_intrinsic.mat[i][j];
  
  for (int i=0; i<4; i++)
    cparam[3][i] = m_distorsion.data[i];
}

const Matrix33& CameraCalibration::getIntrinsic() const
{
  return m_intrinsic;
}

const Vector4&  CameraCalibration::getDistorsion() const
{
  return m_distorsion;
}

#endif
