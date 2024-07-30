/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/
#include "Matrix.h"
#include <stdlib.h>

namespace Matrix
{
Matrix_d::Matrix_d(uint32_t _row, uint32_t _col) :
    row(_row), col(_col)
{
    data = (double*)malloc(sizeof(double));
    // data = malloc()
};

double Matrix_d::operator()(uint32_t row_index, uint32_t col_index)
{
    assert(row_index < row && col_index < col);
    uint32_t offset = row_index * col + col_index;
    return data[offset];
}

Mat_33d::Mat_33d() : Matrix_d(3, 3) {};

Mat_34d::Mat_34d() : Matrix_d(3, 4) {};

Mat_44d::Mat_44d() : Matrix_d(4, 4) {};

} //namespace ORB_SLAM