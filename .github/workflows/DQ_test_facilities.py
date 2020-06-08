"""(C) Copyright 2019 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
"""

from dqrobotics import *
import numpy


def get_list_of_dq_from_mat(name, mat):
    dq_mat = mat[name]
    dq_list = []
    for vec in numpy.transpose(dq_mat):
        dq_list.append(DQ(vec))
    return dq_list


def get_list_of_vector_from_mat(name, mat):
    vec_mat = mat[name]
    vec_list = []
    for vec in numpy.transpose(vec_mat):
        vec_list.append(vec)
    return vec_list


def get_list_of_matrices_from_mat(name, mat):
    mat_mat = mat[name]
    mat_list = []
    (row, col, index) = mat_mat.shape
    for i in range(0, index):
        mat_list.append(mat_mat[:, :, i])
    return mat_list


def get_point_from_dq(dq):
    return translation(normalize(dq))


def get_line_from_dq(dq, primitive):
    dq = normalize(dq)
    return Ad(rotation(dq), primitive) + E_ * cross(translation(dq), Ad(rotation(dq), primitive))


def get_plane_from_dq(dq, primitive):
    dq = normalize(dq)
    return Ad(rotation(dq), primitive) + E_ * dot(translation(dq), Ad(rotation(dq), primitive))
