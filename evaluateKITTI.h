/**
* This file is part of map_viewer.
*
* Copyright (C) 2018 Otacílio de Araújo Ramos neto <otaciliodearaujo at gmail com>
* For more information see <https://github.com/OtacilioNeto/map_viewer>
*
* map_viewer is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* map_viewer is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with map_viewer. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __EVALUATEKITTI_H__

#define __EVALUATEKITTI_H__

#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

struct errors {
  int32_t first_frame;
  float   r_err;
  float   t_err;
  float   len;
  float   speed;
  errors (int32_t first_frame,float r_err,float t_err,float len,float speed) :
    first_frame(first_frame),r_err(r_err),t_err(t_err),len(len),speed(speed) {}
};

bool eval(vector<string> labels, vector<vector<Matrix4f> > maps, unsigned int rindex, string diretorio);

#endif
