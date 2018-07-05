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

#include "evaluateKITTI.h"

#include <libgen.h>

float lengths[] = {100,200,300,400,500,600,700,800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000,
                   2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700};
unsigned int num_lengths = 37;

vector<float> trajectoryDistances(vector<Matrix4f> &poses)
{
    vector<float> dist;
    dist.push_back(0);
    for (unsigned i=1; i<poses.size(); i++) {
        Matrix4f P1 = poses[i-1];
        Matrix4f P2 = poses[i];
        float dx = P1(0, 3)-P2(0, 3);
        float dy = P1(1, 3)-P2(1, 3);
        float dz = P1(2, 3)-P2(2, 3);
        dist.push_back(dist[i-1]+sqrt(dx*dx+dy*dy+dz*dz));
  }
  return dist;
}

unsigned int lastFrameFromSegmentLength(vector<float> &dist, unsigned int first_frame,float len)
{
  for (unsigned int i=first_frame; i<dist.size(); i++)
    if (dist[i]>dist[first_frame]+len)
      return i;
  return -1;
}

inline float rotationError(Matrix4f &pose_error)
{
  float a = pose_error(0, 0);
  float b = pose_error(1, 1);
  float c = pose_error(2, 2);
  float d = 0.5*(a+b+c-1.0);
  return acos(max(min(d,1.0f),-1.0f));
}

inline float translationError(Matrix4f &pose_error)
{
  float dx = pose_error(0, 3);
  float dy = pose_error(1, 3);
  float dz = pose_error(2, 3);
  return sqrt(dx*dx+dy*dy+dz*dz);
}

vector<errors> calcSequenceErrors(vector<Matrix4f> &poses_gt, vector<Matrix4f> &poses_result)
{

  // error vector
  vector<errors> err;

  // parameters
  unsigned int step_size = 10; // every second

  // pre-compute distances (from ground truth as reference)
  vector<float> dist = trajectoryDistances(poses_gt);

  // for all start positions do
  for (unsigned int first_frame=0; first_frame<poses_gt.size(); first_frame+=step_size) {

    // for all segment lengths do
    for (unsigned int i=0; i<num_lengths; i++) {

      // current length
      float len = lengths[i];

      // compute last frame
      int32_t last_frame = lastFrameFromSegmentLength(dist, first_frame,len);

      // continue, if sequence not long enough
      if (last_frame==-1)
        continue;

      // compute rotational and translational errors
      Matrix4f pose_delta_gt     = poses_gt[first_frame].inverse()*poses_gt[last_frame];
      Matrix4f pose_delta_result = poses_result[first_frame].inverse()*poses_result[last_frame];
      Matrix4f pose_error        = pose_delta_result.inverse()*pose_delta_gt;
      float r_err = rotationError(pose_error);
      float t_err = translationError(pose_error);

      // compute speed
      float num_frames = (float)(last_frame-first_frame+1);
      float speed = len/(0.1*num_frames);

      // write to file
      err.push_back(errors(first_frame,r_err/len,t_err/len,len,speed));
    }
  }

  // return error vector
  return err;
}

void saveSequenceErrors(vector<errors> &err,string file_name)
{

    // open file
    FILE *fp;
    fp = fopen(file_name.c_str(),"w");

    // write to file
    for (vector<errors>::iterator it=err.begin(); it!=err.end(); it++){
        fprintf(fp,"%d %f %f %f %f\n",it->first_frame, it->r_err, it->t_err, it->len, it->speed);
    }

    // close file
    fclose(fp);
}

void savePathPlot(vector<Matrix4f> &poses_gt,vector<Matrix4f> &poses_result,string file_name)
{

  // parameters
  int32_t step_size = 3;

  // open file
  FILE *fp = fopen(file_name.c_str(),"w");

  // save x/z coordinates of all frames to file
  for (unsigned int i=0; i<poses_gt.size(); i+=step_size)
    fprintf(fp,"%f %f %f %f\n",poses_gt[i](0, 3),poses_gt[i](2, 3), poses_result[i](0, 3),poses_result[i](2, 3));

  // close file
  fclose(fp);
}

vector<int> computeRoi(vector<Matrix4f> &poses_gt,vector<Matrix4f> &poses_result)
{
    float x_min = numeric_limits<int32_t>::max();
    float x_max = numeric_limits<int32_t>::min();
    float z_min = numeric_limits<int32_t>::max();
    float z_max = numeric_limits<int32_t>::min();

    for(unsigned int i=0; i<poses_gt.size(); i++) {
        float x = poses_gt[i](0, 3);
        float z = poses_gt[i](2, 3);
        if (x<x_min) x_min = x; if (x>x_max) x_max = x;
        if (z<z_min) z_min = z; if (z>z_max) z_max = z;
    }

    for(unsigned int i=0; i<poses_result.size(); i++){
        float x = poses_result[i](0, 3);
        float z = poses_result[i](2, 3);
        if (x<x_min) x_min = x; if (x>x_max) x_max = x;
        if (z<z_min) z_min = z; if (z>z_max) z_max = z;
    }

    float dx = 1.1*(x_max-x_min);
    float dz = 1.1*(z_max-z_min);
    float mx = 0.5*(x_max+x_min);
    float mz = 0.5*(z_max+z_min);
    float r  = 0.5*max(dx,dz);

    vector<int32_t> roi;
    roi.push_back((int32_t)(mx-r));
    roi.push_back((int32_t)(mx+r));
    roi.push_back((int32_t)(mz-r));
    roi.push_back((int32_t)(mz+r));

    return roi;
}

void plotPathPlot(string dir,vector<int> &roi,string mapa, string gt, string vo, Scalar cor1, Scalar cor2)
{

  // gnuplot file name
  char command[1024];
  char file_name[256];
  sprintf(file_name,"%s.gp",mapa.c_str());
  string full_name = dir + "/" + file_name;

  // create png + eps
  for (unsigned int i=0; i<2; i++) {

    // open file
    FILE *fp = fopen(full_name.c_str(),"w");

    fprintf(fp,"set encoding utf8\n");

    // save gnuplot instructions
    if(i==0) {
      fprintf(fp,"set term png size 900,900\n");
      fprintf(fp,"set output \"%s.png\"\n",mapa.c_str());
    } else {
      fprintf(fp,"set term postscript eps enhanced color\n");
      fprintf(fp,"set output \"%s.eps\"\n",mapa.c_str());
    }

    fprintf(fp,"set size ratio -1\n");
    fprintf(fp,"set xrange [%d:%d]\n",roi[0],roi[1]);
    fprintf(fp,"set yrange [%d:%d]\n",roi[2],roi[3]);
    fprintf(fp,"set xlabel \"x [m]\"\n");
    fprintf(fp,"set ylabel \"z [m]\"\n");
    fprintf(fp,"plot \"%s\" using 1:2 lc rgb \"#%02X%02X%02X\" title '%s' w lines,",mapa.c_str(),
        (unsigned int)cor1[0], (unsigned int)cor1[1], (unsigned int)cor1[2], gt.c_str());
    fprintf(fp,"\"%s\" using 3:4 lc rgb \"#%02X%02X%02X\" title '%s' w lines,",mapa.c_str(),
        (unsigned int)cor2[0], (unsigned int)cor2[1], (unsigned int)cor2[2], vo.c_str());
    fprintf(fp,"\"< head -1 %s\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Início' w points\n",mapa.c_str());

    // close file
    fclose(fp);

    // run gnuplot => create png + eps
    sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
    system(command);
  }

  // create pdf and crop
  sprintf(command,"cd %s; ps2pdf %s.eps %s_large.pdf",dir.c_str(),mapa.c_str(),mapa.c_str());
  system(command);
  sprintf(command,"cd %s; pdfcrop %s_large.pdf %s.pdf",dir.c_str(),mapa.c_str(),mapa.c_str());
  system(command);
  sprintf(command,"cd %s; rm %s_large.pdf",dir.c_str(),mapa.c_str());
  system(command);
}

void saveErrorPlots(vector<errors> &seq_err,string plot_error_dir,string prefix)
{

  // file names
  char file_name_tl[1024]; sprintf(file_name_tl,"%s/%s_tl.txt",plot_error_dir.c_str(),prefix.c_str());
  char file_name_rl[1024]; sprintf(file_name_rl,"%s/%s_rl.txt",plot_error_dir.c_str(),prefix.c_str());
  char file_name_ts[1024]; sprintf(file_name_ts,"%s/%s_ts.txt",plot_error_dir.c_str(),prefix.c_str());
  char file_name_rs[1024]; sprintf(file_name_rs,"%s/%s_rs.txt",plot_error_dir.c_str(),prefix.c_str());

  // open files
  FILE *fp_tl = fopen(file_name_tl,"w");
  FILE *fp_rl = fopen(file_name_rl,"w");
  FILE *fp_ts = fopen(file_name_ts,"w");
  FILE *fp_rs = fopen(file_name_rs,"w");

  // for each segment length do
  for (unsigned int i=0; i<num_lengths; i++) {

    float t_err = 0;
    float r_err = 0;
    float num   = 0;

    // for all errors do
    for (vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
      if (fabs(it->len-lengths[i])<1.0) {
        t_err += it->t_err;
        r_err += it->r_err;
        num++;
      }
    }

    // we require at least 3 values
    if (num>2.5) {
      fprintf(fp_tl,"%f %f\n",lengths[i],t_err/num);
      fprintf(fp_rl,"%f %f\n",lengths[i],r_err/num);
    }
  }

  // for each driving speed do (in m/s)
  for (float speed=2; speed<25; speed+=2) {

    float t_err = 0;
    float r_err = 0;
    float num   = 0;

    // for all errors do
    for (vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
      if (fabs(it->speed-speed)<2.0) {
        t_err += it->t_err;
        r_err += it->r_err;
        num++;
      }
    }

    // we require at least 3 values
    if (num>2.5) {
      fprintf(fp_ts,"%f %f\n",speed,t_err/num);
      fprintf(fp_rs,"%f %f\n",speed,r_err/num);
    }
  }

  // close files
  fclose(fp_tl);
  fclose(fp_rl);
  fclose(fp_ts);
  fclose(fp_rs);
}

void plotErrorPlotsComb(string dir, vector<string> &labels, unsigned int exclui, vector<Scalar> &cores)
{
    char command[4096];

    string prefix = string("err_plot_comb.txt");

    // for all four error plots do
    for(unsigned int i=0; i<4; i++){

        // create suffix
        char suffix[16];

        switch(i){
        case 0:
            sprintf(suffix,"tl");
            break;  // translacao por distancia
        case 1:
            sprintf(suffix,"rl");
            break;  // rotacao por distancia
        case 2:
            sprintf(suffix,"ts");
            break;  // translacao por velocidade
        case 3:
            sprintf(suffix,"rs");
            break;  // rotacao por velocidade
        }

        // gnuplot file name
        char file_name[4096];
        char full_name[4096];
        sprintf(file_name,"%s_%s.gp",   prefix.c_str(), suffix);
        sprintf(full_name,"%s/%s",      dir.c_str(),    file_name);

        // create png + eps
        for(unsigned int j=0; j<2; j++) {

            // open file
            FILE *fp = fopen(full_name,"w");

            fprintf(fp,"set encoding utf8\n");

            // save gnuplot instructions
            if(j==0){
                fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
                fprintf(fp,"set output \"%s_%s.png\"\n",prefix.c_str(),suffix);
            }else{
                fprintf(fp,"set term postscript eps enhanced color\n");
                fprintf(fp,"set output \"%s_%s.eps\"\n",prefix.c_str(),suffix);
            }

            // start plot at 0
            fprintf(fp,"set size ratio 0.5\n");
            fprintf(fp,"set yrange [0:*]\n");

            // x label
            if(i<=1)
                fprintf(fp,"set xlabel \"Distância Percorrida [m]\"\n");
            else
                fprintf(fp,"set xlabel \"Velocidade [km/h]\"\n");

            // y label
            if(i==0 || i==2)
                fprintf(fp,"set ylabel \"Erro de Translação[%%]\"\n");
            else
                fprintf(fp,"set ylabel \"Erro de Rotação [grau/m]\"\n");

            // plot error curve
            fprintf(fp,"plot \\\n");
            for(unsigned int k=0; k<labels.size(); k++){
                if(k!=exclui){
                    fprintf(fp, "\"%s_%s.txt\" using ", (labels[k] + "-err_plot.txt").c_str(), suffix);
                    switch(i){
                    case 0:
                        fprintf(fp,"1:($2*100) title '%s'",         labels[k].c_str());
                        break;
                    case 1:
                        fprintf(fp,"1:($2*57.3) title '%s'",        labels[k].c_str());
                        break;
                    case 2:
                        fprintf(fp,"($1*3.6):($2*100) title '%s'",  labels[k].c_str());
                        break;
                    case 3:
                        fprintf(fp,"($1*3.6):($2*57.3) title '%s'", labels[k].c_str());
                        break;
                    }
                    if(k==labels.size()-1 || (k+1==exclui && exclui==labels.size()-1)){
                        fprintf(fp," lc rgb \"#%02X%02X%02X\" pt 4 w linespoints\n",  (unsigned int)cores[k][2],
                                                                                (unsigned int)cores[k][1],
                                                                                (unsigned int)cores[k][0]);
                    }else{
                        fprintf(fp," lc rgb \"#%02X%02X%02X\" pt 4 w linespoints, \\\n", (unsigned int)cores[k][2],
                                                                                (unsigned int)cores[k][1],
                                                                                (unsigned int)cores[k][0]);
                    }
                }
            }

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
            system(command);
        }

        // create pdf and crop
        sprintf(command,"cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",dir.c_str(),prefix.c_str(),suffix,prefix.c_str(),suffix);
        system(command);
        sprintf(command,"cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",dir.c_str(),prefix.c_str(),suffix,prefix.c_str(),suffix);
        system(command);
        sprintf(command,"cd %s; rm %s_%s_large.pdf",dir.c_str(),prefix.c_str(),suffix);
        system(command);
    }
}

void plotErrorPlots(string dir, string label, Scalar cor)
{
    char command[4096];

    string prefix = label+string("-err_plot.txt");

    // for all four error plots do
    for(unsigned int i=0; i<4; i++){

        // create suffix
        char suffix[16];

        switch(i){
        case 0:
            sprintf(suffix,"tl");
            break;  // translacao por distancia
        case 1:
            sprintf(suffix,"rl");
            break;  // rotacao por distancia
        case 2:
            sprintf(suffix,"ts");
            break;  // translacao por velocidade
        case 3:
            sprintf(suffix,"rs");
            break;  // rotacao por velocidade
        }

        // gnuplot file name
        char file_name[4096];
        char full_name[4096];
        sprintf(file_name,"%s_%s.gp",   prefix.c_str(), suffix);
        sprintf(full_name,"%s/%s",      dir.c_str(),    file_name);

        // create png + eps
        for(unsigned int j=0; j<2; j++) {

            // open file
            FILE *fp = fopen(full_name,"w");

            fprintf(fp,"set encoding utf8\n");

            // save gnuplot instructions
            if(j==0){
                fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
                fprintf(fp,"set output \"%s_%s.png\"\n",prefix.c_str(),suffix);
            }else{
                fprintf(fp,"set term postscript eps enhanced color\n");
                fprintf(fp,"set output \"%s_%s.eps\"\n",prefix.c_str(),suffix);
            }

            // start plot at 0
            fprintf(fp,"set size ratio 0.5\n");
            fprintf(fp,"set yrange [0:*]\n");

            // x label
            if(i<=1)
                fprintf(fp,"set xlabel \"Distância Percorrida [m]\"\n");
            else
                fprintf(fp,"set xlabel \"Velocidade [km/h]\"\n");

            // y label
            if(i==0 || i==2)
                fprintf(fp,"set ylabel \"Erro de Translação[%%]\"\n");
            else
                fprintf(fp,"set ylabel \"Erro de Rotação [grau/m]\"\n");

            // plot error curve
            fprintf(fp,"plot \"%s_%s.txt\" using ",prefix.c_str(), suffix);
            switch(i){
            case 0:
                fprintf(fp,"1:($2*100) title '%s'",         label.c_str());
                break;
            case 1:
                fprintf(fp,"1:($2*57.3) title '%s'",        label.c_str());
                break;
            case 2:
                fprintf(fp,"($1*3.6):($2*100) title '%s'",  label.c_str());
                break;
            case 3:
                fprintf(fp,"($1*3.6):($2*57.3) title '%s'", label.c_str());
                break;
            }
            fprintf(fp," lc rgb \"#%02X%02X%02X\" pt 4 w linespoints\n",    (unsigned int)cor[2],
                                                                            (unsigned int)cor[1],
                                                                            (unsigned int)cor[0]);

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
            system(command);
        }

        // create pdf and crop
        sprintf(command,"cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",dir.c_str(),prefix.c_str(),suffix,prefix.c_str(),suffix);
        system(command);
        sprintf(command,"cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",dir.c_str(),prefix.c_str(),suffix,prefix.c_str(),suffix);
        system(command);
        sprintf(command,"cd %s; rm %s_%s_large.pdf",dir.c_str(),prefix.c_str(),suffix);
        system(command);
    }
}

void saveStats(vector<errors> err,string dir)
{

  float t_err = 0;
  float r_err = 0;

  // for all errors do => compute sum of t_err, r_err
  for (vector<errors>::iterator it=err.begin(); it!=err.end(); it++) {
    t_err += it->t_err;
    r_err += it->r_err;
  }

  // open file
  FILE *fp = fopen((dir + "/stats.txt").c_str(),"w");

  // save errors
  float num = err.size();
  fprintf(fp,"%f %f\n",t_err/num,r_err/num);

  // close file
  fclose(fp);
}

bool eval(vector<string> &labels, vector<vector<Matrix4f> > &maps, unsigned int rindex, string diretorio, vector<Scalar> &cores)
{
    // total errors
    vector<errors> total_err;

    for(unsigned int i=0; i<maps.size(); i++){
        if(i != rindex){
            // plot status
            std::cerr << "Processing: "<<labels[i]<<", poses: "<<maps[i].size()<<"/"<<maps[rindex].size()<<std::endl;

            // check for errors
            if (maps[i].size()==0 || maps[i].size()!=maps[rindex].size()) {
                std::cerr << "ERROR: Couldn't read (all) poses of: "<<labels[i]<<std::endl;
                return false;
            }

            // compute sequence errors
            vector<errors> seq_err = calcSequenceErrors(maps[rindex], maps[i]);
            saveSequenceErrors(seq_err, diretorio + "/" + labels[i] + "-seq_err.txt");

            // add to total errors
            total_err.insert(total_err.end(), seq_err.begin(), seq_err.end());

            // save + plot bird's eye view trajectories
            savePathPlot(maps[rindex], maps[i], diretorio + "/" + labels[i] + "-path_plot.txt");

            vector<int> roi = computeRoi(maps[rindex], maps[i]);
            plotPathPlot(diretorio, roi, labels[i] + string("-path_plot.txt"), labels[rindex], labels[i], cores[rindex],
                                                                                                cores[i]);

            // save + plot individual errors
            saveErrorPlots(seq_err, diretorio, labels[i] + string("-err_plot.txt"));
            plotErrorPlots(diretorio, labels[i], cores[i]);
        }
    }

    plotErrorPlotsComb(diretorio, labels, rindex, cores);

    // save + plot total errors + summary statistics
    if (total_err.size()>0) {
        char prefix[16];
        sprintf(prefix,"avg");
        /*saveErrorPlots(total_err,diretorio,prefix);
        plotErrorPlots(diretorio,prefix);*/
        saveStats(total_err,diretorio);
    }
    // success
	return true;
}
