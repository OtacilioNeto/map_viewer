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

#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdlib.h>
#include <getopt.h>
#include <locale.h>

#include "evaluateKITTI.h"

#define WIDTH   1920
#define HEIGHT  1080
#define BORDAL  5
#define PIXEL_BORDA 40
#define SPACE_TEXT  20

using namespace std;
using namespace Eigen;
using namespace cv;

static const char short_options[] = "m:l:r:d:";

static const struct option
        long_options[] = {
		{ "map",        required_argument, NULL, 'm' },
		{ "label",      required_argument, NULL, 'l' },
		{ "reference",  required_argument, NULL, 'r' },
		{ "directory",  required_argument, NULL, 'd' },
        { 0, 	                        0, 	  0,  0  }  // Para casos em que o argumento não eh obrigatório use no_argument
};

static void usage(int argc, char **argv)
{
    cerr << "Usage " << argv[0] << " [options]"<<endl<<endl;
    cerr << "Options:"<< endl;
    cerr << "-m | --map       File with map to load"<<endl;
    cerr << "-l | --label     Label to map"<<endl;
    cerr << "-r | --reference Index of reference"<<endl;
    cerr << "-d | --directory Directory to store results"<<endl;
    cerr << "example: " << endl;
    cerr << argv[0] << " -m /home/ota/cluster/KITTI/data_odometry_poses/dataset/poses/00.txt -l Referencia "
                       "-m /home/ota/workspace/OTA_SLAM/ORB_SLAM2_Estatistica/2000KEYPOINTSDELL.1000Mbps_iteracao_3/i1a1_1_1KeyFrameTrajectory.txt -l Teste "
                       "-m /home/ota/workspace/OTA_SLAM/ORB_SLAM2_Estatistica/2000KEYPOINTSDELL.1000Mbps_iteracao_11/KeyFrameTrajectory_referencia.txt -l ORB-SLAM "
                       "-r 0 "
                       "-d /home/ota/workspace/OTA_SLAM/ORB_SLAM2_Estatistica/2000KEYPOINTSDELL.1000Mbps_iteracao_3/i1a1_1_1.dir"<<endl;
}

vector<Matrix4f> load_file(string arquivo)
{
    vector<Matrix4f> mapa;
    ifstream stream;
    Matrix4f matriz;
    matriz << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    stream.open(arquivo);

    stream  >> matriz(0, 0) >> matriz(0, 1) >> matriz(0, 2) >> matriz(0, 3)
            >> matriz(1, 0) >> matriz(1, 1) >> matriz(1, 2) >> matriz(1, 3)
            >> matriz(2, 0) >> matriz(2, 1) >> matriz(2, 2) >> matriz(2, 3);
    while(stream.good()){
        mapa.push_back(matriz);

        stream >> matriz(0, 0) >> matriz(0, 1) >> matriz(0, 2) >> matriz(0, 3)
               >> matriz(1, 0) >> matriz(1, 1) >> matriz(1, 2) >> matriz(1, 3)
               >> matriz(2, 0) >> matriz(2, 1) >> matriz(2, 2) >> matriz(2, 3);
    }

    stream.close();

    return mapa;
}

void smallxz(vector<Matrix4f> &mapa, float &menorx, float &maiorx, float &menorz, float &maiorz)
{
    menorx = mapa[0](0, 3);
    maiorx = mapa[0](0, 3);

    menorz = mapa[0](2, 3);
    maiorz = mapa[0](2, 3);

    for(unsigned int j=0; j<mapa.size(); j++){
        if(mapa[j](0, 3)<menorx) menorx = mapa[j](0, 3);
        if(mapa[j](0, 3)>maiorx) maiorx = mapa[j](0, 3);

        if(mapa[j](2, 3)<menorz) menorz = mapa[j](2, 3);
        if(mapa[j](2, 3)>maiorz) maiorz = mapa[j](2, 3);
    }
}

void desenhaBordasLabel(vector<string> &labels, Mat &imagem, vector<Scalar> &cores, Vector2f &refX,
    float refmenorx, float refmaiorx, float refmenory, float refmaiory)
{
    float xi, yi;
    float xf, yf;
    unsigned int corAtual = 0;
    // Vamos por o label na esquerda da imagem
	unsigned int addy = PIXEL_BORDA;

	for(unsigned int i=0; i<labels.size(); i++){
        string texto = labels[i];
        int fontFace = FONT_HERSHEY_SIMPLEX;
        double fontScale = 1;
        int thickness = 2;
        int baseline=0;

        Size textSize = getTextSize(texto, fontFace, fontScale, thickness, &baseline);
        Point textOrg(BORDAL, addy+textSize.height);
        putText(imagem, texto, textOrg, fontFace, fontScale, cores[corAtual], thickness, 8);

        addy += textSize.height + SPACE_TEXT;
        //cout << addy <<endl;
        corAtual++;
	}

    xi = -refmenorx*refX(0) + refX(1);
    yi =  refmenory*refX(0) + refX(1);

    xf = -refmaiorx*refX(0) + refX(1);
    yf =  refmaiory*refX(0) + refX(1);

    rectangle(imagem, Point(xi, yi), Point(xf, yf), Scalar(0, 0, 0), 2, LINE_AA);

    for(int i=-300; i<=300; i+=100){
        string texto;
        int fontFace = FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int thickness = 1;
        int baseline=0;

        texto = to_string(i) + " m";
        Size textSize = getTextSize(texto, fontFace, fontScale, thickness, &baseline);

        xi = -((float)i)*refX(0) + refX(1);
        Point textOrg(xi, yi-textSize.height);
        putText(imagem, texto, textOrg, fontFace, fontScale, Scalar(255, 0, 0), thickness, LINE_AA );
        line(imagem, Point(xi, yi-10), Point(xi, yi+10), Scalar(0, 0, 0) , 2, LINE_AA);
    }

    for(int i=0; i<=600; i+=50){
        string texto;
        int fontFace = FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int thickness = 1;

        texto = to_string(i) + " m";

        yi = ((float)i)*refX(0) + refX(1);
        Point textOrg(xf+15, yi);
        putText(imagem, texto, textOrg, fontFace, fontScale, Scalar(255, 0, 0), thickness, LINE_AA );
        line(imagem, Point(xf-10, yi), Point(xf+10, yi), Scalar(0, 0, 0) , 2, LINE_AA);
    }
}

void translateScala(vector<vector<Matrix4f> > &maps, unsigned int rindex)
{
    Vector2f X, Z;
    float refmenorx;
    float refmaiorx;
    float refmenorz;
    float refmaiorz;

    smallxz(maps[rindex], refmenorx, refmaiorx, refmenorz, refmaiorz);
	for(unsigned int i=0; i<maps.size(); i++){
        float menorx, maiorx, menorz, maiorz;

        if(i != rindex){
            smallxz(maps[i], menorx, maiorx, menorz, maiorz);

            // Vamos encontrar a transformação linear que faz a mudança de escala da imagem para a referência
            Matrix2f A;
            Vector2f B;

            A << menorx, 1,
                 maiorx, 1;
            B << refmenorx, refmaiorx;  // Esta usando valores diferentes dos máximos para ficar uma borda

            X = A.colPivHouseholderQr().solve(B);

            A << menorz, 1,
                 maiorz, 1;
            B << refmenorz, refmaiorz;  // Esta usando valores diferentes dos máximos para ficar uma borda

            Z = A.colPivHouseholderQr().solve(B);

            for(unsigned int j=1; j<maps[i].size(); j++){
                maps[i][j](0, 3) = maps[i][j](0, 3)*X(0) + X(1);
                maps[i][j](2, 3) = maps[i][j](2, 3)*Z(0) + Z(1);
            }
        }
	}
}

int main(int argc, char **argv)
{
    vector<string> labels;
    vector<vector<Matrix4f> > maps;
    Mat imagem(HEIGHT, WIDTH, CV_8UC3, Scalar::all(255));  // A matriz que vai armazenar a imagem
    unsigned int rindex = 0;
    Vector2f refX;
    float refmenorx;
    float refmaiorx;
    float refmenory;
    float refmaiory;
    unsigned int corAtual = 0;
    Vector2f X;
    float xi, yi;
    float xf, yf;
    string diretorio;
    vector<Scalar> cores = {    Scalar(255,   0,   0),  // Blue Green Red
                                Scalar(0,   255,   0),
                                Scalar(0,     0, 255),
                                Scalar(255,   0, 255),
                                Scalar(246, 166,  38),
                                Scalar(125, 125, 125),
                                Scalar(125, 125, 255),
                                Scalar(125, 225, 125),
                                Scalar(225, 125, 125),
                                Scalar(0,   125, 125),
                                Scalar(125,   0, 125),
                                Scalar(125, 125,   0)};
    setlocale(LC_COLLATE, "");  // Adiciona suporte a impressao de caracteres especiaisi
    for (;;) {
        int idx;
        int c;

        c = getopt_long(argc, argv, short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c) {
        case 0: // getopt_long() flag
            break;
        case 'm':
            maps.push_back(load_file(optarg));
            break;
        case 'l':
            labels.push_back(optarg);
            break;
        case 'r':
            rindex = strtol(optarg, NULL, 10);
            break;
        case 'd':
            diretorio = optarg;
            break;
		default:
			usage(argc, argv);
			exit(EXIT_FAILURE);
		}
	}

	if(maps.size()==0 || diretorio.empty()){
        usage(argc, argv);
        exit(EXIT_FAILURE);
	}

	// Precisamos encontrar os menores valores do mapa de referência
	// para poder fazer a transformação de escala
	translateScala(maps, rindex);

	for(unsigned int i=0; i<maps.size(); i++){
        // Quando chegar aqui já carregou todos os mapas. Agora é preciso encontrar o menor e maior valor para x e z
        float menorx, maiorx, menorz, maiorz;

        smallxz(maps[i], menorx, maiorx, menorz, maiorz);

        // Vamos encontrar uma transformação linear que mapeia os pontos do mapa nos pontos da imagem
        Matrix2f A;
        Vector2f B;

        A << menorz, 1,
             maiorz, 1;
        B << imagem.rows-PIXEL_BORDA, PIXEL_BORDA;  // Esta usando valores diferentes dos máximos para ficar uma borda

        X = A.colPivHouseholderQr().solve(B);

        if(rindex == i){
            // Estamos processando a referência. Vamos salvar os valores para desenhar a caixa em torno

            refX = X;   // Salva a escala e o deslocamento
            refmenorx = menorx-30;
            refmaiorx = maiorx+20;

            refmenory = menorz-10;
            refmaiory = maiorz+10;
        }

        // Já temos as equações, vamos desenhar o mapa
        xi = -maps[i][0](0, 3)*X(0) + X(1);
        yi =  maps[i][0](2, 3)*X(0) + X(1);

        for(unsigned int j=1; j<maps[i].size(); j++){
            xf = -maps[i][j](0, 3)*X(0) + X(1);
            yf =  maps[i][j](2, 3)*X(0) + X(1);

            line(imagem, Point(xi, yi), Point(xf, yf), cores[corAtual], 2, LINE_AA);

            xi = xf;
            yi = yf;
        }

        corAtual++;

        if(corAtual>=cores.size()){
            corAtual = 0;
            cerr << "Warning - number of colors small than number of maps" << endl;
        }
    }

    desenhaBordasLabel(labels, imagem, cores, refX, refmenorx, refmaiorx, refmenory, refmaiory);

    system(("mkdir " + diretorio).c_str());

	imwrite(diretorio+"/00mapa.png", imagem);

	// Agora vamos fazer as analises de acordo com o dataset do KITTI

	eval(labels, maps, rindex, diretorio, cores);

    return EXIT_SUCCESS;
}
