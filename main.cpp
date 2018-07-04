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

#define WIDTH   1920
#define HEIGHT  1080
#define BORDAL  5
#define PIXEL_BORDA 40
#define SPACE_TEXT  20

using namespace std;
using namespace Eigen;
using namespace cv;

static const char short_options[] = "m:l:r:";

static const struct option
        long_options[] = {
		{ "map",        required_argument, NULL, 'm' },
		{ "label",      required_argument, NULL, 'l' },
		{ "reference",  required_argument, NULL, 'r' },
        { 0, 	                        0, 	  0,  0  }  // Para casos em que o argumento não eh obrigatório use no_argument
};

static void usage(int argc, char **argv)
{
    cerr << "Usage " << argv[0] << " [options]"<<endl<<endl;
    cerr << "Options:"<< endl;
    cerr << "-m | --map    File with map to load"<<endl;
    cerr << "-l | --label  Label to map"<<endl;
    cerr << "-r | --reference Index of reference"<<endl;
    cerr << "exemplo: " << endl;
    cerr << argv[0] << " -m /home/ota/cluster/KITTI/data_odometry_poses/dataset/poses/00.txt -l Referencia "
                       "-m /home/ota/workspace/OTA_SLAM/ORB_SLAM2_Estatistica/2000KEYPOINTSDELL.1000Mbps_iteracao_3/i1a1_1_1KeyFrameTrajectory.txt -l Teste "
                       "-m /home/ota/workspace/OTA_SLAM/ORB_SLAM2_Estatistica/2000KEYPOINTSDELL.1000Mbps_iteracao_11/KeyFrameTrajectory_referencia.txt -l ORB-SLAM "
                       "-r 0"<<endl;
}

vector<Matrix4f> load_file(string arquivo)
{
    vector<Matrix4f> mapa;
    ifstream stream;
    Matrix4f matriz = Matrix4f::Zero();
    matriz << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    stream.open(arquivo);

    mapa.push_back(matriz);
    while(!stream.eof()){
        stream >> matriz(0, 0) >> matriz(0, 1) >> matriz(0, 2) >> matriz(0, 3)
               >> matriz(1, 0) >> matriz(1, 1) >> matriz(1, 2) >> matriz(1, 3)
               >> matriz(2, 0) >> matriz(2, 1) >> matriz(2, 2) >> matriz(2, 3);

        mapa.push_back(matriz);
    }

    stream.close();

    return mapa;
}

int main(int argc, char **argv)
{
    vector<string> labels;
    vector<vector<Matrix4f> > maps;
    Mat imagem(HEIGHT, WIDTH, CV_8UC3, Scalar::all(255));  // A matriz que vai armazenar a imagem
    unsigned int corAtual = 0;
    unsigned int rindex = 0;
    Vector2f refX;
    float refmenorx;
    float refmaiorx;
    float refmenory;
    float refmaiory;
    Vector2f X;
    float xi, yi;
    float xf, yf;
    vector<Scalar> cores = {    Scalar(255,   0,   0),
                                Scalar(0,   255,   0),
                                Scalar(0,     0, 255),
                                Scalar(255,   0, 255),
                                Scalar(0,   255, 255),
                                Scalar(125, 125, 125),
                                Scalar(125, 125, 255),
                                Scalar(125, 225, 125),
                                Scalar(225, 125, 125),
                                Scalar(0,   125, 125),
                                Scalar(125,   0, 125),
                                Scalar(125, 125,   0)};

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
		default:
			usage(argc, argv);
			exit(EXIT_FAILURE);
		}
	}

	if(maps.size()==0){
        usage(argc, argv);
        exit(EXIT_FAILURE);
	}

	for(unsigned int i=0; i<maps.size(); i++){

        // Quando chegar aqui já carregou todos os mapas. Agora é preciso encontrar o menor e maior valor para x e z
        float menorx = maps[i][0](0, 3);
        float maiorx = maps[i][0](0, 3);

        float menorz = maps[i][0](2, 3);
        float maiorz = maps[i][0](2, 3);


        for(unsigned int j=0; j<maps[i].size(); j++){
            if(maps[i][j](0, 3)<menorx) menorx = maps[i][j](0, 3);
            if(maps[i][j](0, 3)>maiorx) maiorx = maps[i][j](0, 3);

            if(maps[i][j](2, 3)<menorz) menorz = maps[i][j](2, 3);
            if(maps[i][j](2, 3)>maiorz) maiorz = maps[i][j](2, 3);
        }

        Matrix2f A;
        Vector2f B;

        A << menorz, 1,
             maiorz, 1;
        B << imagem.rows-PIXEL_BORDA, PIXEL_BORDA;  // Esta usando valores diferentes dos máximos para ficar uma borda

        X = A.colPivHouseholderQr().solve(B);

        if(rindex==i){
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

	// Vamos por o label na esquerda da imagem
	corAtual=0;
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

	imwrite("imagem.png", imagem);

    return EXIT_SUCCESS;
}
