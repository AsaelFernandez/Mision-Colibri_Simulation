#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <tgmath.h>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>

cv::Mat Transformada(cv::Mat real, cv::Mat complex);
cv::Mat Transformada_inversa(cv::Mat real, cv::Mat complex);
cv::Mat change_range(cv::Mat original, double minNew, double maxNew);

using namespace cv;
using namespace std;

//medidas en micras
const float PI = 3.141592653589793238463;

/*float M = 10192, N = 10192; //tamaño imagen
float dx = 2.5, dy = 2.5; //tamaño pixel, um
float pn = 0.2, cpn = 0.8; //porcentaje de ruido
float lambda = 0.67; //longitud de onda nm
float zt = 65000; //tamaño de la caja, 4.5cm real
float dmp = 15000; //diámetro de la masa de prueba, real 19000
float dsl = 10000; //diametro de la fuente de luz diametro real 4200
float z1, z2;*/

float M = 5096, N = 5096; //tamaño imagen
float dx = 5, dy = 5; //tamaño pixel, um
float pn = 0.2, cpn = 0.8; //porcentaje de ruido
float lambda = 0.67; //longitud de onda nm
float zt = 65000; //tamaño de la caja, 4.5cm real
float dmp = 15000; //diámetro de la masa de prueba, real 19000
float dsl = 10000; //diametro de la fuente de luz diametro real 4200
float z1, z2;

int main() {
	srand(time(NULL));
	z1 = zt / 2; //distancia entre fuente y pelota
	z2 = zt - z1; //distancia entre pelota y sensor

	float menosPI = (-1) * PI;

	//arreglo base coordenadas
	N = floor(N / 2) * 2;
	M = floor(M / 2) * 2;

	//centro de la imagen
	float cox = floor(M / 2) + 1;
	float coy = floor(N / 2) + 1;

	// comienza generacion de campo
	//************************************************************
	//******* define luz de entrada******************************

	//****************LED1**************************************

	float radtemp = (dsl / 2) * (dsl / 2);
	float x1, xx1, y1, yy1;

	cv::Mat temp(M, N, CV_32F), temp2(M, N, CV_32F);
	cv::Mat input(M, N, CV_32F);
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			//declarar
			x1 = (-cox + 1) + i;
			xx1 = x1 * dx;
			y1 = (-coy + 1) + j;
			yy1 = (y1 * dy) + 7000;
			temp.at<float>(i, j) = (pow(xx1, 2) + pow(yy1, 2)) / radtemp;
			if (temp.at<float>(i, j) < 1) {
				input.at<float>(i, j) = 1;
			}
			else {
				input.at<float>(i, j) = 0;
			}
		}
	}

	//**********************Fase LED1********************************
	cv::Mat noise(M, N, CV_32F), aux(M, N, CV_32F);
	cv::randu(noise, 0, 1);

	cv::Mat Ig(M, N, CV_32F);
	Ig = input * 255;

	cv::Mat tmp = (noise * 255 * pn);
	cv::Mat tmp2;
	aux = tmp.mul(input);
	aux.copyTo(tmp2);
	Ig = Ig * cpn + tmp2;

	float z0 = dmp * 1.2 / (2 * ((float)tan((int)15 * PI / 180))); //20 grados valor real led

	cv::Mat Ep_real(M, N, CV_32F), Ep_complex(M, N, CV_32F), Ep(M, N, CV_32F);

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			x1 = (-cox + 1) + i;
			xx1 = x1 * dx;
			y1 = (-coy + 1) + j;
			yy1 = y1 * dy + 7000;
			Ep_real.at<float>(i, j) = (float)cos((PI * (pow(xx1, 2) + pow(yy1, 2))) / ((float)z0 * lambda));
			Ep_complex.at<float>(i, j) = (float)sin((PI * (pow(xx1, 2) + pow(yy1, 2))) / ((float)z0 * lambda));
			Ep.at<float>(i, j) = atan2(Ep_complex.at<float>(i, j), Ep_real.at<float>(i, j));
		}
	}

	Ep = Ep * cpn + noise * pn * PI;

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			Ep_real.at<float>(i, j) = (float)cos(Ep.at<float>(i, j));
			Ep_complex.at<float>(i, j) = (float)sin(Ep.at<float>(i, j));
		}
	}

	cv::Mat Ig_real(M, N, CV_32F), Ig_complex(M, N, CV_32F);

	aux = Ig.mul(Ep_real);
	aux.copyTo(Ig_real);

	aux = Ig.mul(Ep_complex);
	aux.copyTo(Ig_complex);

	//**************************LED2********************************

	cv::Mat input2(M, N, CV_32F);
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			//declarar
			x1 = (-cox + 1) + i;
			xx1 = x1 * dx;
			y1 = (-coy + 1) + j; //pixeles
			yy1 = (y1 * dy) - 7000; //micras
			temp2.at<float>(i, j) = (pow(xx1, 2) + pow(yy1, 2)) / radtemp;
			if (temp2.at<float>(i, j) < 1) {
				input2.at<float>(i, j) = 1;
			}
			else {
				input2.at<float>(i, j) = 0;
			}
		}
	}

	imwrite("temp2.jpg",temp2);
	temp2.release();

	cv::randu(noise, 0, 1);

	cv::Mat Ig2(M, N, CV_32F);
	Ig2 = input2 * 255;

	tmp = (noise * 255 * pn);
	aux = tmp.mul(input2);
	aux.copyTo(tmp2);
	Ig2 = Ig2 * cpn + tmp2;

	//*****************LED2 Fase*****************************

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			x1 = (-cox + 1) + i;
			xx1 = x1 * dx;
			y1 = (-coy + 1) + j;
			yy1 = y1 * dy - 7000;
			Ep_real.at<float>(i, j) = (float)cos((PI * (pow(xx1, 2) + pow(yy1, 2))) / ((float)z0 * lambda));
			Ep_complex.at<float>(i, j) = (float)sin((PI * (pow(xx1, 2) + pow(yy1, 2))) / ((float)z0 * lambda));
			Ep.at<float>(i, j) = atan2(Ep_complex.at<float>(i, j), Ep_real.at<float>(i, j));
		}
	}

	Ep = Ep * cpn + noise * pn * PI;
	imwrite("noise.jpg", noise);
	noise.release();

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			Ep_real.at<float>(i, j) = (float)cos(Ep.at<float>(i, j));
			Ep_complex.at<float>(i, j) = (float)sin(Ep.at<float>(i, j));
		}
	}

	cv::Mat Ig2_real(M, N, CV_32F), Ig2_complex(M, N, CV_32F);

	aux = Ig2.mul(Ep_real);
	aux.copyTo(Ig2_real);

	aux = Ig2.mul(Ep_complex);
	aux.copyTo(Ig2_complex);

	imwrite("Ep.jpg", Ep);
	Ep.release();
	//**************UNIR LEDS*********************
	Ig = Ig + Ig2;

	Ig_real = Ig_real + Ig2_real;

	Ig_complex = Ig_complex + Ig2_complex;
	imwrite("input.jpg", input);
	input.release();
	imwrite("input2.jpg", input2);
	input2.release();
	imwrite("tmp2.jpg", tmp2);
	tmp2.release();
	imwrite("Ig2.jpg", Ig2);
	Ig2.release();
	imwrite("Ig2_real.jpg", Ig2_real);
	Ig2_real.release();
	imwrite("Ig2_complex.jpg", Ig2_complex);
	Ig2_complex.release();
	imwrite("Ig.jpg", Ig);
	Ig.release();

	/**********************************************************************/
	/***********************define kernel propagacion**********************/
	/**********************************************************************/

	//sampling frecuencia en el plano de fourier
	float dox = 1 / (M * dx);
	float doy = 1 / (M * dy);

	cv::Mat FP0(M, N, CV_32F);

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < M; j++) {
			x1 = (-cox + 1) + i;
			xx1 = x1 * dox;
			y1 = (-coy + 1) + j;
			yy1 = y1 * doy;
			FP0.at<float>(i, j) = sqrt(1 - lambda * lambda * (pow(xx1, 2) + pow(yy1, 2)));
		}
	}

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			Ep_real.at<float>(i, j) = (float)cos(2 * PI * z1 * (FP0.at<float>(j, i) / lambda));
			Ep_complex.at<float>(i, j) = (float)sin(2 * PI * z1 * (FP0.at<float>(j, i) / lambda));
		}
	}


	/**********************************************************************/
	/*****************************Fourier 1********************************/
	/**********************************************************************/
	//Intres= abs(Prop2)
	//cv::Mat Intres_real(M, N, CV_32F), Intres_complex(M, N, CV_32F), Intres(M, N, CV_32F);
	//mmax=max(max(Intres));

	float fox = 1 / (M * dx);
	float foy = 1 / (N * dy);

	//f2=fftshift(fft2(fftshift(Ig)))*dox*doy;
	cv::Mat f2_real(M, N, CV_32F);
	cv::Mat f1_real(M, N, CV_32F);
	cv::Mat f2_complex(M, N, CV_32F);
	cv::Mat f1_complex(M, N, CV_32F);
	cv::Mat resultado;
	cv::Mat canales[2];
	cv::Mat rangoAmp, rangoFase;

	resultado = Transformada(Ig_real, Ig_complex);
	imwrite("Ig_real.jpg", Ig_real);
	Ig_real.release();
	imwrite("Ig_complex.jpg", Ig_complex);
	Ig_complex.release();

	split(resultado, canales);  // canales[0] = Re(DFT(I), canales[1] = Im(DFT(I))

	f2_real = canales[0];
	f2_complex = canales[1];

	cv::Mat auxAmp(M, N, CV_32F), auxFase(M, N, CV_32F);
	magnitude(canales[0], canales[1], auxAmp);
	phase(canales[0], canales[1], auxFase);
	rangoAmp = change_range(auxAmp, (double)0, (double)255);
	rangoFase = change_range(auxFase, (double)menosPI, (double)PI);

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			f2_real.at<float>(i, j) *= (dox * doy);
			f2_complex.at<float>(i, j) *= (dox * doy);
		}
	}

	//Multiplicando la imagen de entrada por el factor de propagación
	f1_real = Ep_real.mul(f2_real);
	f1_complex = Ep_complex.mul(f2_complex);

	//Prop=fftshift(ifft2(fftshift(f1)))*fox*foy;
	cv::Mat Prop_real(M, N, CV_32F);
	cv::Mat Prop_complex(M, N, CV_32F);

	//Prop_real = Transformada_real(f1_real,f1_complex);

	resultado = Transformada_inversa(f1_real, f1_complex);
	split(resultado, canales);  // canales[0] = Re(DFT(I), canales[1] = Im(DFT(I))

	Prop_real = canales[0];
	Prop_complex = canales[1];

	magnitude(canales[0], canales[1], auxAmp);
	phase(canales[0], canales[1], auxFase);
	rangoAmp = change_range(auxAmp, 0, 255);
	rangoFase = change_range(auxFase, menosPI, PI);

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			Prop_real.at<float>(i, j) *= (fox * foy);
			Prop_complex.at<float>(i, j) *= (fox * foy);
		}
	}

	/**********************************************************************/
	/*****************************la pelotita******************************/
	/**********************************************************************/

	radtemp = (dmp / 2) * (dmp / 2);
	cv::Mat stop(M, N, CV_32F);

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			x1 = (-cox + 1) + i;
			xx1 = x1 * dx;
			y1 = (-coy + 1) + j;
			yy1 = y1 * dy;
			temp.at<float>(i, j) = (pow(xx1, 2) + pow(yy1, 2)) / radtemp;
			if (temp.at<float>(i, j) > 1) {
				stop.at<float>(i, j) = 1;
			}
			else {
				stop.at<float>(i, j) = 0;
			}
		}
	}

	//campo propagado por plano de la esfera
	aux = Prop_real.mul(stop);
	aux.copyTo(Prop_real);

	aux = Prop_complex.mul(stop);
	aux.copyTo(Prop_complex);

	magnitude(Prop_real, Prop_complex, auxAmp);
	phase(Prop_real, Prop_complex, auxFase);
	rangoAmp = change_range(auxAmp, 0, 255);
	rangoFase = change_range(auxFase, menosPI, PI);

	imwrite("stop.jpg", stop);
	stop.release();
	imwrite("temp.jpg", temp);
	temp.release();

	/**********************************************************************/
	/************************segunda propagación***************************/
	/**********************************************************************/

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < M; j++) {
			x1 = (-cox + 1) + i;
			xx1 = x1 * dox;
			y1 = (-coy + 1) + j;
			yy1 = y1 * doy;
			FP0.at<float>(i, j) = sqrt(1 - lambda * lambda * (pow(xx1, 2) + pow(yy1, 2)));
		}
	}

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			Ep_real.at<float>(i, j) = (float)cos(2 * PI * z2 * (FP0.at<float>(j, i) / lambda));
			Ep_complex.at<float>(i, j) = (float)sin(2 * PI * z2 * (FP0.at<float>(j, i) / lambda));
		}
	}

	imwrite("FP0.jpg", FP0);
	FP0.release();
	//f2_real = Transformada_real(Prop_real, Prop_complex);
	//f2 = fftshift(fft2(fftshift(Prop))) * dox * doy;

	resultado = Transformada(Prop_real, Prop_complex);
	split(resultado, canales);  // canales[0] = Re(DFT(I), canales[1] = Im(DFT(I))

	f2_real = canales[0];
	f2_complex = canales[1];

	magnitude(canales[0], canales[1], auxAmp);
	phase(canales[0], canales[1], auxFase);
	rangoAmp = change_range(auxAmp, 0, 255);
	rangoFase = change_range(auxFase, menosPI, PI);

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			f2_real.at<float>(i, j) *= (dox * doy);
			f2_complex.at<float>(i, j) *= (dox * doy);
		}
	}

	aux = Ep_real.mul(f2_real);
	aux.copyTo(f1_real);
	aux = Ep_complex.mul(f2_complex);
	aux.copyTo(f1_complex);

	imwrite("f2_real.jpg", f2_real);
	f2_real.release();
	imwrite("f2_complex.jpg", f2_complex);
	f2_complex.release();
	imwrite("Ep_real.jpg", Ep_real);
	Ep_real.release();
	imwrite("Ep_complex.jpg", Ep_complex);
	Ep_complex.release();
	imwrite("aux.jpg", aux);
	aux.release();
	

	//Prop=fftshift(ifft2(fftshift(f1)))*fox*foy;
	cv::Mat Prop2_real(M, N, CV_32F);
	cv::Mat Prop2_complex(M, N, CV_32F);

	//Prop2_real = Transformada_real(f1_real, f1_complex);
	resultado = Transformada_inversa(f1_real, f1_complex);
	imwrite("f1_complex.jpg", f1_complex);
	f1_complex.release();
	imwrite("f1_real.jpg", f1_real);
	f1_real.release();
	split(resultado, canales);  // canales[0] = Re(DFT(I), canales[1] = Im(DFT(I))

	Prop2_real = canales[0];
	Prop2_complex = canales[1];

	magnitude(canales[0], canales[1], auxAmp);
	phase(canales[0], canales[1], auxFase);
	rangoAmp = change_range(auxAmp, 0, 255);
	rangoFase = change_range(auxFase, 0, 255);

	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			Prop2_real.at<float>(i, j) *= (fox * foy);
			Prop2_complex.at<float>(i, j) *= (fox * foy);
		}
	}

	/*************************************/
	Mat Intres(M, N, CV_32F);
	magnitude(Prop2_complex, Prop2_real, Intres);
	Intres = Intres.mul(Intres);

	//Intres=255*Intres/mmax;
	double min, max;
	minMaxLoc(Intres, &min, &max);
	Intres = 255 * Intres / (float)max;

	return 0;
}

cv::Mat Transformada(cv::Mat real, cv::Mat complex) {
	cv::Mat tmp, transformacion_real, transformacion_complex;
	//espacio para valores reales e imaginarios
	Mat planes[] = { real, complex };
	Mat complexI;
	merge(planes, 2, complexI);

	//transformada, da valores complejos
	dft(complexI, complexI);

	//obtener la magnitud del número complejo
	split(complexI, planes);  // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))

	//magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude

	transformacion_real = planes[0];

	int cx = transformacion_real.cols / 2;
	int cy = transformacion_real.rows / 2;

	Mat q0(transformacion_real, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
	Mat q1(transformacion_real, Rect(cx, 0, cx, cy));  // Top-Right
	Mat q2(transformacion_real, Rect(0, cy, cx, cy));  // Bottom-Left
	Mat q3(transformacion_real, Rect(cx, cy, cx, cy)); // Bottom-Right

	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);
	q1.copyTo(tmp);
	q2.copyTo(q1);
	tmp.copyTo(q2);

	transformacion_complex = planes[1];

	cx = transformacion_complex.cols / 2;
	cy = transformacion_complex.rows / 2;

	Mat p0(transformacion_complex, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
	Mat p1(transformacion_complex, Rect(cx, 0, cx, cy));  // Top-Right
	Mat p2(transformacion_complex, Rect(0, cy, cx, cy));  // Bottom-Left
	Mat p3(transformacion_complex, Rect(cx, cy, cx, cy)); // Bottom-Right

	p0.copyTo(tmp);
	p3.copyTo(p0);
	tmp.copyTo(p3);
	p1.copyTo(tmp);
	p2.copyTo(p1);
	tmp.copyTo(p2);

	Mat results[] = { transformacion_real, transformacion_complex };
	Mat last;
	merge(results, 2, last);

	return last;
}

cv::Mat Transformada_inversa(cv::Mat real, cv::Mat complex) {
	cv::Mat tmp, transformacion_real, transformacion_complex;

	//espacio para valores reales e imaginarios
	Mat planes[] = { real, complex };
	Mat complexI;
	merge(planes, 2, complexI);

	//transformada, da valores complejos
	idft(complexI, complexI);

	//obtener la magnitud del número complejo
	split(complexI, planes);  // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))

	//magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude

	transformacion_real = planes[0];
	transformacion_complex = planes[1];

	int cx = transformacion_real.cols / 2;
	int cy = transformacion_real.rows / 2;

	Mat results[] = { transformacion_real, transformacion_complex };
	Mat last;
	merge(results, 2, last);

	return last;
}

Mat change_range(cv::Mat original, double newMin, double newMax) {
	double minOld, maxOld;
	minMaxLoc(original, &minOld, &maxOld);

	float num, valor;
	Mat nueva(original.rows, original.cols, CV_32F);


	float oldRange = (float)maxOld - (float)minOld;
	float newRange = (float)(newMax - newMin);

	for (int i = 0; i < nueva.rows; i++) {
		for (int j = 0; j < nueva.cols; j++) {
			num = original.at<float>(i, j);
			valor = (float)(((num - (float)minOld) * newRange) / oldRange) + (float)newMin;
			nueva.at<float>(i, j) = valor;
		}
	}

	return nueva;
}