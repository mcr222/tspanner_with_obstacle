//
//  DataGene.cpp
//  spanners
//
//  Created by Xuqiang Fang on 09/01/2017.
//  Copyright © 2017 Xuqiang Fang. All rights reserved.


#include <iostream>
#include <cstdlib>
#include <math.h>
#include <ctime>
#include <algorithm>
#include <vector>


#define PI 3.1415926

using namespace std;



int N = 300; // size of the bounding box
int width = 100;// size of the bouding box for obstacle
int n = 0; // the number of vertices
int n_obs = 0;// the number of vertices for the obstacle
float temp = 0;
bool Inside = 0;


float calcu_angle(int x1, int y1, int x_center, int y_center){
    long double x = (x1 - x_center)*1.00000;
    long double y = (y1 - y_center)*1.00000;
    //float x_unit = 100.000000*x / (x^2 + y^2);// multiply to keep the precision
    //float y_unit = 100*y / (x^2 + y^2);
    float angle = 0;
    angle = atan2(y,x)*180/PI;
    
    if (y >= 0){
        angle = angle;
    }
    else if (y < 0) {
        angle = 360 + angle;
    }
    return angle;
}
// detect intersection of an obstacle edge and a horizontal line
//bool intersect(int x1, int y1, int x2, int y2, int x, int y) {
//    return 1;
//}

int main(int argc, const char * argv[]) {
    
    srand((unsigned)time(0));
    n = 30 + rand() % (N - 1);
    
    
    //generate random points, the first point (x_coord[0],y_coord[0]) is for obstacle center
    int x_coord [n];
    int y_coord [n];
    for (int j = 1; j < n; j++){
        x_coord[j] = (rand() % (N - 1)) + 1;
        y_coord[j] = (rand() % (N - 1)) + 1;
    }
    
    //generate obstacle center
    //bug detected
    x_coord[0] = width + rand() % (N - 2*width);
    y_coord[0] = width + rand() % (N - 2*width);
    
    //generate the obstacle vertices number
    n_obs = 3 + rand() % (width/5);
    
    //generate the obstacle points
    int obs_x_coord[n_obs];
    int obs_y_coord[n_obs];
    for (int i = 0; i < n_obs; i++) {
        obs_x_coord[i] = x_coord[0] - width + rand() % (2 * width);
        obs_y_coord[i] = y_coord[0] - width + rand() % (2 * width);
    }
    
    //build the obstacle
    float angle_array [n_obs];
    float temp_array [n_obs];
    for(int i = 0; i < n_obs; i++) {
        //cout << "(" << obs_x_coord[i] << "," << obs_y_coord[i] << ")" << "\n";
        angle_array[i] = calcu_angle(obs_x_coord[i], obs_y_coord[i], x_coord[0], y_coord[0]);
        temp_array[i] = angle_array[i];
        //cout << angle_array[i];
        //cout << "\n";
        
    }
    // sort the array
    /*for (int i = 0; i < n_obs; i++){
        for (int j =0; j < n_obs-1; j++){
            if (angle_array[j] > angle_array[j+1]) {
                temp = angle_array[j];
                angle_array[j] = angle_array[j+1];
                angle_array[j+1] = temp;
            }
        }
    }*/
    sort(angle_array,angle_array + n_obs);
    
    //find the index
    int index_array [n_obs];
    for (int i = 0; i < n_obs; i++) {
        for (int j = 0; j < n_obs; j++) {
            if (angle_array[i] == temp_array[j]) {
                index_array[i] = j;
            }
        }
    }
    
    //add starting point to the end, to make a polygon
    // polygon coordinates in order
    int obs_x_polygon[n_obs+1];
    int obs_y_polygon[n_obs+1];
    for (int i=0; i<n_obs;i++) {
        obs_x_polygon[i] = obs_x_coord[index_array[i]];
        obs_y_polygon[i] = obs_y_coord[index_array[i]];
    }
    obs_x_polygon[n_obs] = obs_x_polygon[0];
    obs_y_polygon[n_obs] = obs_y_polygon[0];
    
    /*
    //detect repeated points and retain only one.
    for(int i = 0; i < n_obs; i++) {
        //cout << angle_array[i] << " ";
        //cout << index_array[i] << "\n";
        cout << obs_x_coord[index_array[i]] << " " << obs_y_coord[index_array[i]] << "\n";
    }
    cout << obs_x_coord[index_array[0]] << " " << obs_y_coord[index_array[0]] << "\n";
    cout << "\n";
     
    for (int i = 0; i <= n_obs; i++) {
        cout << obs_x_polygon[i] << " " << obs_y_polygon[i] << "\n";
    }
     
    //remove all points within the square box that contains the obstacle
    for (int i = 1; i < n; i++) {
        if ((x_coord[i] >= x_coord[0]-width && x_coord[i] <= x_coord[0]+width)&&(y_coord[i] >= y_coord[0]-width && y_coord[i]<= y_coord[0]+width)) {
            continue;
        }
        cout << x_coord[i] << " " << y_coord[i] << "\n";
    }
     
     
    */
    
    //or we could be more specific, we remove all points within the obstacle
    // We have a polygon, and we have a query point, determine how many edges of the polygon intersect with the ray starting from query point upward.
    //segment tree with range tree
    for (int i = 0; i <= n_obs; i++) {
        cout << obs_x_polygon[i] << " " << obs_y_polygon[i] << "\n";
    }
    
    //raycasting algorithm
    int constant[n_obs];
    float multiple[n_obs];
    int relay = n_obs;
    int Inside[n];
    for (int i=0;i<n;i++) {
        Inside[i]=0;
    }
    //calculte for each edge, the line equation
    for (int i=0; i<n_obs;i++) {
        if(obs_y_polygon[relay] == obs_y_polygon[i]) {
            constant[i] = obs_x_polygon[i];
            multiple[i] = 0;
        }
        else {
            constant[i] = obs_x_polygon[i]-(obs_y_polygon[i]*obs_x_polygon[relay])/(obs_y_polygon[relay]-obs_y_polygon[i])+
            (obs_y_polygon[i]*obs_x_polygon[i])/(obs_y_polygon[relay]-obs_y_polygon[i]);
            multiple[i] = (float)(obs_x_polygon[relay]-obs_x_polygon[i])/(obs_y_polygon[relay]-obs_y_polygon[i]);
            relay = i;
        }
        //cout << multiple[i] << "\n";
    }
    //calculate the x-coordinate and compare with the point
    for (int temp=0; temp<n; temp++) {
        //cout << x_coord[temp] << " " << y_coord[temp] << "   ";
        int   j = n_obs;
        for (int i=0; i<n_obs; i++) {
            if (((obs_y_polygon[i]< y_coord[temp] && obs_y_polygon[j]>=y_coord[temp]) ||
                 (obs_y_polygon[j]< y_coord[temp] && obs_y_polygon[i]>=y_coord[temp]))) {
                Inside[temp]^=(y_coord[temp]*multiple[i]+constant[i]<(float)(1.00*x_coord[temp]));
            }
            j=i;
        }
        //cout << Inside[temp] << "\n";
    }
    /*
    bool pointInPolygon(int x, int y) {
        int   i, j=n_obs-1 ;
        for (i=0; i<n_obs; i++) {
            if ((obs_y_coord[i]< y && obs_y_coord[j]>=y   ||  obs_y_coord[j]< y && obs_y_coord[i]>=y)) {
                Inside^=(y*multiple[i]+constant[i]<x);
            }
            j=i;
        }
        return Inside;
    }
     */
    
    for (int i=0;i<n;i++) {
        if (Inside[i] == 0) {
            cout << x_coord[i] << " " << y_coord[i] << "\n";
        }
    }
    //cout << n_obs << " " << x_coord[0] << " " << y_coord[0] <<";" << "\n";
    return 0;
}
