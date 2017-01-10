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


#define PI 3.1415926

using namespace std;



int N = 300; // size of the bounding box
int width = 100;// size of the bouding box for obstacle
int n = 0; // the number of vertices
int n_obs = 0;// the number of vertices for the obstacle
float temp = 0;


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
bool intersect(int x1, int y1, int x2, int y2, int x, int y) {
    return 1;
}

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
    
    //detect repeated points and retain only one.
    for(int i = 0; i < n_obs; i++) {
        //cout << angle_array[i] << " ";
        //cout << index_array[i] << "\n";
        cout << obs_x_coord[index_array[i]] << " " << obs_y_coord[index_array[i]] << "\n";
    }
    
    cout << "\n";
    //remove all points within the square box that contains the obstacle
    for (int i = 1; i < n; i++) {
        if ((x_coord[i] >= x_coord[0]-width && x_coord[i] <= x_coord[0]+width)&&(y_coord[i] >= y_coord[0]-width && y_coord[i]<= y_coord[0]+width)) {
            continue;
        }
        cout << x_coord[i] << " " << y_coord[i] << "\n";
    }
    //or we could be more specific, we remove all points within the obstacle
    //for (int i = 1; i < n; i++) {
   //     if(intersect(<#int x1#>, <#int y1#>, <#int x2#>, <#int y2#>, <#int x#>, <#int y#>))
    //}

    cout << n_obs << " " << x_coord[0] << " " << y_coord[0] <<";" << "\n";
    return 0;
}
