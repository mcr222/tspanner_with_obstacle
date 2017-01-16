//
//  data.cpp
//  data
//
//  Created by Xuqiang Fang on 15/01/2017.
//  Copyright © 2017 Xuqiang Fang. All rights reserved.
//


#include "data.h"


#define PI 3.1415926

using namespace std;


long double calcu_angle(int x1, int y1, int x_center, int y_center){
    long double x = (x1 - x_center)*1.00000;
    long double y = (y1 - y_center)*1.00000;
    long double angle = 0;
    angle = atan2(y,x)*180.00000/PI;
    if (y < 0) {
        angle = 360.0 + angle;
    }
    else if (y>=0) {
        angle = 0.0 + angle;
    }
    return angle;
}


int findIndex(long double *arr, long double x, int sizeofarray) {
    int index = 0;
    int _size = sizeofarray;
    for (int i=0; i<_size;i++) {
        if (x>arr[i] && x<arr[i+1]) {
            index = i;
            break;
        }
    }
    if (x<arr[0] || x>arr[_size-1]){
        index = _size-1;
    }
    return index;
}
/*
long double calcu_intersect_x (int x1, int y1, int x2, int y2 ) {
    
}
*/
long double calcu_distance(long double x1, long double y1, long double x2, long double y2) {
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))*1.0000;
}



//int main() {
    //int N_par = 300;
    //int n_par = 300;
    
    
    
void datagene (int N_par, int n_par) {
    int N = 4*N_par; // size of the bounding box
    int n = N_par;
    int n_obs = n_par;// the number of vertices for the obstacles
    int count = 0;
    int origin_x = 0;
    int origin_y = 0;
    bool is_inside = true;

    int x_coord[n];
    int y_coord[n];
    int obs_x_coord[n_obs];
    int obs_y_coord[n_obs];
    
    
    //generate the obstacle vertices number and random points number
    srand((unsigned)time(0));
    //n = 30 + rand() % (N - 1);
    //n_obs = 3 + rand() % (/5);
    
    //generate obstalce vertices points, with obstacle origin is not smallest or biggest
    //while(!(feasible_small_x*feasible_big_x*feasible_big_y*feasible_small_y)) {
        
        //x_coord[0] = n + rand() % (2*n);
        //y_coord[0] = n + rand() % (2*n);
        
        for (int i = 0; i < n_obs; i++) {
            obs_x_coord[i] = 1.5*n + rand() % (2*n);
            obs_y_coord[i] = 1.5*n + rand() % (2*n);
        }
    origin_x = (obs_x_coord[0] + obs_x_coord[n_obs-1])/2;
    origin_y = (obs_y_coord[0] + obs_y_coord[n_obs-1])/2;

    
   // }//end while
    
    
    //build the obstacle
    long double angle_array[n_obs];
    long double temp_array[n_obs];
    for(int i = 0; i < n_obs; i++) {
        //cout << "(" << obs_x_coord[i] << "," << obs_y_coord[i] << ")" << "\n";
        angle_array[i] = calcu_angle(obs_x_coord[i], obs_y_coord[i], origin_x, origin_y);
        temp_array[i] = angle_array[i];
        //cout << angle_array[i] << "\n";
        //cout << x_coord[0] << " " << y_coord[0]<<"\n";
        
    }
    
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
    for (int i=0; i<n_obs; i++) {
        //cout << angle_array[i]<<"\n";
        /* index_array[i] <<"\n";*/
    }
    cout << origin_x << " " << origin_y << endl;
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
    
    //remove duplicates for obstacle vertices
    int obs_x_polygon_temp[n_obs+1];
    int obs_y_polygon_temp[n_obs+1];
    for (int i=0; i<=n_obs; i++) {
        obs_x_polygon_temp[i] = obs_x_polygon[i];
        obs_y_polygon_temp[i] = obs_y_polygon[i];
    }
    for (int i=0; i<n_obs; i++) {
        for (int j=i+1; j<=n_obs; j++) {
            if((obs_x_polygon[i] == obs_x_polygon_temp[j]) &&(obs_y_polygon[i] == obs_y_polygon_temp[j])) {
                obs_x_polygon_temp[j] = -1;
                obs_y_polygon_temp[j] = -1;
            }
        }
    }
    count = 0;
    for (int i=0; i<n_obs; i++) {
        if (obs_x_polygon_temp[i] > 0) {
            obs_x_polygon[count] = obs_x_polygon_temp[i];
            obs_y_polygon[count] = obs_y_polygon_temp[i];
            count += 1;
        }
    }
    obs_x_polygon[count] = obs_x_polygon[0];
    obs_y_polygon[count] = obs_y_polygon[0];
    for (int i = 0; i <= count; i++) {
        cout << obs_x_polygon[i] << " " << obs_y_polygon[i] << "\n";
    }
    
    n_obs = count;
    long double distance[n_obs];
    for (int i=0; i<n_obs ;i++) {
        distance[i] = calcu_distance(origin_x, origin_y, obs_x_polygon[i], obs_y_polygon[i]);
    }
    
    count = 0;
    while (count < n) {
        is_inside = true;
        int j = count;
        int index = 0;
        long double distance_coord = 0;
        long double value = 0;
        x_coord[j] = (rand() % (N - 1)) + 1;
        y_coord[j] = (rand() % (N - 1)) + 1;
        cout << x_coord[j] <<  " " << y_coord[j] << endl;
        distance_coord = calcu_distance(origin_x, origin_y, x_coord[j], y_coord[j]);
        value = calcu_angle(x_coord[j], y_coord[j], origin_x, origin_y);
        cout << value <<endl;;
        index = findIndex(angle_array, value,n_obs);
        cout << index <<endl;
        
        if ((index == (n_obs-1))&&(distance_coord>max(distance[0],distance[n_obs-1]))) {
            is_inside = false;
            cout << x_coord[j] <<  " " << y_coord[j] << endl;
            count += 1;

        }
        else if((index !=(n_obs-1))&&(distance_coord>max(distance[index],distance[index+1]))) {
            is_inside = false;
            cout << x_coord[j] <<  " " << y_coord[j] << endl;
            count += 1;

        }
        
    }
    
    //write text file
    ofstream myfile("data.txt");
    //myfile.open("data.txt");
    myfile << n <<"\n";
    myfile << n_obs <<"\n";
    myfile << 3 << " " << 2 <<"\n";
    for (int i=0;i<n;i++) {
        myfile << x_coord[i] << " " << y_coord[i] <<"\n";
    }
    for (int i=n_obs-1; i>=0;i--){
        myfile << obs_x_polygon[i] << " " <<obs_y_polygon[i] <<"\n";
    }
    
    myfile.close();
    cout << "finished" <<endl;
    
}

    
    
    /*
    
    //cout << x_coord[0] <<" "<<y_coord[0] << "\n";
    //calculat the equation for edge, only count number of edges
    
    int constant[n_obs];
    long double multiple[n_obs];
    int relay = n_obs;
    for (int i=0; i<n_obs; i++) {
        if(obs_y_polygon[relay] == obs_y_polygon[i]) {
            constant[i] = obs_x_polygon[i];
            multiple[i] = 0;
        }
        else {
            constant[i] = obs_x_polygon[i]-(obs_y_polygon[i]*obs_x_polygon[relay])/(obs_y_polygon[relay]-obs_y_polygon[i])+
            (obs_y_polygon[i]*obs_x_polygon[i])/(obs_y_polygon[relay]-obs_y_polygon[i]);
            //cout << "constant before" << constant[i] <<endl;
            multiple[i] = (long double)(obs_x_polygon[relay]-obs_x_polygon[i])/(obs_y_polygon[relay]-obs_y_polygon[i]);
            constant[i] = (long double)(obs_x_polygon[i]-multiple[i]*obs_y_polygon[i]);
            //cout << "constant after" << constant[i] << endl;
            relay = i;
        }
    }
    
    //generate a point and test if it is in the obstacle
    count = 0;
    //int Inside[n];
    //for (int i=0;i<n;i++) {
      //  Inside[i]=1;
    //}
    int j = 0;
    //long double before = 0;
    while(count <= n) {
        
        int index = 0;
        long double value = 0;
        long double temp_coord = 0;
        long double distance_a = 0;
        long double distance_b = 0;
        
        bool inside = false;
        //bool double_inside = false;
        j = count;
        x_coord[j] = (rand() % (N - 1)) + 1;
        y_coord[j] = (rand() % (N - 1)) + 1;
        value = calcu_angle(x_coord[j], y_coord[j], origin_x, origin_y);
        cout << value <<endl;;
        index = findIndex(angle_array, value,n_obs);
        cout << index <<endl;
        
        
     
        if (origin_y == y_coord[index]) {
            temp_coord = (long double)1.00000*(y_coord[j]*multiple[index]+constant[index]);
            distance_a = sqrtf((temp_coord-origin_x)*(temp_coord-origin_x));
        }
        else {
                   }
     
        distance_b = calcu_distance(origin_x, origin_y, x_coord[j], y_coord[j]);
        
        int temp = n_obs-1;
        for (int i=0; i<n_obs; i++) {
            if (((obs_y_polygon[i]< y_coord[j] && obs_y_polygon[temp]>=y_coord[j]) ||
                 (obs_y_polygon[temp]< y_coord[j] && obs_y_polygon[i]>=y_coord[j]))) {
                
                inside^=(long double)1.00000*(y_coord[j]*multiple[i]+constant[i])<(long double)(1.00*x_coord[j]);
     
                for (int j =0; j< n_obs; j++){
                    if ((!inside) && (y_coord[i]==obs_y_polygon[j])) {
                        inside ^= inside;
                    }
                }
     
            }
            temp = i;
        }
        temp = n_obs-1;
        for (int i=0; i<n_obs; i++) {
            if (((obs_y_polygon[i]< y_coord[j] && obs_y_polygon[temp]>=y_coord[j]) ||
                 (obs_y_polygon[temp]< y_coord[j] && obs_y_polygon[i]>=y_coord[j]))) {
                
                double_inside^=(long double)1.00000*(y_coord[j]*multiple[i]+constant[i])<(long double)(1.00*x_coord[j]);
     
                 for (int j =0; j< n_obs; j++){
                 if ((!inside) && (y_coord[i]==obs_y_polygon[j])) {
                 inside ^= inside;
                 }
                 }
     
            }
            temp = i;
        }

        
        if ((!inside)&&(!double_inside )) {
            //cout << x_coord[j] << " " << y_coord[j] <<"\n";
            count += 1;
        }
    }
    
    
    //cout << count << "\n";
    

     
     int Inside[n];
     for (int i=0;i<n;i++) {
     Inside[i]=0;
     }
     for (int j = 0; j < n; j++){
     x_coord[j] = (rand() % (N - 1)) + 1;
     y_coord[j] = (rand() % (N - 1)) + 1;
     }
     //remove all duplicates for data points
     int x_coord_temp[n];
     int y_coord_temp[n];
     for (int i=0; i<n; i++) {
     x_coord_temp[i] = x_coord[i];
     y_coord_temp[i] = y_coord[i];
     }
     for (int i=1; i<n; i++) {
     for (int j=i+1; j<n;j++) {
     if(x_coord[i]==x_coord_temp[j] && y_coord[i]==y_coord_temp[j]) {
     x_coord_temp[j] = N+1;
     y_coord_temp[j] = N+1;
     }
     }
     }
     count = 1;
     for (int i=1; i<n; i++) {
     if ((x_coord_temp[i] <= N) && (y_coord_temp[i] <= N)) {
     x_coord[count] = x_coord_temp[i];
     y_coord[count] = y_coord_temp[i];
     count += 1;
     }
     }
     //cout << "number of duplicates" << n-count << "\n";
     n = count;//non-unique points
     
     
     
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
     

    count = 1;
    for (int i=0;i<n;i++) {
        if (Inside[i] == 0) {
            count += 1;
            //cout << x_coord[i] << " " << y_coord[i] << "\n";
        }
    }
    for (int i = 0; i <= n_obs; i++) {
        //cout << obs_x_polygon[i] << " " << obs_y_polygon[i] << "\n";
    }
    

    
    

    //return 0;
}
*/


