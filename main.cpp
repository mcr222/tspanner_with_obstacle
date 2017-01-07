#include <tuple>
#include <iostream>
#include <string>
#include<fstream>
#include <vector>
#include <math.h>
#include <queue>

using namespace std;

typedef pair<int,int> coord;
typedef vector<vector<int>> graph;
struct problem {
	int n;
	int nobs;
	int a;
	int b;
	vector<coord> points;
	vector<coord> obstacle_vert;
};

class CompareDist
{
public:
    bool operator()(pair<int,int> n1,pair<int,int> n2) {
        return n1.first>n2.first;
    }
};

problem readFile(char filename[]) {
	cout << "Reading dataset: " << filename << endl;
	ifstream file(filename);
	int n, nobs,a,b;
	file >> n;
	file >> nobs;
	file >> a >> b;

	vector<coord> points(n);
	vector<coord> obstacle_vert(nobs);
	problem prbl;
	prbl.n = n;
	prbl.nobs = nobs;
	prbl.a = a;
	prbl.b = b;
	cout << "problem parameters " << prbl.n << " " << prbl.nobs << " "<< prbl.a << " " << prbl.b << endl;
	int coord1,coord2;

	if (file.is_open()) {
		for(int i=0;i<n;++i){
			file >> coord1 >> coord2;
			points[i]=coord(coord1,coord2);
		}
		for(int i=0;i<nobs;++i){
			file >> coord1 >> coord2;
			obstacle_vert[i]=coord(coord1,coord2);
		}
	}
	prbl.points = points;
	prbl.obstacle_vert = obstacle_vert;
	file.close();
	return prbl;
}

double euclidean_distance(coord p1, coord p2) {
	return sqrt(pow(double(p1.first-p2.first),2)+pow(double(p1.second-p2.second),2));
}

double dijkstra_shortest_path(int n, graph& tspanner, vector<coord>& points, int source, int target) {
	vector<double> distances(n,-1);
	vector<bool> visited(n,false);
	priority_queue<pair<int,int>,vector<pair<int,int>>,CompareDist> pq;
	//priority queue has the pair: priority,point_number
	pq.push(make_pair(0,source));
	distances[source]=0;
	pair<int,int> prior_point;
	vector<int> neighbors;
	double alt_dist;
	while(!pq.empty()){
		prior_point = pq.top();
		pq.pop();
		neighbors = tspanner[prior_point.second];
		if(!visited[prior_point.second]) {
			visited[prior_point.second] = true;
			for(int i=0;i<neighbors.size();++i) {
				alt_dist = distances[prior_point.second]+euclidean_distance(points[prior_point.second],points[neighbors[i]]);
				if(distances[neighbors[i]]==-1 || alt_dist<distances[neighbors[i]]) {
					distances[neighbors[i]] = alt_dist;
					pq.push(make_pair(alt_dist,neighbors[i]));
				}
			}
		}
	}
	/*for(int i=0;i<distances.size();++i) {
		cout<<distances[i]<<endl;
	}*/
	//TODO: this can be returned before computing all distances
	return distances[target];
}

void insert_edge(graph& tspanner, int p1, int p2) {
	tspanner[p1].push_back(p2);
	tspanner[p2].push_back(p1);
}

int orientation_3_points(coord p1,coord p2, coord p3) {
    int val = (p2.second - p1.second) * (p3.first - p2.first) -
              (p2.first - p1.first) * (p3.second - p2.second);

    if (val == 0) return 0;  // colinear points

    return (val > 0)? 1: 2; // clock or counterclock wise order
}

bool edges_intersect(coord p1, coord p2, coord v1, coord v2) {
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation_3_points(p1, p2, v1);
	int o2 = orientation_3_points(p1, p2, v2);
	int o3 = orientation_3_points(v1, v2, p1);
	int o4 = orientation_3_points(v1, v2, p2);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	//TODO: this assumes general position of points

	return false;
}

bool edge_intersects_obstacle(int p1, int p2, vector<coord>& points, vector<coord>& obstacle_vert){
	int n = obstacle_vert.size();
	if(edges_intersect(points[p1],points[p2],obstacle_vert[n-1],obstacle_vert[0])) {
		return true;
	}
	for(int i=1;i<n;++i) {
		if(edges_intersect(points[p1],points[p2],obstacle_vert[i-1],obstacle_vert[i])) {
			return true;
		}
	}
	return false;
}

int main() {
	char filename[] = "Data_examples/geometricspanners.txt";
	problem pr = readFile(filename);
	graph tspanner(pr.n, vector<int>(0));

//	Example of dijkstra distance
	vector<coord> points = {make_pair(0,0),make_pair(0,1),make_pair(1,1),make_pair(1,2)};
//    insert_edge(tspanner, 0, 1);
//    insert_edge(tspanner,0, 2);
//    insert_edge(tspanner,1,3);
//    insert_edge(tspanner,1,2);

	//TODO: Missing iterating over all pair of points (edges) and sorting them by distance
	//If an edge intersects any other edge of the obstacle then do visibility graph and compute
	//distance using dijsktra in the visibility graph between the two points (including the two points
	//and the obstacle vertices, adding only the edges that do not cross the obstacle).
	//Must add in dijsktra the returning of this set of edges that make the shortest path
	//Do greedy algorithm with this sorted list. Adding edges or the shortest paths in the visibility graph
	//between the two points

	double shortest_dist = dijkstra_shortest_path(pr.n,tspanner,pr.points,1,2);

}
