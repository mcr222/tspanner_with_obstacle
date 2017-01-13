#include <tuple>
#include <iostream>
#include <string>
#include<fstream>
#include <vector>
#include <math.h>
#include <queue>
#include <algorithm>
#include <set>
#include <limits>
#include <time.h>

using namespace std;

typedef pair<double,double> coord;
typedef vector<set<int>> graph;
struct problem {
	int n;
	int nobs;
	double t;
	vector<coord> points;
	vector<coord> obstacle_vert;
};

problem pr;

struct edge {
	int p1;
	int p2;
	double dist;
	//integer when over number of points then is an obstacle_vert
	vector<int> shortest_path;
	bool straight_line;

	bool operator < (const edge& edg) const {
		return(dist<edg.dist);
	}

	//tested
	bool operator == (const edge& edg) const {
			return((edg.p1==p1 && edg.p2==p2)||
					(edg.p1==p2 && edg.p2==p1));
	}
};

struct event {
	int p;
	double angle;
	bool starting;
	edge segment;

	bool operator < (const event& evt) const {
			return(angle<evt.angle);
	}
};

class CompareDist
{
public:
    bool operator()(pair<int,int> n1,pair<int,int> n2) {
        return n1.first>n2.first;
    }
};

//tested
void readFile(char filename[]) {
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
	prbl.t = a/(double)b;
	cout << "problem parameters " << prbl.n << " " << prbl.nobs << " "<< prbl.t << endl;
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
	pr = prbl;
}

double euclidean_distance(coord p1, coord p2) {
	return sqrt(pow(double(p1.first-p2.first),2)+pow(double(p1.second-p2.second),2));
}

coord getpoint(int i) {
	if(i<pr.n) {
		return pr.points[i];
	} else {
		return pr.obstacle_vert[i-pr.n];
	}
}

//tested
double dijkstra_shortest_path(graph& tspanner, int source, int target, vector<int>& path) {
	vector<double> distances(pr.n+pr.nobs,-1);
	vector<int> previous(pr.n+pr.nobs,-1);
	vector<bool> visited(pr.n+pr.nobs,false);
	priority_queue<pair<int,int>,vector<pair<int,int>>,CompareDist> pq;
	//priority queue has the pair: priority,point_number
	pq.push(make_pair(0,source));
	distances[source]=0;
	pair<int,int> prior_point;
	set<int> neighbors;
	double alt_dist;
	while(!pq.empty()){
		prior_point = pq.top();
		pq.pop();
		if(prior_point.second == target) {
			int curr=target;
			while(curr!=source) {
				path.push_back(curr);
				curr=previous[curr];
			}
			path.push_back(curr);
			return distances[target];
		}
		neighbors = tspanner[prior_point.second];
		if(!visited[prior_point.second]) {
			visited[prior_point.second] = true;
			for(set<int>::iterator it=neighbors.begin();it!=neighbors.end();++it) {
				alt_dist = distances[prior_point.second]+euclidean_distance(getpoint(prior_point.second),getpoint(*it));
				if(distances[*it]==-1 || alt_dist<distances[*it]) {
					distances[*it] = alt_dist;
					previous[*it] = prior_point.second;
					pq.push(make_pair(alt_dist,*it));
				}
			}
		}
	}

	return distances[target];
}

//tested
void insert_edge(graph& tspanner, int p1, int p2) {
	tspanner[p1].insert(p2);
	tspanner[p2].insert(p1);
}

int orientation_3_points(coord p1,coord p2, coord p3) {
    int val = (p2.second - p1.second) * (p3.first - p2.first) -
              (p2.first - p1.first) * (p3.second - p2.second);

    if (val == 0) return 0;  // colinear points

    return (val > 0)? 1: 2; // clock or counterclock wise order
}

//tested
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

	//This assumes general position of points

	return false;
}

bool edge_intersects_obstacle(int p1, int p2){
	if(edges_intersect(getpoint(p1),getpoint(p2),getpoint(pr.n+pr.nobs-1),getpoint(pr.n))) {
		return true;
	}
	for(int i=1;i<pr.nobs;++i) {
		if(edges_intersect(getpoint(p1),getpoint(p2),getpoint(pr.n+i-1),getpoint(pr.n+i))) {
			return true;
		}
	}
	return false;
}

bool parallel_vectors(coord v1, coord v2) {
	return (v1.first*v2.first+v1.second*v2.second == sqrt(v1.first*v1.first+v1.second*v1.second)*
			sqrt(v2.first*v2.first+v2.second*v2.second));
}
//tested
double compute_angle(coord p1, coord p, coord p2) {
	coord v1 = make_pair(p2.first-p.first,p2.second-p.second);
	coord v2 = make_pair(p1.first-p.first,p1.second-p.second);
//	if(parallel_vectors(v1,v2)) {
//		return 0;
//	}
	int dot = v1.first*v2.first + v1.second*v2.second;
	int det = v1.first*v2.second - v1.second*v2.first;
	return atan2(-det, -dot)+M_PI;
}

coord infinity_point(0,100000);
void insert_segment_events(vector<event>& events,int i, int p, int obst1, int obst2) {
	event evt1;
	event evt2;
	edge segment;
	segment.p1 = obst1;
	segment.p2 = obst2;
	segment.straight_line=true;
	evt1.segment = segment;
	evt2.segment = segment;
	evt1.p = obst1;
	evt2.p = obst2;
	evt1.angle = compute_angle(infinity_point,getpoint(p),getpoint(obst1));
	evt2.angle = compute_angle(infinity_point,getpoint(p),getpoint(obst2));
	if(edges_intersect(getpoint(p),infinity_point,getpoint(obst1),getpoint(obst2))) {
		if(evt1.angle<evt2.angle) {
			evt1.starting=false;
			evt2.starting=true;
		} else {
			evt1.starting=true;
			evt2.starting=false;
		}
	}
	else {
		if(evt1.angle<evt2.angle) {
			evt1.starting=true;
			evt2.starting=false;
		} else{
			evt1.starting=false;
			evt2.starting=true;
		}
	}
	events[i] = evt1;
	events[i+pr.nobs] = evt2;
}

//tested
coord find_intersection_lines(coord p1, coord p2, coord v1, coord v2) {
	//RETURNS -nan if lines are parallel
	double n1,d,n2;
	n1=((p1.first*p2.second-p1.second*p2.first)*(v1.first-v2.first)-(p1.first-p2.first)*(v1.first*v2.second-v1.second*v2.first));
	n2=((p1.first*p2.second-p1.second*p2.first)*(v1.second-v2.second)-(p1.second-p2.second)*(v1.first*v2.second-v1.second*v2.first));
	d=((p1.first-p2.first)*(v1.second-v2.second)-(p1.second-p2.second)*(v1.first-v2.first));
	return make_pair(n1/d,n2/d);
}

//tested
bool point_in_segment(coord inters, coord p1, coord p2) {
	coord v = make_pair(p1.first-p2.first,p1.second-p2.second);
	double t;
	if(v.first!=0) {
		t=(inters.first-p2.first)/v.first;
	} else {
		t=(inters.second-p2.second)/v.second;
	}
	//cout << "t: " << t << endl;
	return (0<=t && t<=1);
}

struct status_segment {
	int p1;
	int p2;
	int p;

	//tested
	//OBS: if the touch (share one point p1 or p2), then they will be ordered randomly (but deterministically)
	//but because the shared point will be processed twice, then it will be at the bottom
	//for one of the segments processed
	bool operator < (const status_segment seg) const {
		//as they are status segments, they will overlap from the points p view
		//coord intersection = find_intersection(getpoint(p),infinity_point,getpoint(p1),getpoint(p2));

		if((seg.p1==p1 && seg.p2==p2)||
				(seg.p1==p2 && seg.p2==p1)) {
			//cout << "first false" << endl;
			return false;
		}

		coord inters;
		if(seg.p1!= p1 && seg.p1!=p2) {
			if(edges_intersect(getpoint(p),getpoint(seg.p1),getpoint(p1),getpoint(p2))) {
				//cout << "1st true" << endl;
				return true;
			}
			else {
				//cout << "inters1" << endl;
				inters = find_intersection_lines(getpoint(p),getpoint(seg.p1),getpoint(p1),getpoint(p2));
				if(point_in_segment(inters,getpoint(p1),getpoint(p2))) {
					return false;
				}
			}
		}
		if(seg.p2!= p1 && seg.p2!=p2) {
			if(edges_intersect(getpoint(p),getpoint(seg.p2),getpoint(p1),getpoint(p2))) {
				//cout << "2nd true" << endl;
				return true;
			}
			else {
				//cout << "inters2" << endl;
				inters = find_intersection_lines(getpoint(p),getpoint(seg.p2),getpoint(p1),getpoint(p2));
				if(point_in_segment(inters,getpoint(p1),getpoint(p2))) {
					return false;
				}
			}

		}
		if(p1!= seg.p1 && p1!=seg.p2) {
			if(edges_intersect(getpoint(p),getpoint(p1),getpoint(seg.p1),getpoint(seg.p2))) {
				//cout << "1st false" << endl;
				return false;
			}
			else {
				//cout << "inters3" << endl;
				inters = find_intersection_lines(getpoint(p),getpoint(p1),getpoint(seg.p1),getpoint(seg.p2));
				if(point_in_segment(inters,getpoint(seg.p1),getpoint(seg.p2))) {
					return true;
				}
			}

		}
		if(p2!= seg.p1 && p2!=seg.p2) {
			if(edges_intersect(getpoint(p),getpoint(p2),getpoint(seg.p1),getpoint(seg.p2))) {
				//cout << "2nd false" << endl;
				return false;
			}
			else {
				//cout << "inters4" << endl;
				inters = find_intersection_lines(getpoint(p),getpoint(p2),getpoint(seg.p1),getpoint(seg.p2));
				//cout << inters.first << " " << inters.second << endl;
				if(point_in_segment(inters,getpoint(seg.p1),getpoint(seg.p2))) {
					//cout << "inters4 true" << endl;
					return true;
				}
			}
		}


		//this is reached when segments share an edge and are ordered sideways
		//thus they do not have an order from the p point of view but must be
		//ordered anyway to be able to search in the set (binary search tree).
		//cout << p1 << " " << p2 << " " << seg.p1 << " " << seg.p2 << endl;
		string this_seg = to_string(p1)+to_string(p2);
		string other_seg = to_string(seg.p1)+to_string(seg.p2);
		if(this_seg<other_seg) {
			//cout << "last true" << endl;
			return true;
		} else {
			//cout << "last false" << endl;
			return false;
		}
	}

};

void initiate_status_segment(set<status_segment>& status, int p, int obs1, int obs2) {
	if(edges_intersect(getpoint(p),infinity_point,getpoint(obs1),getpoint(obs2))) {
		status_segment edg;
		edg.p1 = obs1;
		edg.p2 = obs2;
		edg.p = p;
		status.insert(edg);
	}
}

void initiate_status(set<status_segment>& status, int p){
	initiate_status_segment(status, p, pr.n+pr.nobs-1, pr.n);
	for(int i=1;i<pr.nobs;++i) {
		initiate_status_segment(status, p, pr.n+i-1, pr.n+i);
	}
}

void print_vector(vector<event> vec) {
	for(int i=0;i<vec.size();++i) {
		cout << " p "<<vec[i].p<< " angle: " <<vec[i].angle<< " p1: " <<vec[i].segment.p1<< " p2: " <<vec[i].segment.p2
				<< " start: " <<vec[i].starting;
		cout << endl;
	}
	cout << endl;
}

void visibility_point(graph& visibility, int p) {

	//TODO: this isn't to careful with parallel lines
	vector<event> events(2*pr.nobs);
	insert_segment_events(events,0, p, pr.n+pr.nobs-1, pr.n);
	for(int i=1;i<pr.nobs;++i) {
		insert_segment_events(events,i, p, pr.n+i-1, pr.n+i);
	}
	sort(events.begin(),events.end(),less<event>());
	//print_vector(events);

	set<status_segment> status;
	initiate_status(status, p);
	/*cout << "initial status: " << endl;
	for(set<status_segment>::iterator it = status.begin();it!=status.end();++it) {
		cout <<(*it).p1 << " " << (*it).p2 << endl;
	}*/

	event evt;
	status_segment seg, closest_seg;
	edge closest_edge;
	set<int> visible_points;
	seg.p = p;
	for(int i=0;i<events.size();++i) {
		evt = events[i];
		seg.p1 = evt.segment.p1;
		seg.p2 = evt.segment.p2;
		//cout << "status "<< evt.p << endl;
		if(evt.starting) {
			//cout << "inserting: " << seg.p1 << seg.p2 << endl;
			status.insert(seg);
		} else {
			//cout << "erasing: " << seg.p1 << seg.p2 << endl;
			status.erase(seg);
		}

		/*for(set<status_segment>::iterator it = status.begin();it!=status.end();++it) {
			cout <<(*it).p1 << " " << (*it).p2 << endl;
		}*/

		closest_seg = *(status.begin());
		closest_edge.p1 = closest_seg.p1;
		closest_edge.p2 = closest_seg.p2;

		if(status.size()==0 || evt.segment == closest_edge) {
			//cout << "visible: " << evt.p << endl;
			visible_points.insert(evt.p);
		}
	}

	for(set<int>::iterator it=visible_points.begin();it!=visible_points.end();++it) {
		insert_edge(visibility, p, *(it));
	}

}

void insert_obstacle_edges(graph& visibility) {
	insert_edge(visibility,pr.n+pr.nobs-1,pr.n);
	for(int i=1;i<pr.nobs;++i) {
		insert_edge(visibility,pr.n+i-1,pr.n+i);
	}
}

graph visibility_graph(int p1, int p2) {
	graph visibility(pr.n+pr.nobs, set<int>());
	insert_obstacle_edges(visibility);
	visibility_point(visibility,p1);
	visibility_point(visibility,p2);
	return visibility;
}

void print_vector(vector<int> vec) {
	for(int i=0;i<vec.size();++i) {
		cout<<vec[i] << " ";
	}
	cout << endl;
}

vector<edge> find_all_edges() {
	vector<edge> all_edges;
	edge edg;
	graph vis_gr;
	double dist;
	for(int i=0;i<pr.n;++i){
		for(int j=i+1;j<pr.n;++j) {
			edg.p1 = i;
			edg.p2 = j;
			if(edge_intersects_obstacle(i, j)) {
				//cout << edg.p1 << " " << edg.p2 << endl;
				edg.straight_line = false;
				vis_gr = visibility_graph(i,j);
				/*for(int i = 0;i<vis_gr.size();++i) {
						cout << "row: " << i << endl;
						print_vector(vis_gr[i]);
				}*/
				vector<int> path;
				dist = dijkstra_shortest_path(vis_gr,i,j,path);
				//print_vector(path);
				//cout << "distance: " << dist << endl;
				edg.dist = dist;
				edg.shortest_path = path;
			}
			else {
				edg.dist = euclidean_distance(getpoint(i), getpoint(j));
				edg.straight_line = true;
			}
			all_edges.push_back(edg);
		}
	}
	//TODO: check that ordered with smaller
	sort(all_edges.begin(),all_edges.end(),less<edge>());
	return all_edges;
}

void greedy_tspanner(graph& tspanner, vector<edge> edges){
	double dist;
	vector<int> shortest_path;
	vector<int> unused;
	for(int i=0;i<edges.size();++i) {
		dist = dijkstra_shortest_path(tspanner, edges[i].p1,edges[i].p2,unused);
		//cout << dist/euclidean_distance(getpoint(edges[i].p1),getpoint(edges[i].p2)) << endl;
		//dist>pr.t*euclidean_distance(getpoint(edges[i].p1),getpoint(edges[i].p2))
		if(dist == -1 || dist>pr.t*edges[i].dist) {
			//TODO: inserts twice some edges
			if(edges[i].straight_line) {
				insert_edge(tspanner, edges[i].p1, edges[i].p2);
			} else {
				shortest_path = edges[i].shortest_path;
				for(int j=1;j<shortest_path.size();++j) {
					insert_edge(tspanner,shortest_path[j-1],shortest_path[j]);
				}
			}
		}
	}
}

struct result {
	double dilation;
	int size;
	double weight;
	double execution_time;
};

void writeFile(char filename[], graph& graph, result& res) {
	cout << "Writting output"<< endl;
    ifstream file(filename);
    string str;
    ofstream outputFile("output.txt");
    while (getline(file, str))
    {
        outputFile << str << endl;
    }

    for(int i=0;i<graph.size();++i) {
    	for(set<int>::iterator it=graph[i].begin();it!=graph[i].end();++it){
    		outputFile << " " << *it;
    	}
    	outputFile << endl;
    }

    outputFile << res.dilation << endl;
    outputFile << res.size << endl;
    outputFile << res.weight << endl;
    outputFile << res.execution_time << endl;


}

result computeResultParameters(graph& tspanner, vector<edge>& edges) {
	result res;

	double dist,dilation_max=0,dil;
	vector<int> unused;
	for(int i=0;i<edges.size();++i) {
		dist = dijkstra_shortest_path(tspanner, edges[i].p1,edges[i].p2,unused);
		dil=dist/edges[i].dist;
		if(dil>dilation_max) {
			dilation_max=dil;
		}
		//cout << edges[i].p1 << " " << edges[i].p2 << " " << dil << endl;
	}
	res.dilation = dilation_max;

	int size=0;
	double weight=0;
	for(int i=0;i<tspanner.size();++i) {
		for(set<int>::iterator it=tspanner[i].begin();it!=tspanner[i].end();++it) {
			++size;
			weight += euclidean_distance(getpoint(*it),getpoint(i));
		}
	}

	res.size = size/(double)2;
	res.weight = weight/2;

	return res;
}

int main() {
	char filename[] = "Data_examples/geometricspanners.txt";
	readFile(filename);
	graph tspanner(pr.n+pr.nobs, set<int>());


	clock_t tStart = clock();
	vector<edge> all_edges_sorted = find_all_edges();

	greedy_tspanner(tspanner,all_edges_sorted);
	double execution_timer = (clock()-tStart)/(double)(CLOCKS_PER_SEC/1000);

	result res = computeResultParameters(tspanner,all_edges_sorted);
	res.execution_time = execution_timer;
	cout << "result: " << res.dilation << " " << res.size << " " << res.weight << " " << res.execution_time << endl;

	writeFile(filename, tspanner, res);

	/*graph tspanner(4, vector<int>(0));
//	Example of dijkstra distance
	vector<coord> points = {make_pair(0,0),make_pair(0,1),make_pair(1,1),make_pair(1,2)};
	pr.points = points;
	pr.n=4;
    insert_edge(tspanner, 0, 1);
    insert_edge(tspanner,1, 2);
    insert_edge(tspanner,2,3);
    //insert_edge(tspanner,1,2);

    vector<int> path1;
    cout << dijkstra_shortest_path(tspanner, 1,0,path1) << endl;
    print_vector(path1);
    cout << "aaaaaaaa" << endl;
    vector<int> path2;
    cout << dijkstra_shortest_path(tspanner, 2,0,path2) << endl;
    print_vector(path2);
    cout << "aaaaaaaa" << endl;
    vector<int> path3;
    cout << dijkstra_shortest_path(tspanner, 0,3,path3) << endl;
    print_vector(path3);
    cout << "aaaaaaaa" << endl;
    vector<int> path4;
    cout << dijkstra_shortest_path(tspanner, 2,3,path4) << endl;
    print_vector(path4);
    cout << "aaaaaaaa" << endl;
    vector<int> path5;
    cout << dijkstra_shortest_path(tspanner, 1,3,path5) << endl;
    print_vector(path5);*/

	/*cout << compute_angle(make_pair(0,1), make_pair(2,1), make_pair(3,4))*180/M_PI << endl;
	cout << compute_angle(make_pair(2,1), make_pair(0,1), make_pair(3,4))*180/M_PI << endl;
	cout << compute_angle(make_pair(0,1), make_pair(3,4), make_pair(2,1))*180/M_PI << endl;
	cout << compute_angle(make_pair(2,1), make_pair(3,4), make_pair(0,1))*180/M_PI << endl;
	cout << compute_angle(make_pair(3,4), make_pair(0,1), make_pair(2,1))*180/M_PI << endl;
	cout << compute_angle(make_pair(3,4), make_pair(2,1), make_pair(0,1))*180/M_PI << endl;*/

	/*cout << find_intersection_lines(make_pair(1.0,1.0), make_pair(2,2.0), make_pair(1,2.0), make_pair(2.0,1.0)).first << endl;
	cout << find_intersection_lines(make_pair(1.0,1.0), make_pair(2,2.0), make_pair(1,2.0), make_pair(2.0,1.0)).second << endl;
	cout << find_intersection_lines(make_pair(0.0,0.0), make_pair(4,4.0), make_pair(0,4), make_pair(4,0)).first << endl;
	cout << find_intersection_lines(make_pair(0.0,0.0), make_pair(4,4.0), make_pair(0,4), make_pair(4,0)).second << endl;
	cout << find_intersection_lines(make_pair(1,0), make_pair(-1,0), make_pair(0,1), make_pair(0,-1)).first << endl;
	cout << find_intersection_lines(make_pair(1,0), make_pair(-1,0), make_pair(0,1), make_pair(0,-1)).second << endl;
	cout << find_intersection_lines(make_pair(1,0), make_pair(-1,0), make_pair(2,0), make_pair(-2,0)).first << endl;
	cout << find_intersection_lines(make_pair(1,0), make_pair(-1,0), make_pair(2,0), make_pair(-2,0)).second << endl;*/


	//cout << point_in_segment(make_pair(1,1),make_pair(0,0), make_pair(2,2)) << endl;
	//cout << point_in_segment(make_pair(3,3),make_pair(0,0), make_pair(2,2)) << endl;


	//TODO:output to file and plot

	/*vector<coord> points = {make_pair(0,1)};
	pr.points = points;
	pr.n = 1;
	vector<coord> obst_vert = {make_pair(3,4),make_pair(4,0),make_pair(2,1),make_pair(1,2),make_pair(1,3)};
	pr.obstacle_vert = obst_vert;
	pr.nobs = 5;*/
	/*graph visibility(pr.n+pr.nobs, vector<int>(0));
	visibility_point(visibility, 0);
	for(int i = 0;i<visibility.size();++i) {
		cout << "row: " << i << endl;
		print_vector(visibility[i]);
	}*/


	/*pr.n = 5;
	status_segment sg1, sg2, sg3,sg4;
	sg1.p=0;
	sg1.p1=1;
	sg1.p2=2;
	sg4.p=0;
	sg4.p1=1;
	sg4.p2=3;
	sg2.p=0;
	sg2.p1=3;
	sg2.p2=4;
	sg3.p=0;
	sg3.p1 = 5;
	sg3.p2 = 6;
	set<status_segment> stat;
	stat.insert(sg1);
	stat.insert(sg4);
	stat.insert(sg2);
	stat.insert(sg3);
	for (set<status_segment>::iterator it = stat.begin() ; it != stat.end(); ++it) {
	    cout << ' ' << (*it).p1<< ' ' << (*it).p2<<endl;
	}*/


}
