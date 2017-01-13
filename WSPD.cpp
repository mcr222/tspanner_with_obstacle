#include <tuple>
#include <iostream>
#include <string>
#include<fstream>
#include <vector>
#include <map>
#include <math.h>
#include <queue>
#include <algorithm>

using namespace std;

typedef pair<int,int> coord;
typedef vector<vector<int>> graph;
struct problem {
	int n;
	int nobs;
	double t;
	vector<coord> points;
	vector<coord> obstacle_vert;
};

class Quadtree
{
	public:
        //4 children
        Quadtree* nw;
        Quadtree* ne;
        Quadtree* sw;
        Quadtree* se;

        vector<coord> boundary;
        vector<vector<coord>> containedPoints;


        Quadtree();
        Quadtree(vector<coord> boundary);
        Quadtree(vector<coord> boundary, vector<vector<coord>>containedPoints);

        ~Quadtree();

        bool insert(vector<coord> value);
        void setNEChild(vector<coord> childBoundary, vector<vector<coord>> childPoints);

};

Quadtree::Quadtree()
{
    nw = NULL;
    ne = NULL;
    sw = NULL;
    se = NULL;
    boundary = std::vector<coord>();
    containedPoints = std::vector<vector<coord>>();
}

Quadtree::Quadtree(vector<coord> boundary)
{
	containedPoints = std::vector<vector<coord>>();
    nw = NULL;
    ne = NULL;
    sw = NULL;
    se = NULL;
    this->boundary = boundary;
}

Quadtree::Quadtree(vector<coord> boundary,vector<vector<coord>> containedPoints )
{
	this->containedPoints = containedPoints;
    nw = NULL;
    ne = NULL;
    sw = NULL;
    se = NULL;
    this->boundary = boundary;
}

Quadtree::~Quadtree()
{
    delete nw;
    delete sw;
    delete ne;
    delete se;
}

/*void Quadtree::setNEChild(vector<coord> childBoundary, vector<coord> childPoints){
	Quadtree child(childBoundary,childPoints);
	this->ne = temp;
}*/



queue<pair<vector<vector<coord>>,vector<vector<coord>>>> pair_queue; // queue storing the well-separated pairs
vector<pair<Quadtree*,Quadtree*>> checkDuplicate; // vector storing cells for which wspd is already found

problem pr;
float epsilon; // to calculate 1/epsilion well-seperated pair

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
	prbl.t = a/b;
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

struct compare_x
{
    bool operator() (pair<int,int> &p1, pair<int,int> &p2)
    {
        return (p1.first < p2.first);
    }
};

struct compare_y
{
    inline bool operator() (coord &p1, coord &p2)
    {
        return (p1.second < p2.second);
    }
};

vector<coord> sort_x(vector<coord> points){
	sort(points.begin(), points.end(), compare_x());
	return points;
}

vector<coord> sort_y(vector<coord> points){
	sort(points.begin(), points.end(), compare_y());
	return points;
}

map<string, vector<coord>> split_points(vector<coord> points){

	vector<coord> split_ne;
	vector<coord> split_nw;
	vector<coord> split_se;
	vector<coord> split_sw;
	map < string, vector<coord>> splits;

	// constructs hyper rectangle and returns points splitted in two vectors.
	vector<coord>  x_sorted_points = sort_x(points);
	vector<coord>  y_sorted_points = sort_y(points);
	// hyperrectangle's diagonal has endpoint_lb, endpoint_lt, endpoint_rb and endpoint_rt as its endpoints
	coord endpoint_lb = make_pair(x_sorted_points.front().first, y_sorted_points.front().second);
	coord endpoint_lt = make_pair(x_sorted_points.front().first, y_sorted_points.back().second);
	coord endpoint_rb = make_pair(x_sorted_points.back().first, y_sorted_points.front().second);
	coord endpoint_rt = make_pair(x_sorted_points.back().first, y_sorted_points.back().second);

	// bounding box vertices stored
	vector<coord> hyper_rectangle;
	hyper_rectangle.push_back(endpoint_lb);
	hyper_rectangle.push_back(endpoint_lt);
	hyper_rectangle.push_back(endpoint_rt);
	hyper_rectangle.push_back(endpoint_rb);

	//point of intersection of diagonals
	coord intersection = make_pair((endpoint_lb.first + endpoint_rt.first)/2 ,(endpoint_lb.second + endpoint_rt.second)/2);
	hyper_rectangle.push_back(intersection);

	splits["hyper_rectangle"] = hyper_rectangle;


	// split points into 4 sets based on these 2 sides.
	double split_at_x;
	double split_at_y;

	split_at_x = (double)(endpoint_lb.first + endpoint_rb.first)/2;
	split_at_y = (double)(endpoint_lb.second + endpoint_lt.second)/2;

	size_t i;
	for(i =0 ; i < points.size(); i++){
		if(points.at(i).first > split_at_x && points.at(i).second > split_at_y)
			split_ne.push_back(points.at(i));
		else if (points.at(i).first <= split_at_x && points.at(i).second > split_at_y)
			split_nw.push_back(points.at(i));
		else if (points.at(i).first <= split_at_x && points.at(i).second <= split_at_y)
			split_sw.push_back(points.at(i));
		else if (points.at(i).first > split_at_x && points.at(i).second <= split_at_y)
			split_se.push_back(points.at(i));
		}

	splits["nw"]= split_nw;
	splits["ne"]= split_ne;
	splits["sw"]= split_sw;
	splits["se"]= split_se;


	return splits;
}

void constructQuadTree(Quadtree* quadtree,vector<coord> &points){

	if(points.size()== 1){
		vector<vector<coord>> singlePointSet;
		singlePointSet.push_back(points);
		quadtree->containedPoints = singlePointSet;
		/*//boundary is rectangle of side zero i.e. point itself
		// inserting 5 coordinates for boundary vertices and intersection points
		vector<coord> bound;
		for(int i=0;i<5;i++){
			bound.push_back(points.at(0));
		}
		quadtree->boundary = bound;*/
	}
	else{
		// split_points constructs hyper rectangle and splits the points into two vectors
		map < string, vector<coord>> splits;
		splits = split_points(points);
		vector<vector<coord>> pointSets;

		pointSets.push_back(splits["ne"]);
		pointSets.push_back(splits["nw"]);
		pointSets.push_back(splits["se"]);
		pointSets.push_back(splits["sw"]);

		quadtree->containedPoints = pointSets ;
		quadtree->boundary = splits["hyper_rectangle"];

		map < string, vector<coord>> splits_children;
		if(splits["ne"].size()!=0){
			quadtree->ne=  new Quadtree(splits["ne"]);
			splits_children = split_points(splits["ne"]);
			quadtree->ne->boundary = splits_children["hyper_rectangle"];
			constructQuadTree(quadtree->ne, splits["ne"]);
		}
		if(splits["nw"].size()!=0){
			quadtree->nw=  new Quadtree(splits["nw"]);
			splits_children = split_points(splits["nw"]);
			quadtree->nw->boundary = splits_children["hyper_rectangle"];
			constructQuadTree(quadtree->nw, splits["nw"]);
		}
		if(splits["se"].size()!=0){
			quadtree->se= new Quadtree(splits["se"]);
			splits_children = split_points(splits["se"]);
			quadtree->se->boundary = splits_children["hyper_rectangle"];
			constructQuadTree(quadtree->se, splits["se"]);
		}
		if(splits["sw"].size()!=0){
			quadtree->sw= new Quadtree(splits["sw"]);
			splits_children = split_points(splits["sw"]);
			quadtree->sw->boundary = splits_children["hyper_rectangle"];
			constructQuadTree(quadtree->sw, splits["sw"]);
		}
	}

}

void constructWSPD(Quadtree* cell1, Quadtree* cell2, float epsilon){
	/* boundary stored as..
	 * cell1.boundary.at(0) = lb
	 * cell1.boundary.at(1) = lt
	 * cell1.boundary.at(2) = rt
	 * cell1.boundary.at(3) = rb
	 * cell1.boundary.at(4) = intersection of cell diagonals
	 */

	double cellDiameter1 = euclidean_distance(cell1->boundary.at(0),cell1->boundary.at(2));
	double cellDiameter2 = euclidean_distance(cell2->boundary.at(0),cell2->boundary.at(2));

	/*cout<< cellDiameter1<<":cell1Diameter"<<endl;
	cout<< cellDiameter2 << ":cell2 Diameter"<<endl;*/

	double dist = euclidean_distance(cell1->boundary.at(4), cell2->boundary.at(4));
	/*cout<<"distance"<<dist<<endl;
	cout<< "epsilon * dist"<< epsilon * dist<<endl;*/

	if(cellDiameter1 < cellDiameter2){
		// Swap the cells
		Quadtree* temp;
		temp = cell1;
		cell1 = cell2;
		cell2 = temp;
	}
	if(cellDiameter1 < epsilon * dist ){
		/*cout<<"cellDiameter1 < epsilon * dist"<<endl;
		cout<<"------------------"<<endl;
		cout<< cell1->containedPoints.size()<<": cell1 points"<<endl;
		cout<< cell2->containedPoints.size()<<": cell2 points"<<endl;
		cout<<"-------------------"<<endl;*/
		//The two cell's points are 1/epsilon-separated
		pair_queue.push(make_pair(cell1->containedPoints,cell2->containedPoints));
		return;
	}
	if(!(cell1->ne == NULL && cell1->nw == NULL && cell1->se == NULL && cell1->sw == NULL)){
		if(cell1->ne !=NULL){
			if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->ne,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->ne)) == checkDuplicate.end()){
				constructWSPD(cell1->ne,cell2,epsilon);
				checkDuplicate.push_back(make_pair(cell1->ne,cell2));
			}

		}
		if(cell1->nw !=NULL){
			if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->nw,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->nw)) == checkDuplicate.end()){
				constructWSPD(cell1->nw,cell2,epsilon);
				checkDuplicate.push_back(make_pair(cell1->nw,cell2));
			}
		}
		if(cell1->se !=NULL){
			if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->se,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->se)) == checkDuplicate.end()){
				constructWSPD(cell1->se,cell2,epsilon);
				checkDuplicate.push_back(make_pair(cell1->se,cell2));
			}
		}
		if(cell1->sw !=NULL){
			if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->sw,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->sw)) == checkDuplicate.end()){
				constructWSPD(cell1->sw,cell2,epsilon);
				checkDuplicate.push_back(make_pair(cell1->sw,cell2));
			}
		}
	}
}

void writeFile(char filename[]) {
	cout << "Writing output"<< endl;
    ifstream file(filename);
    string str;
    ofstream outputFile("output_wspd.txt");
    /*while (getline(file, str))
    {
        outputFile << str << endl;
    }*/
    cout<< "Number of well-seperated paires created = "<<pair_queue.size()<<endl;
    outputFile << pair_queue.size() <<endl;
    pair<vector<coord>,vector<coord>> ws_pairs;
    while(!pair_queue.empty()){
    		ws_pairs = pair_queue.front();
    		pair_queue.pop();
    		cout<<"Sizes:"<<ws_pairs.first.size()<<","<<ws_pairs.second.size()<<endl;
    		outputFile << ws_pairs.first.size() << endl;
    		for(size_t i =0;i<ws_pairs.first.size();i++){
    			cout<<"("<< ws_pairs.first.at(i).first <<","<<ws_pairs.first.at(i).second<<")";
    			outputFile << ws_pairs.first.at(i).first << " " <<ws_pairs.first.at(i).second<<endl;
    		}
    		outputFile << ws_pairs.second.size() << endl;
    		cout<<endl;
    		for(size_t i =0;i<ws_pairs.second.size();i++){
    			cout<<"("<< ws_pairs.second.at(i).first <<","<<ws_pairs.second.at(i).second<<")";
    			outputFile << ws_pairs.second.at(i).first << " " <<ws_pairs.second.at(i).second<<endl;
    		}
    		cout<<endl;
    	}
}

int main() {
	char filename[] = "Data_examples/geometricspanners.txt";
	readFile(filename);
	//graph tspanner(pr.n, vector<int>(0));
	map < string, vector<coord>> splits;
	splits = split_points(pr.points);
	Quadtree quadtree(splits["hyper_rectangle"]);
	constructQuadTree(&quadtree,pr.points);

	/*cout<< quadtree.containedPoints.size()<<endl;
	cout<< quadtree.ne->containedPoints.size()<<endl;
	cout<< quadtree.nw->containedPoints.size()<<endl;
	cout<< quadtree.se->containedPoints.size()<<endl;
	cout<< quadtree.sw->containedPoints.size()<<endl;*/

	// assuming pr.t = 1.5
	cout<<pr.t<<": stretch factor"<<endl;
	//epsilon = (4*(pr.t +1))/( pr.t - 1 + 0.01);
	epsilon = 1.5; //TODO DELETE
	cout<<epsilon <<": epsilon"<<endl;

	constructWSPD(&quadtree,&quadtree,epsilon);

	writeFile(filename);

}
