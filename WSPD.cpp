#include <tuple>
#include <iostream>
#include <string>
#include<fstream>
#include <vector>
#include <map>
#include <math.h>
#include <queue>
#include <algorithm>
#include <list>

using namespace std;

typedef pair<double,double> coord;
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
        double cellArea;


        Quadtree();
        Quadtree(vector<coord> boundary);
        Quadtree(vector<coord> boundary, vector<vector<coord>>containedPoints);

        ~Quadtree();

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



vector<pair<Quadtree*,Quadtree*>> well_separated_pairs; // queue storing the well-separated pairs
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

double area(vector<coord> boundary){
	return euclidean_distance(boundary.at(0),boundary.at(1))*euclidean_distance(boundary.at(0),boundary.at(3));
}

coord getMidPoint(coord p1, coord p2) {
	return make_pair(((p1.first+p2.first)/2),(p1.second+p2.second)/2);
}

struct compare_x
{
    bool operator() (pair<double,double> &p1, pair<double,double> &p2)
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

vector<coord> getHyperRectangle(vector<coord> points){

	//corner case for two axis parallel points
	if(points.size()==2){
		sort(points.begin(), points.end());
		if(points.at(0).first == points.at(1).first){
			// points are y parallel
			coord endpoint_lb = make_pair(points.at(0).first, points.at(0).second);
			coord endpoint_lt = make_pair(points.at(1).first, points.at(1).second);
			coord endpoint_rb = make_pair(points.at(0).first + 1, points.at(0).second);
			coord endpoint_rt = make_pair(points.at(1).first +1,points.at(0).second);
			// bounding box vertices stored
				vector<coord> hyper_rectangle;
				hyper_rectangle.push_back(endpoint_lb);
				hyper_rectangle.push_back(endpoint_lt);
				hyper_rectangle.push_back(endpoint_rt);
				hyper_rectangle.push_back(endpoint_rb);

				//point of intersection of diagonals
				coord intersection = make_pair((endpoint_lb.first + endpoint_rt.first)/2 ,(endpoint_lb.second + endpoint_rt.second)/2);
				hyper_rectangle.push_back(intersection);
			return hyper_rectangle;

		}
		if(points.at(0).second == points.at(1).second){
			// points are x parallel
			coord endpoint_lb = make_pair(points.at(0).first, points.at(0).second);
			coord endpoint_lt = make_pair(points.at(0).first , points.at(0).second+1);
			coord endpoint_rb = make_pair(points.at(1).first , points.at(1).second );
			coord endpoint_rt = make_pair(points.at(1).first,points.at(1).second +1);
			// bounding box vertices stored
				vector<coord> hyper_rectangle;
				hyper_rectangle.push_back(endpoint_lb);
				hyper_rectangle.push_back(endpoint_lt);
				hyper_rectangle.push_back(endpoint_rt);
				hyper_rectangle.push_back(endpoint_rb);

				//point of intersection of diagonals
				coord intersection = make_pair((endpoint_lb.first + endpoint_rt.first)/2 ,(endpoint_lb.second + endpoint_rt.second)/2);
				hyper_rectangle.push_back(intersection);
			return hyper_rectangle;

		}
	}


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

	return hyper_rectangle;
}


map<string, vector<coord>> split_points(vector<coord> points, vector<coord> boundary){
	map < string, vector<coord>> splits;
	vector<coord> split_ne;
	vector<coord> split_nw;
	vector<coord> split_se;
	vector<coord> split_sw;

	vector<coord> boundary_ne;
	vector<coord> boundary_nw;
	vector<coord> boundary_se;
	vector<coord> boundary_sw;

	coord lb;
	coord lt;
	coord rt;
	coord rb;
	coord intersection;

	if(points.size()<=1){
		splits["single"] = points;

	}

	else{
		// split points into 4 sets based on these 2 sides.
			double split_at_x;
			double split_at_y;


			split_at_x = boundary.at(4).first;
			split_at_y = boundary.at(4).second;

			/*split_at_x = (boundary.at(3).first - boundary.at(0).first )/2;
			split_at_y = (boundary.at(1).second - boundary.at(0).second)/2;

			cout<<"Split_x:"<< boundary.at(3).first<<"-"<< boundary.at(0).first<<"/2 ="<< split_at_x<<endl;
			cout<<"Split_y:"<< boundary.at(1).second <<"-"<< boundary.at(0).second<<"/2="<<split_at_y<<endl;*/
			//cout<<"("<<boundary.at(4).first<<","<<boundary.at(4).second<<")"<<"...intersection"<<endl;

			//if(split_at_x!=0 && split_at_y!=0){
		size_t i;
		for(i =0 ; i < points.size(); i++){
			//cout<<"("<<points.at(i).first<<","<<points.at(i).second<<")";
			if(points.at(i).first > split_at_x && points.at(i).second > split_at_y)
				split_ne.push_back(points.at(i));
			else if (points.at(i).first <= split_at_x && points.at(i).second > split_at_y)
				split_nw.push_back(points.at(i));
			else if (points.at(i).first <= split_at_x && points.at(i).second <= split_at_y)
				 split_sw.push_back(points.at(i));
			else
				split_se.push_back(points.at(i));
			}
		//cout<<"sw:"<<split_sw.size()<<endl;

		splits["nw"]= split_nw;
		splits["ne"]= split_ne;
		splits["sw"]= split_sw;
		splits["se"]= split_se;



		// ne boundary
		if(split_ne.size()>0){
				lb = boundary.at(4);
				lt = make_pair(boundary.at(4).first, boundary.at(1).second);
				rt = boundary.at(2);
				rb = make_pair(boundary.at(2).first,boundary.at(4).second);
				intersection = make_pair((lb.first + rt.first)/2 ,(lb.second + rt.second)/2);
				boundary_ne.push_back(lb);
				boundary_ne.push_back(lt);
				boundary_ne.push_back(rt);
				boundary_ne.push_back(rb);
				boundary_ne.push_back(intersection);

				splits["ne_boundary"] = boundary_ne;
				/*cout<<"NE_BOUNDARY"<<endl;
				for(size_t i =0;i<boundary_ne.size();i++){
						cout<<"("<<boundary_ne.at(i).first<<","<<boundary_ne.at(i).second<<"),";
					}
				cout<<endl;*/
			}

			// nw boundary
			if(split_nw.size() >0 ){
				lb = make_pair(boundary.at(0).first,boundary.at(4).second);
				lt = boundary.at(1);
				rt = make_pair(boundary.at(4).first, boundary.at(1).second);
				rb = boundary.at(4);
				intersection = make_pair((lb.first + rt.first)/2 ,(lb.second + rt.second)/2);
				boundary_nw.push_back(lb);
				boundary_nw.push_back(lt);
				boundary_nw.push_back(rt);
				boundary_nw.push_back(rb);
				boundary_nw.push_back(intersection);

				splits["nw_boundary"] = boundary_nw;
				/*cout<<"NW_BOUNDARY"<<endl;
				for(size_t i =0;i<boundary_ne.size();i++){
						cout<<"("<<boundary_nw.at(i).first<<","<<boundary_nw.at(i).second<<"),";
					}
				cout<<endl;*/
			}

			// sw boundary
			if(split_sw.size()>0){
				lb = boundary.at(0);
				lt = make_pair(boundary.at(0).first, boundary.at(4).second);
				rt = boundary.at(4);
				rb = make_pair(boundary.at(4).first,boundary.at(0).second);
				intersection = make_pair((lb.first + rt.first)/2 ,(lb.second + rt.second)/2);
				boundary_sw.push_back(lb);
				boundary_sw.push_back(lt);
				boundary_sw.push_back(rt);
				boundary_sw.push_back(rb);
				boundary_sw.push_back(intersection);

				splits["sw_boundary"] = boundary_sw;
				/*cout<<"SW_BOUNDARY"<<endl;
				for(size_t i =0;i<boundary_sw.size();i++){
						cout<<"("<<boundary_sw.at(i).first<<","<<boundary_sw.at(i).second<<"),";
					}
				cout<<endl;*/
			}

			// se boundary
			if(split_se.size() > 0){
				lb = make_pair(boundary.at(4).first,boundary.at(0).second);
				lt = boundary.at(4);
				rt = make_pair(boundary.at(3).first,boundary.at(4).second);
				rb = boundary.at(3);
				intersection = make_pair((lb.first + rt.first)/2 ,(lb.second + rt.second)/2);
				boundary_se.push_back(lb);
				boundary_se.push_back(lt);
				boundary_se.push_back(rt);
				boundary_se.push_back(rb);
				boundary_se.push_back(intersection);

				splits["se_boundary"] = boundary_se;
				/*cout<<"SE_BOUNDARY"<<endl;
				for(size_t i =0;i<boundary_se.size();i++){
						cout<<"("<<boundary_se.at(i).first<<","<<boundary_se.at(i).second<<"),";
					}
				cout<<endl;*/
			}
	//	}

	}

	return splits;
}




void constructQuadTree(Quadtree* quadtree,vector<coord> &points){

	/*cout<<"BOUNDARY"<<endl;
	for(size_t i =0;i<quadtree->boundary.size();i++){
			cout<<"("<<quadtree->boundary.at(i).first<<","<<quadtree->boundary.at(i).second<<"),";
		}
	cout<<endl;*/

	if(points.size()== 1){
		/*cout<<"HERE"<<endl;
		cout<<points.at(0).first<<","<<points.at(0).second<<endl;
		cout<<"***************";*/
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
		splits = split_points(points, quadtree->boundary);
		vector<vector<coord>> pointSets;

		if(splits["ne"].size()!=0)
			pointSets.push_back(splits["ne"]);
		if(splits["nw"].size()!=0)
			pointSets.push_back(splits["nw"]);
		if(splits["se"].size()!=0)
			pointSets.push_back(splits["se"]);
		if(splits["sw"].size()!=0)
			pointSets.push_back(splits["sw"]);

		quadtree->containedPoints = pointSets ;

		if(splits["ne"].size()!=0 && splits.count("ne_boundary")==1){
			vector<coord> outerBoundary = getHyperRectangle(splits["ne"]);
			quadtree->ne=  new Quadtree(outerBoundary);
			//quadtree->ne->cellArea = area(splits["ne_boundary"]);
			quadtree->ne->cellArea = quadtree->cellArea /4;
			constructQuadTree(quadtree->ne, splits["ne"]);
		}
		if(splits["nw"].size()!=0 && splits.count("nw_boundary")==1){
			vector<coord> outerBoundary= getHyperRectangle(splits["nw"]);
			quadtree->nw=  new Quadtree(outerBoundary);
			//quadtree->nw->cellArea = area(splits["nw_boundary"]);
			quadtree->nw->cellArea = quadtree->cellArea /4;
			constructQuadTree(quadtree->nw, splits["nw"]);
		}
		if(splits["se"].size()!=0 && splits.count("se_boundary")==1){
			vector<coord> outerBoundary = getHyperRectangle(splits["se"]);
			quadtree->se= new Quadtree(outerBoundary);
			//quadtree->se->cellArea = area(splits["se_boundary"]);
			quadtree->se->cellArea = quadtree->cellArea /4;
			constructQuadTree(quadtree->se, splits["se"]);
		}
		if(splits["sw"].size()!=0 && splits.count("sw_boundary")==1){
			vector<coord> outerBoundary= getHyperRectangle(splits["sw"]);
			quadtree->sw= new Quadtree(outerBoundary);
			//quadtree->sw->cellArea = area(splits["sw_boundary"]);
			quadtree->sw->cellArea = quadtree->cellArea /4;
			constructQuadTree(quadtree->sw, splits["sw"]);
		}

	}

}

void printContainedPoints(Quadtree* quadTree){
	if(quadTree!=NULL){
			int total=0;
		for (size_t i=0;i<quadTree->containedPoints.size();i++){
					total += quadTree->containedPoints.at(i).size();
					cout<<"(";
					for(size_t j=0;j<quadTree->containedPoints.at(i).size();j++){
						cout<<"("<<quadTree->containedPoints.at(i).at(j).first<<","<<quadTree->containedPoints.at(i).at(j).second<<")";
					}
					cout<<"),";
				}
				cout<<endl;
				cout<<"Total Points in cell="<<total<<endl;
	}
}

void printQuadTree(Quadtree* quadTree){
	if(quadTree!=NULL){
		int total=0;
		for (size_t i=0;i<quadTree->containedPoints.size();i++){
			total += quadTree->containedPoints.at(i).size();
			cout<<"(";
			for(size_t j=0;j<quadTree->containedPoints.at(i).size();j++){
				cout<<"("<<quadTree->containedPoints.at(i).at(j).first<<","<<quadTree->containedPoints.at(i).at(j).second<<")";
			}
			cout<<"),";
		}
		cout<<endl;
		cout<<"Total Points in cell="<<total<<endl;
		cout<<"-----------------"<<endl;
		printQuadTree(quadTree->ne);
		printQuadTree(quadTree->nw);
		printQuadTree(quadTree->se);
		printQuadTree(quadTree->sw);
	}
}

vector<coord> getContainedPoints(Quadtree* quadTree){
	vector<coord> containedPoints;
	for (size_t i=0;i<quadTree->containedPoints.size();i++){
		for(size_t j=0;j<quadTree->containedPoints.at(i).size();j++){
			containedPoints.push_back(make_pair(quadTree->containedPoints.at(i).at(j).first,quadTree->containedPoints.at(i).at(j).second));
		}
	}
	return containedPoints;
}

vector<coord> getFarthestPoints(vector<coord> points){
	sort(points.begin(), points.end());
	vector<coord> farthestPoints;
	//cout<<"("<<points.front().first<<","<<points.front().second<<"),("<<points.back().first<<","<<points.back().second<<")"<<endl;
	farthestPoints.push_back(points.front());
	farthestPoints.push_back(points.back());
	return farthestPoints;
}

void getTotalPoints(Quadtree* quadTree){
	if(quadTree!=NULL){
		int total=0;
		for (size_t i=0;i<quadTree->containedPoints.size();i++){
			total += quadTree->containedPoints.at(i).size();
			cout<<"(";
			for(size_t j=0;j<quadTree->containedPoints.at(i).size();j++){
				cout<<"("<<quadTree->containedPoints.at(i).at(j).first<<","<<quadTree->containedPoints.at(i).at(j).second<<")";
			}
			cout<<"),";
		}
		cout<<"Total points:"<<total<<endl;
	}
	}

bool checkLeaves(Quadtree* cell1, Quadtree* cell2){
	if(cell1 == NULL) return false;
	//return checkLeaves(cell1->ne,cell2) && checkLeaves(cell1->nw,cell2) && checkLeaves(cell1->se,cell2) && checkLeaves(cell1->sw,cell2);
	return cell2 == cell1->ne || cell2 == cell1->nw || cell2 == cell1->se || cell2== cell1->sw ;
}

vector<coord> instersection(vector<coord> &v1, vector<coord> &v2)
{

    vector<coord> v3;

    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());

    set_intersection(v1.begin(),v1.end(),v2.begin(),v2.end(),back_inserter(v3));

    return v3;
}


void constructWSPD(Quadtree* cell1, Quadtree* cell2, float epsilon){
	/* boundary stored as..
	 * cell1.boundary.at(0) = lb
	 * cell1.boundary.at(1) = lt
	 * cell1.boundary.at(2) = rt
	 * cell1.boundary.at(3) = rb
	 * cell1.boundary.at(4) = intersection of cell diagonals
	 */
	if(cell1!=NULL && cell2!=NULL){


		/*double farthest1;
		double farthest2;*/

		vector<coord> cell1Points = getHyperRectangle(getContainedPoints(cell1));
		vector<coord> cell2Points = getHyperRectangle(getContainedPoints(cell2));

		/*farthest1 = getFarthestPoints(cell1Points);
		farthest2 = getFarthestPoints(cell2Points);*/

		double cellDiameter1 = euclidean_distance(cell1Points.at(0),cell1Points.at(2));
		double cellDiameter2 = euclidean_distance(cell2Points.at(0),cell2Points.at(2));

		double areaCell1 = euclidean_distance(cell1->boundary.at(0),cell1->boundary.at(1))*euclidean_distance(cell1->boundary.at(0),cell1->boundary.at(3));
		double areaCell2 = euclidean_distance(cell2->boundary.at(0),cell2->boundary.at(1))*euclidean_distance(cell2->boundary.at(0),cell2->boundary.at(3));

		double largerDiameter = cellDiameter1 >cellDiameter2 ? cellDiameter1 : cellDiameter2;

		double dist = euclidean_distance(cell1Points.at(4), cell2Points.at(4));

		// if cell2 is a leaf of cell1, return;
		if (cell1Points.size()==0 || cell2Points.size()==0 || checkLeaves(cell1,cell2)){
			return;
		}
		/*coord cen;
					cout<<"Diameters calculated"<<endl;
					cout<<cellDiameter1<<":cell1Diameter"<<endl;
					cout<<cellDiameter2<<":cell2Diameter"<<endl;
					cout<<largerDiameter<<":the larger one"<<endl;
					cen = cell1Points.at(4);
					cout<<"center1...("<<cen.first<<","<<cen.second<<")"<<endl;
					cen = cell2Points.at(4);
					cout<<"center2...("<<cen.first<<","<<cen.second<<")"<<endl;
					cout<<"distance........"<<dist<<endl;
					 cout<< "epsilon * radius......"<< epsilon * largerDiameter/2 <<endl;
					 cout<<"...cell1"<<endl;
					 getTotalPoints(cell1);
					 cout<<".....cell2"<<endl;
					 getTotalPoints(cell2);
					 cout<<"CHECK IF WS"<<endl;*/
		if(dist >= epsilon * (largerDiameter/2) ){
			/*cout<<"WELL-SEPARATED"<<endl;
			coord cen;
			cout<<"Diameters calculated"<<endl;
			cout<<cellDiameter1<<":cell1Diameter"<<endl;
			cout<<cellDiameter2<<":cell2Diameter"<<endl;
			cout<<largerDiameter<<":the larger one"<<endl;
			cen = cell1Points.at(4);
			cout<<"center1...("<<cen.first<<","<<cen.second<<")"<<endl;
			cen = cell2Points.at(4);
			cout<<"center2...("<<cen.first<<","<<cen.second<<")"<<endl;
			cout<<"distance........"<<dist<<endl;
			 cout<< "epsilon * radius......"<< epsilon * largerDiameter/2 <<endl;
			 cout<<"...cell1"<<endl;
			 getTotalPoints(cell1);
			 cout<<".....cell2"<<endl;
			 getTotalPoints(cell2);*/

			 // If leaf cells, and equal, remove them
			 bool equalLeaves = false;
			 if(cell1->containedPoints.size() == 1 && cell2->containedPoints.size()==1){
				 if (cell1->containedPoints.at(0).size() == 1 && cell2->containedPoints.at(0).size()==1){
					 if( cell1->containedPoints.at(0).at(0).first == cell2->containedPoints.at(0).at(0).first && cell1->containedPoints.at(0).at(0).second == cell2->containedPoints.at(0).at(0).second){
						 equalLeaves = true;
					 }
				 }
			 }


			//The two cell's points are 1/epsilon-separated
			//vector<coord> diff = instersection(cell1Points, cell2Points);
			// To remove such pairs.
			//if(diff.size() <=0 )
			 if(!equalLeaves)
				well_separated_pairs.push_back(make_pair(cell1,cell2));
			 return;
		}
	else{
		if(cell1->cellArea < cell2->cellArea){
		//if(areaCell1 < areaCell2){
				// Swap the cells
				/*cout<<"--------------------"<<endl;
				cout<<"BEFORE SWAP"<<endl;
				printContainedPoints(cell1);
				printContainedPoints(cell2);*/
				Quadtree* temp;
				temp = cell1;
				cell1 = cell2;
				cell2 = temp;
				/*cout<<"SWAPPED"<<endl;
				printContainedPoints(cell1);
				printContainedPoints(cell2);
				cout<<"0000000000000000000000000"<<endl;*/
			}
		if(!(cell1->ne == NULL && cell1->nw == NULL && cell1->se == NULL && cell1->sw == NULL)){
			if(cell1->ne !=NULL){
				if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->ne,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->ne)) == checkDuplicate.end()){
					checkDuplicate.push_back(make_pair(cell1->ne,cell2));
					constructWSPD(cell1->ne,cell2,epsilon);
				}

			}
			if(cell1->nw !=NULL){
				if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->nw,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->nw)) == checkDuplicate.end()){
					checkDuplicate.push_back(make_pair(cell1->nw,cell2));
					constructWSPD(cell1->nw,cell2,epsilon);
				}
			}
			if(cell1->se !=NULL){
				if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->se,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->se)) == checkDuplicate.end()){
					checkDuplicate.push_back(make_pair(cell1->se,cell2));
					constructWSPD(cell1->se,cell2,epsilon);
				}
			}
			if(cell1->sw !=NULL){
				if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->sw,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->sw)) == checkDuplicate.end()){
					checkDuplicate.push_back(make_pair(cell1->sw,cell2));
					constructWSPD(cell1->sw,cell2,epsilon);
				}
			}
		}
	}
}
}

void writeFile(char filename[], int depth) {
	/*
	 *
	 * DOES NOT WRITE TO FILE. will change after format decided upon
	 *
	 */
	cout << "Writing output"<< endl;
    ifstream file(filename);
    string str;
    ofstream outputFile("output_wspd.txt");
    /*while (getline(file, str))
    {
        outputFile << str << endl;
    }*/
    outputFile<< "Number of well-separated pairs created = "<<well_separated_pairs.size()<<endl;
    outputFile<<" Depth of quadtree = "<<depth;
    cout<< "Number of well-separated pairs created = "<<well_separated_pairs.size()<<endl;
    //outputFile << well_separated_pairs.size() <<endl;
    pair<Quadtree*,Quadtree*> ws_pairs;
    for(size_t i = 0; i<well_separated_pairs.size();i++){
    		ws_pairs = well_separated_pairs.at(i);
    		//well_separated_pairs.pop();
    		printContainedPoints(ws_pairs.first);
    		printContainedPoints(ws_pairs.second);
    		cout<<endl;
    	}
}

vector<coord> getAllPoints(vector<vector<coord>> points){
	vector<coord> allPoints;
	for(size_t i = 0;i< points.size();i++){
		for(size_t j=0;j<points.at(i).size();j++){
			allPoints.push_back(points.at(i).at(j));
		}
	}
	return allPoints;
}

int getDepth(Quadtree* quadtree){
	if(quadtree == NULL){
		return 0;
	}
	else{
		int ne = getDepth(quadtree->ne);
		int nw = getDepth(quadtree->nw);
		int se = getDepth(quadtree->se);
		int sw = getDepth(quadtree->sw);

		return max(max(ne,nw),max(se,sw))+1;
	}
}


int main() {
	char filename[] = "Data_examples/geometricspanners.txt";
	readFile(filename);
	//graph tspanner(pr.n, vector<int>(0));
	vector<coord> outerBoundary = getHyperRectangle(pr.points);
	Quadtree quadtree = Quadtree(outerBoundary);
	quadtree.cellArea = area(outerBoundary);
	constructQuadTree(&quadtree,pr.points);

	//printQuadTree(&quadtree);

	int depth = getDepth(&quadtree);

	// assuming pr.t = 1.5
	cout<<pr.t<<": stretch factor"<<endl;
	pr.t = 2;
	epsilon = (4*(pr.t +1))/( pr.t - 1);
	//epsilon = 1.05; //TODO DELETE
	cout<<epsilon <<": epsilon"<<endl;
	constructWSPD(&quadtree,&quadtree,epsilon);


	writeFile(filename, depth);




}
