#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// to get info from map
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>



#include <stdio.h>
#include <cmath>

#include <ctime>

#include <string>

using namespace std;


const double pi = 3.14159;




double angular;
double linear;

double bot_x;
double bot_y;
double bot_yaw;


const double max_lin_speed = 0.25;
const double max_ang_speed = pi/5;


class Timer
{
    private:
    clock_t startTime;

    public:
    Timer ()
    {
        startTime = std::clock();
    }

    double getElapsedTime()
    {
        return (double) (std::clock() - startTime) / CLOCKS_PER_SEC;
    }
};

struct node {
    typedef pair<int, int> edge_pair; 	// (edge cost, node index)
    // For our grid we can have 8 node transitions
    edge_pair n, s, e, w, ne, se, nw, sw;
    edge_pair* edges[8] = {&n, &ne, &e, &se, &s, &sw, &w, &nw};
    int value = 0;
    int has_360 = 0;
    int has_wiggle = 0;
    int idx;
};

class GridGraph
{
public:
    node* nodes;
    int w, h, size;
    GridGraph(int w, int h)
    {
    	nodes = new node[w*h];
    	size = w*h;
		GridGraph::w = w;
		GridGraph::h = h;
    	for (int i=0; i < w*h; i++)
    	{
    		node new_node;
    		new_node.idx = i;
    		new_node.value = 0;
			new_node.nw= make_pair(1, i - w - 1);
			new_node.n = make_pair(1, i - w);
			new_node.ne= make_pair(1, i - w + 1);
			new_node.sw= make_pair(1, i + w - 1);
			new_node.s = make_pair(1, i + w);
			new_node.se= make_pair(1, i + w + 1);
			new_node.w = make_pair(1, i - 1);
			new_node.e= make_pair(1, i + 1);

    		if (i < w)		// if first row, we can't have norther neighbours
			{
    			new_node.nw = make_pair(-1, -1);
    			new_node.n = make_pair(-1, -1);
    			new_node.ne = make_pair(-1, -1);
			}
    		if (i >= w*(h-1)) // if last row, we can't have souther neighbours
    		{
    			new_node.sw = make_pair(-1, -1);
    			new_node.s = make_pair(-1, -1);
    			new_node.se = make_pair(-1, -1);
    		}
    		if (i % w == 0) // if first col, we can't have a western neighbour
    		{
    			new_node.nw = make_pair(-1, -1);
    			new_node.w = make_pair(-1, -1);
    			new_node.sw = make_pair(-1, -1);
    		}
    		if (i % w == w - 1) // if last col, we can't have a eastern neighbour
    		{
    			new_node.ne = make_pair(-1, -1);
    			new_node.e = make_pair(-1, -1);
    			new_node.se = make_pair(-1, -1);
    		}
    		nodes[i] = new_node;
    	} // for
    } // constructor
};

class GridMap
{
public:
	int prev, curr, origin;
	GridGraph* world;
	double w, h, res, diagonal;
	vector<int> accumulation, visited;
	typedef pair<int, int> range_pair; 	// (start, end)
	range_pair* convex_blob;
	int accumulation_size = 10;
	int accumulation_sum = 0;
public:
	/*
	 * Here the GridMap is a grid graph with unidirectional edges
	 * Node.value = {-1\cannot go 0\unexplored 1\visited}
	 * edges.cost = {-1\cannot traverse 0\unknown 1\can traverse}
	 */
	GridMap(double w, double h, double res)
	{
		int size = ceil(max(w,h)*3/res);
		if (size % 2 == 0) {size += 1;}
		origin = (size+1)*(int(size/2));	 // pick the middle node as origin
		prev = origin;
		curr = origin;
		cout << "size = " << size << " -> " << size*size << " origin = " << origin <<endl;
		world = new GridGraph(size, size);
		convex_blob = new range_pair[size];		// one range pair for each row
		for (int i = 0; i < size; i++)
			convex_blob[i] = make_pair(-1,-1);
		diagonal = sqrt(2)*res;
		GridMap::res = res;
	}

	void visitLocation(double x, double y)
	{
		prev = curr;
		curr = getNode(x,y);
		if (prev != curr)
		{
			accumulation.insert(accumulation.begin(), world->nodes[curr].value);
			accumulation_sum += world->nodes[curr].value;
			if ((int)accumulation.size() > accumulation_size)
			{
				accumulation_sum -= accumulation.back();
				accumulation.pop_back();
			}
			world->nodes[curr].value += 1;		// Add 1 to the number of times the node has been visitied
			interpolateEdge(curr, prev, 1);		// Set the edge that we traversed through = 1
			if (find(visited.begin(), visited.end(), curr) != visited.end())
				visited.insert(visited.begin(), curr);
			else
				addToConvexBlob(curr);
		}
	}

	int getNodeRow(int node) {return node/world->w;}

	void addToConvexBlob(int node)
	{
		// Major assumption is that the robot does not teleport and thus the blob is connected
		int row = getNodeRow(node);
		if (convex_blob[row].first == -1)
		{
			convex_blob[row].first = node;
			convex_blob[row].second = node;
		}
		else
		{
			convex_blob[row].first = min(convex_blob[row].first, node);
			convex_blob[row].second = max(convex_blob[row].second, node);
		}
	}

	bool insideConvexBlob(int node, int strict)
	{
		// Strictly inside means that the query node cannot land on the edge of the blob
		int row = getNodeRow(node);
		if (convex_blob[row].first == -1)
			return false;
		else if (convex_blob[row].first < node < convex_blob[row].second && strict ==1)
			return true;
		else if (convex_blob[row].first <= node <= convex_blob[row].second && strict ==0)
			return true;
		else
			return false;
	}

	int getNodeValue(int node)
	{
		return world->nodes[node].value;
	}

	int getCurrNode() {return curr;}

	int getPrevNode() {return prev;}

	void markNodeHas360(int node)
	{
		world->nodes[node].has_360 += 1;
	}

	int getNodeHas360(int node)
	{
		return world->nodes[node].has_360;
	}

	void markNodeHasWiggle(int node)
	{
		world->nodes[node].has_wiggle = 1;
	}

	int getNodeHasWiggle(int node)
	{
		return world->nodes[node].has_wiggle;
	}

	void markNextAsUnaccessible(double x, double y, double yaw)
	{
		int nxt = getNextNode(x,y,yaw);
		// Note: A node might be accessible via other edges so we do not mark the nxt node as unaccessible
		interpolateEdge(nxt,curr,-1);
		 print_node(getNode(x,y));	   // Here we should see that the nodes outside the walls are also outside of the blob
	}

	int getNode(double x, double y)
	{
		int idx = origin - (int)round(y/res)*world->w;
		idx += (int)round(x/res);
		// when the idx is out of bounds, it raises seg fault; treat case here
		if (idx >= world->size || idx < 0)
		{
			//cout << "WARN getNode out of bound (x,y,w,res): "<< x << ", " << y << ", " << res << ", " << world->w << endl;
			//cout << prev << " prevNode|thisNode " << idx << endl;
			//cout << "Returning Origin" << endl;
			return -1;
		}
		return idx;
	}

	// In this first version we assume that the dest and src grids are neighbours
	void interpolateEdge(int dest, int src, int c)
	{
		if (world->nodes[dest].n.second == src) {world->nodes[dest].n.first=c; world->nodes[src].s.first=c;}
		if (world->nodes[dest].s.second == src) {world->nodes[dest].s.first=c; world->nodes[src].n.first=c;}
		if (world->nodes[dest].e.second == src) {world->nodes[dest].e.first=c; world->nodes[src].w.first=c;}
		if (world->nodes[dest].w.second == src) {world->nodes[dest].w.first=c; world->nodes[src].e.first=c;}
		if (world->nodes[dest].ne.second == src) {world->nodes[dest].ne.first=c; world->nodes[src].sw.first=c;}
		if (world->nodes[dest].nw.second == src) {world->nodes[dest].nw.first=c; world->nodes[src].se.first=c;}
		if (world->nodes[dest].se.second == src) {world->nodes[dest].se.first=c; world->nodes[src].nw.first=c;}
		if (world->nodes[dest].sw.second == src) {world->nodes[dest].sw.first=c; world->nodes[src].ne.first=c;}
	}

	void getUnexploredWeights(double x, double y, double yaw, int n_sectors, double fov, double* weights)
	{
		//cout << "getting unexplored weights with n_sectors = " << n_sectors << endl;
		// Extend n_sectors number of rays centered about the yaw angle and compute the weight of that ray
		// Here we define the "weight of that ray" to be the number of unvisited nodes encountered
		// The ray sample distance/step size is equal to the diagonal distance of one grid
		double d;
		double delta_yaw = fov/(double) n_sectors;
		double flip = 1;
		double yaw_ray = (yaw + flip*fov/2.0) - flip*delta_yaw/2.0;	// [  ^  |  ^  |  ^  |  ^  ] n_sector = 4, delta_yaw = 4 spaces fov denoted by [].	We order the rays in descending order due to the right handed-ness of the planner code
		for (int sector = 0; sector < n_sectors; sector++)
		{
			d = 0;
			weights[sector] = 0;
			// We step n times along the ray
			int max_steps = 0;
			for (int step = 1; step < 20; step++)
			{
				d += diagonal;
				int nxt_node = getNextNodeRay(x,y,yaw_ray,d);
				// First make sure the next node is valid
				if (nxt_node < 0)
					break;
				// If the next node along the ray is unexplored, add one to the weights
				if (world->nodes[nxt_node].value == 0)
					weights[sector] += 1;
				// However if the next node is inaccessible, stop the ray trace
				if (world->nodes[nxt_node].value == -1)
					break;
				max_steps = step;
			}
			cout << "(yaw,weight,steps)"<<yaw_ray<<", "<<weights[sector]<<", "<<max_steps<< "| ";
			yaw_ray -= flip*delta_yaw;
		}
		cout << endl;
	}

	double getVisitAccumulation()
	{
		return (double) accumulation_sum / (double) accumulation_size;
	}

	void resetAccumulation()
	{
		accumulation_sum = 0;
		accumulation.clear();
	}

	int getNextNodeRay(double x, double y, double yaw, double d)
	{
		double nxt_x = cos(yaw)*d + x;
		double nxt_y = sin(yaw)*d + y;
		//cout << "from nxtnoderay: " << d << ", xy " << x << ", " << y << endl;
		return getNode(nxt_x, nxt_y);
	}

	int getNextNode(double x, double y, double yaw)
	{
		// Here's another method, simply bin the yaw into the 8 direction n s e w ne se nw sw.
		//cout << "getting next node" << endl;
		int curr_node = getNode(x,y);
		int cardinal_direction = getCardinalDirection(yaw);
		int nxt_node;
		switch (cardinal_direction)
		{
			case 0:	nxt_node = world->nodes[curr_node].n.second;break;
			case 1: nxt_node = world->nodes[curr_node].ne.second;break;
			case 2:	nxt_node = world->nodes[curr_node].e.second;break;
			case 3:	nxt_node = world->nodes[curr_node].se.second;break;
			case 4:	nxt_node = world->nodes[curr_node].s.second;break;
			case 5:	nxt_node = world->nodes[curr_node].sw.second;break;
			case 6:	nxt_node = world->nodes[curr_node].w.second;break;
			case 7:	nxt_node = world->nodes[curr_node].nw.second;break;
		}
		return nxt_node;
	}

	double getTargetYaw(int cardinal_direction)
	{
		switch(cardinal_direction)
		{
			case -1: return bot_yaw;
			case 0: return pi/2;
			case 1: return pi/4;
			case 2: return 0;
			case 3: return -pi/4;
			case 4: return -pi/2;
			case 5: return -3*pi/4;
			case 6: return -pi;
			case 7: return 3*pi/4;
		}
	}

	int getCardinalDirection(double yaw)
	{
		double half_sec =pi/8;
		if (yaw >= -half_sec && yaw < half_sec)
			return 2;
		if (yaw >= half_sec && yaw < 3*half_sec)
			return 1;
		if (yaw >= 3*half_sec && yaw < 5*half_sec)
			return 0;
		if (yaw >= 5*half_sec && yaw < 7*half_sec)
			return 7;
		if ((yaw >= 7*half_sec && yaw <= pi) || (yaw >= -pi && yaw < -7*half_sec))
			return 6;
		if (yaw >= -7*half_sec && yaw < -5*half_sec)
			return 5;
		if (yaw >= -5*half_sec && yaw < -3*half_sec)
			return 4;
		if (yaw >= -3*half_sec && yaw < -half_sec)
			return 3;
	}

	int getMostUnexploredDirection(int node)
	{
		int max_score = 0, score;
		int best_direction = -1;
		int edges[8];
		edges[0] = world->nodes[node].n.first;
		edges[1] = world->nodes[node].ne.first;
		edges[2] = world->nodes[node].e.first;
		edges[3] = world->nodes[node].se.first;
		edges[4] = world->nodes[node].s.first;
		edges[5] = world->nodes[node].sw.first;
		edges[6] = world->nodes[node].w.first;
		edges[7] = world->nodes[node].nw.first;
		int next_nodes[8];
		next_nodes[0] = world->nodes[node].n.second;
		next_nodes[1] = world->nodes[node].ne.second;
		next_nodes[2] = world->nodes[node].e.second;
		next_nodes[3] = world->nodes[node].se.second;
		next_nodes[4] = world->nodes[node].s.second;
		next_nodes[5] = world->nodes[node].sw.second;
		next_nodes[6] = world->nodes[node].w.second;
		next_nodes[7] = world->nodes[node].nw.second;
		for (int dir = 0; dir < 8; dir++)
		{
			int edge_cost = edges[dir];
			int next_node = next_nodes[dir];
			int next_value = world->nodes[next_node].value;

			score = 2 - 2*next_value;

			if (edge_cost == -1)
			 	score -= 1;
			if (score > max_score && insideConvexBlob(next_node, 1))
			{
				max_score = score;
				best_direction = dir;
			}
		}
		return best_direction;
	}
	void print_node(int node)
	{
		int edges[8];
		edges[0] = world->nodes[node].n.first;
		edges[1] = world->nodes[node].ne.first;
		edges[2] = world->nodes[node].e.first;
		edges[3] = world->nodes[node].se.first;
		edges[4] = world->nodes[node].s.first;
		edges[5] = world->nodes[node].sw.first;
		edges[6] = world->nodes[node].w.first;
		edges[7] = world->nodes[node].nw.first;
		int next_nodes[8];
		next_nodes[0] = world->nodes[node].n.second;
		next_nodes[1] = world->nodes[node].ne.second;
		next_nodes[2] = world->nodes[node].e.second;
		next_nodes[3] = world->nodes[node].se.second;
		next_nodes[4] = world->nodes[node].s.second;
		next_nodes[5] = world->nodes[node].sw.second;
		next_nodes[6] = world->nodes[node].w.second;
		next_nodes[7] = world->nodes[node].nw.second;
		cout << "node,value,yaw,insideBlob: " << node << ", " << world->nodes[node].value << ", " << bot_yaw << ", " << insideConvexBlob(node, 1) << endl;
		for (int dir = 0; dir < 8; dir++)
		{
			int edge_cost = edges[dir];
			int next_node = next_nodes[dir];
			int next_value = world->nodes[next_node].value;
			cout << "\tdir,edge,next_value,insideBlob: " << dir << ", " << edge_cost << ", " << next_value << ", " << insideConvexBlob(next_node, 1) << endl;
		}
	}
};


// used by planner to issues commands into the main while loop
class command{
	public:
		double angularSpeed;
		double linearSpeed;
		double duration;
		double start_time;
		bool turn_avoid; //true if the last command was a turn to get away from a wall
		int type; //basically a flag for if the request command is critical for avoidance



	command(double ang, double lin, double dur, bool avoid, int command_type){
		this->duration = dur;
		this->angularSpeed = ang;
		this->linearSpeed = lin;
		this->start_time = ros::Time::now().toSec();
		this->turn_avoid = avoid;
		this->type = command_type;
	}

	bool operator == (const command& cmd_rhs)
	{
		if (angularSpeed != cmd_rhs.angularSpeed) {return false;}
		if (linearSpeed != cmd_rhs.linearSpeed) {return false;}
		//if (duration != cmd_rhs.duration) {return false;}
		if (turn_avoid != cmd_rhs.turn_avoid) {return false;}
		if (type != cmd_rhs.type) {return false;}
		return true;
	}
	
};


vector<command> command_history;
int command_history_size = 10;

void printCommandHistory()
{
	cout << "\tmy_cmd_history: ";
	for (int i = 0; i < command_history.size(); i++)
	{
		cout << command_history[i].type;
	}
	cout << endl;
}

void insertCommand(command cmd)
{
	command_history.insert(command_history.begin(), cmd);
	if ((int)command_history.size() > command_history_size)
	{
		//command oldest_command = command_history.back();
		command_history.pop_back();
	}
	printCommandHistory();
}




class bumperStates{
	// 0 is left      1 is center     2 is right
	public:
		bool left;
		bool center;
		bool right;

	void bumperCallback(const kobuki_msgs::BumperEvent msg){
		if(msg.bumper == 0){
			this->left = !this->left;
		}
		else if(msg.bumper == 1){
			this->center = !this->center;
		}
		else if(msg.bumper == 2){
			this->right = !this->right;
		}
	}

	bool isBumped(){
		return left || center || right;
	}
};







// randomly pick either 1 or -1
int randomSign(){
	return 2*(rand()%2)-1;
}







const int laserSize = 639;

const int angleSectors = 5; // try to divide cone into 3 approximately equal sized angle sectors
double sectorRange[angleSectors];
int sectorIndicies[angleSectors];
int del_sector = laserSize/angleSectors;
// Increasing indicies means going from right to left
// if looking at it from a top down view

double sectorWeights[angleSectors];




void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//fill with your


	// return the average laser range in each sector
	for(int i=0; i<angleSectors; i++){
		double sum = 0;
		for(int j=sectorIndicies[i]; j<sectorIndicies[i]+del_sector; j++){

			if(!isnan(msg->ranges[j])){
				sum = sum + msg->ranges[j];
			}
		}
		sectorRange[i] = sum/del_sector;
	}


}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	bot_x = msg->pose.pose.position.x;
	bot_y = msg->pose.pose.position.y;
	bot_yaw = tf::getYaw(msg->pose.pose.orientation);
	//cout << bot_x << ", " << bot_y << endl; // We expect values in meters [-8,8]
}







// simple plan based on sector ranges
double turnState[angleSectors];

// below wallThreshold is critical, move away
// beyond exploreThreshold means that that side is pretty open, take on an exploratory behaviour

command avoidancePlanner(double wallThreshold, command prevCommand, bumperStates bump, GridMap* exploration_map){


	command plannedCommand = command(0,0,0,false,0);




	// stats about the sectors
	double max_range = 0; //can see how far one way is one next obstacle
	double min_range = sectorRange[1]; //can use to see if atleast one section is at a wall
	double avgRange = 0;  //average range value

	double turnNumber = 0;


	for(int i=0; i<angleSectors; i++){
		if(sectorRange[i]<wallThreshold){

			//set to negative number if near a wall
			turnState[i] = -10;

		}
		else{
			turnState[i] = sectorRange[i];
		}

		if(sectorRange[i] < min_range){ min_range = sectorRange[i]; }
		if(sectorRange[i] > max_range){ max_range = sectorRange[i]; }

		avgRange = avgRange + sectorRange[i];
	}

	avgRange = avgRange / angleSectors;






/*------------------------------------------- Check conditions -------------------------------------------- */


	//check bumper first since its the most critical
	if(bump.isBumped()){

		//turn_avoid must be false to prevent
		//just running back into the bumper
		//back up at max speed for 0.35 seconds
		plannedCommand = command(0, -max_lin_speed, 0.25/max_lin_speed, false, 3);
		//printf("got bumped, backing up \n");
		goto endPlanning;
	}


	//if the last command was to backup due to bumper
	//turn a bit in a random direction
	if(prevCommand.type == 3){
		plannedCommand = command(randomSign() * max_ang_speed, 0, (pi/3)/max_ang_speed, false, 0);
		goto endPlanning;
	}


	// if last command was a turn to turn away from a wall
	if(prevCommand.turn_avoid == true){

		//go straight for a bit after turning away from the wall
		// cannot be overwritten by laser avoidance command
		// to avoid condition of getting stuck turning in a corner
		return command(0,0.2,0.1,false,0);
		goto endPlanning;

		//printf("going straight after turning from wall\n");
	}



	// this is true if the full 60 degree cone is within a wall - F
	// since this command has override, only do it if the previous command
	// was not a turn avoid command
	if(max_range < wallThreshold &&  prevCommand.turn_avoid == false){
		exploration_map->markNextAsUnaccessible(bot_x,bot_y,bot_yaw);
		//if left slightly more range than right, turn left90
		if(sectorRange[4]/sectorRange[0] > 1.2 ){
			plannedCommand = command(max_ang_speed, 0, (2*pi/3)/max_ang_speed, true, 2);
			goto endPlanning;
		}
		//if right slightly more range than left, turn right 90
		else if(sectorRange[4]/sectorRange[0] < 1/1.2){
			plannedCommand = command(-max_ang_speed, 0, (2*pi/3)/max_ang_speed, true, 2);
			goto endPlanning;
		}
		//if mostly straight on the wall, then turn in a random direction
		else{
			plannedCommand = command(randomSign()*max_ang_speed, 0, (2*pi/3)/max_ang_speed, true, 2);
			goto endPlanning;
		}

	} 



	//if right corner is near wall, but left corner not near wall
	if(sectorRange[0] < wallThreshold && sectorRange[4] > wallThreshold  &&  prevCommand.turn_avoid == false){
		
		if(sectorRange[1] < wallThreshold){
			exploration_map -> markNextAsUnaccessible(bot_x, bot_y, bot_yaw - 18*(pi/180));
			// if right also near wall, then left turn by more, say ~45 degrees
			plannedCommand = command(max_ang_speed, 0, (pi/4)/max_ang_speed, true, 2);
			goto endPlanning;
		}
		else{
			exploration_map -> markNextAsUnaccessible(bot_x, bot_y, bot_yaw - 24*(pi/180));
			// if only right corner near wall, then just turn left by less, ~ 22 degrees
			plannedCommand = command(max_ang_speed, 0 ,(pi/8)/max_ang_speed, true, 2);
			goto endPlanning;
		}
	}

	//if left corner is near wall, but right corner not near wall
	if(sectorRange[4] < wallThreshold && sectorRange[0] > wallThreshold  &&  prevCommand.turn_avoid == false){
		
		if(sectorRange[3] < wallThreshold){
			exploration_map -> markNextAsUnaccessible(bot_x, bot_y, bot_yaw + 18*(pi/180));
			// if left also near wall, then right turn by more, say ~45 degrees
			plannedCommand = command(-max_ang_speed, 0, (pi/4)/max_ang_speed, true, 2);
			goto endPlanning;			
		}
		else{
			exploration_map -> markNextAsUnaccessible(bot_x, bot_y, bot_yaw + 24*(pi/180));
			// if only right corner near wall, then just turn right by less, ~ 22 degrees
			plannedCommand = command(-max_ang_speed, 0 ,(pi/8)/max_ang_speed, true, 2);
			goto endPlanning;
		}
	}


	//------------------------------------------------------------------------------------------------------------//

	//relative measure of which side has more range
	//more positive -> right side comparatively more range than left
	//more negative -> left side comparatively more range than right
	// ~0 -> both sides about the same


	for(int i=0; i<angleSectors; i++){ turnNumber = turnNumber + turnState[i]*sectorWeights[i]; }
	turnNumber = -1*turnNumber / angleSectors; //switch sign of turnNumber around
	if(turnNumber > 5){ turnNumber = 5; }
	if(turnNumber < -5){ turnNumber = -5; }

	//explore via turnNumber
	plannedCommand = command((turnNumber/5)*1*max_ang_speed, max_lin_speed, 0, false, 0);


	endPlanning:


	//plannedCommand =  command(randomSign()*max_ang_speed, 0, (pi/4)/max_ang_speed, true, false);


	return plannedCommand;
}



// current command MUST be passed by REFERENCE

void highLevelPlanner(command newAvoidanceCommand, command& currentCommand, double timeNow, double timeBetweenSpins, double& timeLastSpin, GridMap* exploration_map){



	// speed of 360
	double spinSpeed = 1*max_ang_speed;

	int curr_node = exploration_map->getCurrNode();


	if(timeNow - currentCommand.start_time > currentCommand.duration && currentCommand.type == 4){
		currentCommand = command(0, max_lin_speed, 0.6/max_lin_speed, false, 0);
		goto endHighLevelPlanning;
	}


	// if the avoidance says we're in danger, do what we always do
	// but if we're already doing a 360 then just go ahead, unless we get a bumper avoidance
	if(newAvoidanceCommand.type != 0 && (currentCommand.type != 1 || currentCommand.type != 4)){
		if(timeNow - currentCommand.start_time > currentCommand.duration || newAvoidanceCommand.type > currentCommand.type){

			currentCommand = newAvoidanceCommand;
			insertCommand(currentCommand);		// update command history
			exploration_map -> print_node(exploration_map->getNode(bot_x,bot_y));
			printf("Doing new avoidance command - start time: %f - turn avoid? %i \n",currentCommand.start_time,currentCommand.turn_avoid);
			printf("angular: %f   linear: %f \n",currentCommand.angularSpeed, currentCommand.linearSpeed);
			printf("command type: %i \n\n ", currentCommand.type);

			goto endHighLevelPlanning;

		}
	}

	// should override spin if we hit a bumper
	else if(newAvoidanceCommand.type == 3 && currentCommand.type == 1){
		currentCommand = newAvoidanceCommand;
		insertCommand(currentCommand);		// update command history
		printf("Hit bumper during spin, performing bumper avoidance manuever \n\n");

		goto endHighLevelPlanning;
	}



	// since we're past the avoidance conditions check the explore conditions



	// is it about time for us to spin again?
    if(timeNow - timeLastSpin > timeBetweenSpins && exploration_map->getNodeHas360(curr_node)==0) {
    	exploration_map->markNodeHas360(curr_node);
    	currentCommand = command(spinSpeed, 0 , 2*pi/spinSpeed, false, 1);
    	insertCommand(currentCommand);		// update command history
    	printf("time to do a do a xbox 360 \n\n");

    	timeLastSpin = ros::Time::now().toSec() + 2*pi/spinSpeed;

    	goto endHighLevelPlanning;
    } else if (timeNow - timeLastSpin > timeBetweenSpins && exploration_map->getNodeHas360(curr_node)!=0) {
    	cout << "Tried to do a 360 but we have done it here already\n\n";
    	timeLastSpin = ros::Time::now().toSec() + 2*pi/spinSpeed;
    }


    // if we don't have to do a spin what to do?

    if(timeNow - currentCommand.start_time > currentCommand.duration){

/*    	printf("Doing new exploration command - start time: %f - turn avoid? %i \n",currentCommand.start_time,currentCommand.turn_avoid);
		printf("angular: %f   linear: %f \n",currentCommand.angularSpeed, currentCommand.linearSpeed);
		printf("command type: %i \n\n ", currentCommand.type);   */
		// For my location, look immediately to my neighbours to see if anyone of them are unexplored.
		int curr_node = exploration_map->getNode(bot_x,bot_y);
		int cardinal_direction = exploration_map->getMostUnexploredDirection(curr_node);
		if (cardinal_direction != -1 && exploration_map->getVisitAccumulation() > 0.8 && exploration_map->getNodeHasWiggle(curr_node)==0)
		{
			double target_yaw = exploration_map->getTargetYaw(cardinal_direction);
			double yawTurnAmount = target_yaw - bot_yaw;
			int sgnTurnAmount = yawTurnAmount/abs(yawTurnAmount);

			if(abs(yawTurnAmount) > pi)
			{
				yawTurnAmount = 2*pi - abs(yawTurnAmount);
				sgnTurnAmount = -1*sgnTurnAmount;
			}

			if (abs(yawTurnAmount) > 3*pi/4)
			{
				cout<< "Abort exploration turn! exploration turn too large.." << endl;
				currentCommand = newAvoidanceCommand;
				goto endHighLevelPlanning;
			}

			currentCommand = command(sgnTurnAmount*max_ang_speed, 0, abs(yawTurnAmount)/max_ang_speed, false, 4);
//---------------------------------------------------------------------------------//
			//prevent 360s while exploring
			timeLastSpin = ros::Time::now().toSec() + 2*pi/spinSpeed;

//---------------------------------------------------------------------------------//

			insertCommand(currentCommand);		// update command history
			if (command_history.size()>4)
			{
				cout << "checking for wiggle" << endl;
				// cout << "\t(0,2)(1,3)= "<< (command_history[0] == command_history[2]) << (command_history[1] == command_history[3]) <<endl;
				// cout << "\tAt least 2 type 4's " << (command_history[0].type == 4) << (command_history[1].type == 4) << endl;
				if (command_history[0] == command_history[2] && command_history[1] == command_history[3] &&
					(command_history[0].type == 4 || command_history[1].type == 4))
				{
					cout<< "Abort exploration turn! Use avoidanceCommand.." << endl;
					exploration_map->markNodeHasWiggle(curr_node);
					//exploration_map->resetAccumulation();
					currentCommand = newAvoidanceCommand;
					goto endHighLevelPlanning;
				}
			}
			printf("starting exploration turn \n");
			cout<< "\t@ " << cardinal_direction << endl;

			goto endHighLevelPlanning;
		} else {
			// use turn number
			currentCommand = newAvoidanceCommand;
		}

	}



    endHighLevelPlanning:
    ; //null statement to close the function after the label

}










int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;


	// laser subscriber
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);


	// bumper subscriber
	bumperStates bumper;
	bumper.left = false; bumper.center = false; bumper.right = false;
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperStates::bumperCallback, &bumper);


	// odom position subscriber
	ros::Subscriber odom = nh.subscribe("/odom", 1, &odomCallback);







	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;



	Timer timer = Timer();

	GridMap exploration_map = GridMap(8.0,4.0,0.5);
	int curr, prev = -1;



	// for getting indices of scan sectors
	for(int s=0; s < angleSectors; s++){
		sectorIndicies[s] = del_sector * s;
	}

	// setting up sectorWeights
	for(int s=0; s < angleSectors; s++){
		sectorWeights[s] = (angleSectors - s) - ceil((double)angleSectors/2);
	}







	command currCommand = command(0,0,0,false,0);

	//time stamps for 360
	double betweenSpin = 10;
	double lastSpin = 0;


	while(ros::ok() && timer.getElapsedTime() < 480){

		ros::spinOnce();
		curr = exploration_map.getNode(bot_x,bot_y);
		if (curr != prev)
		{
			cout << "visited: " << exploration_map.world->nodes[curr].value << endl;
		}
		prev = curr;
		exploration_map.visitLocation(bot_x,bot_y);

		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................
		//fill with your code




		// threshold parameters for planner
		double wallThresh = 0.4;

		// The heirachical controller
		command avoidanceCommand = avoidancePlanner(wallThresh, currCommand, bumper, &exploration_map);
		highLevelPlanner(avoidanceCommand, currCommand, ros::Time::now().toSec(), betweenSpin, lastSpin, &exploration_map);

  		vel.angular.z = currCommand.angularSpeed;
  		vel.linear.x = currCommand.linearSpeed;

  		vel_pub.publish(vel);


  		//ROS_INFO("X:%f Y:%f Yaw:%f", currPosition.x, currPosition.y,currPosition.yaw);
  		//ROS_INFO("LL:%f   L: %f   C:%f   R:%f  RR:%f   TN:%f",sectorRange[4],sectorRange[3], sectorRange[2],sectorRange[1],sectorRange[0],tempTurnNumber);

  		//ROS_INFO("left: %i   center: %i   right: %i   isbumped: %i", bumper.left,bumper.center,bumper.right,bumper.isBumped());

	}


	printf("its done bruv \n");



	//automatically save the map

/*	time_t now = time(0);
	tm *ltm = localtime(&now);

	string year = to_string(1900 + ltm->tm_year);
    string month = to_string(1+ ltm->tm_mon);
	string day = to_string(ltm->tm_mday);
	string hour = to_string(ltm->tm_hour);
	string minute = to_string(ltm->tm_min);


	string directory_name = "~/ros_maps/" + year + "_" + month + "_" + day + "_" + hour + "_" + minute;
	string make_folder = "mkdir " + directory_name;
	system(make_folder.c_str());
	string save_map = "rosrun map_server map_saver -f " + directory_name + "/map";
	system(save_map.c_str());
*/





	return 0;
}









//to do

// 360 turn on time more robust



// bumper callback
// just get bumper state

/*can get the bumper state in an object, also has a function to check if any bumper
are currently bumped, should pass bumper into avoidance planner*/




//check for turning left then turning right if in corner




/*----------- change log --------------- */

/*changed turning for big turn to 120 degrees, to help turn out of corners
*/













//stuff here



/*float obstacleVisionCone(vector<float> scanArray, int laserSize, double obstacleThreshold){


	float sum = 0;
	for(int i = 0; i < laserSize; i++){

		if(isnan(scanArray[i])){
			printf("nan at index %i \n",i);
		}
		else{
			sum = sum + scanArray[i];
			printf("%f \n",sum);
			printf("%f \n",scanArray[i]);
		}
	}

	return sum/laserranges;


	int * obstacleArray;

}*/
