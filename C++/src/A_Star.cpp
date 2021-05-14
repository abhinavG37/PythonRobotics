//A* Grid Planning C++ Variant 
//Uses Sciplot plotting as a substitute for matplotlib
//Eigen used for Linear algebra 
//vector<float/int> used for grid formation and update
//map<int, Node*> used for maintaining traversed nodes
#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <eigen3/Eigen/Dense>
#include <sciplot/sciplot.hpp>
using namespace std;

class A_Star_planner{
public:
   A_Star_planner(int rows_, int cols_, float resolution_, vector<float> ox_, vector<float> oy_ , float rr_){
        vector<vector<float>> grid_(rows_,vector<float>(cols_,0));
        grid = grid_;
        resolution = resolution_;
        rr = rr_;
        ox = ox_;
        oy = oy_;
        this->min_x=0.0, this->min_y = 0.0;
        this->max_x=0.0, this->max_y = 0.0;
        this->x_width = 0.0; this->y_width = 0.0;
        this->motion = this->get_motion_model();
        this->obmap =this->calc_obmap(ox, oy);
    } //CONSTRUCTOR FOR CLASS A*_Planner

    ////////////////////////CLASS NODE//////////////////////////////////////////////////
     class Node{
        public:
            Node(float x, float y, float cost, int parent_index, Node* p_node_=NULL){
                this->x = x;
                this->y = y;
                this->cost = cost;
                this->parent_index = parent_index;
                parent_node = p_node_;
            } //DEFAULT CONSTRUCTOR TO INITIALIZE NODE BASED ON PARENT NODE
            
            string str_maker(){
                return to_string(this->x)+ "," + to_string(this->y) + "," + to_string(
                this->cost) + "," +to_string(this->parent_index);
            }
        //Data members of subclass Node
        float x, y, cost;
        float  parent_index;   
        Node* parent_node;
    };
    //////////////////////////////////////////END CLASS NODE//////////////////////////////////
 

    vector<vector<float>> planner(float sx, float sy, float gx, float gy){
      /*  A star path search

        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
    */
        Node* startNode =  new Node(this->calc_xy_index(sx, this->min_x), this->calc_xy_index(sy, this->min_y), 0.0, -1);
        Node* goalNode =  new Node(this->calc_xy_index(gx, this->min_x), this->calc_xy_index(gy, this->min_y), 0.0, -1);
        //pointer to an object created 
        printf("START----------NODE_X: %f\tNODE_Y:%f\tNODE_COST:%f\tPARENT_INDEX:%f", startNode->x, startNode->y, startNode->cost, startNode->parent_index);
        printf("GOAL----------NODE_X: %f\tNODE_Y:%f\tNODE_COST:%f\tPARENT_INDEX:%f", goalNode->x, goalNode->y, goalNode->cost, goalNode->parent_index);
        
        map<int, Node*> open_set;
        map<int, Node*> closed_set;

// {  TAKES A pointer to NODE object}     //calc_grid_index(Node* node)
         open_set.insert(pair<int, Node*>(this->calc_grid_index(startNode), startNode));
        while(true){
            if(open_set.size() == 0){
                printf("OPEN SET EMPTY");
                break;
            }
            
            //LINE 81: Needs a lambda function equivalent ---C++ functor---- to capture max cost Node in open_set  
            map<int, Node*>::iterator it;
            float temp_min_cost = 0; 
            int c_id= 0; //grid index of current node used as key in open_set 
            
            //BLOCK FUNCTIONING NOT PROPER
            for(it = open_set.begin(); it != open_set.end(); it++){
                temp_min_cost = (it->second->cost)+this->calc_heuristic(goalNode, it->second);
                if( (it->second->cost)+this->calc_heuristic(goalNode, it->second) <= temp_min_cost){
                    temp_min_cost =(it->second->cost)+this->calc_heuristic(goalNode, it->second);
                    c_id = it->first;
                }
            } //find key corresponding to a temp_min_cost
            Node* current = open_set.at(c_id); //ERROR

            if(current->x  == goalNode->x && current->y == goalNode->y){
                printf("-------------Goal Found-------------");
                goalNode->parent_index = current->parent_index;
                goalNode->cost = current->cost;    
            }
            
            open_set.erase(c_id); //Deletes pair at key value c_id
            open_set.insert(pair<int, Node*>(c_id, current));  // add traversed node to closed set
            
        
            /////////continue here from line 110
            for (unsigned int i =0; i<motion.size(); i++){
                Node* node = new Node(current->x+motion[i][0],
                                                        current->y+motion[i][1],
                                                        current->cost+motion[i][2], 
                                                        c_id);
                int n_id = this->calc_grid_index(node);

                //Verification of new node
                //Node not out of bounds or collding with obstacle
                if(!this->verify_node(node)){ continue; }
                //Node exists on already traversed node then do nothin
                if(closed_set.count(n_id)>0){ continue; }

                if(open_set.count(n_id)==0){ open_set.insert(pair<int, Node*>(n_id, node)); }
                else{
                    if(open_set.at(n_id)->cost > node->cost){open_set.insert(pair<int, Node*>(n_id, node)); }
                }
            }
        }
        return this->calc_final_path(goalNode, closed_set);
    }


    vector<vector<int>> calc_obmap(vector<float> ox, vector<float> oy){
        
        this->min_x = round(*std::min_element(ox.begin(), ox.end()));
        this->max_x = round(*std::max_element(ox.begin(), ox.end()));
        this->min_y = round(*std::min_element(oy.begin(), oy.end()));
        this->max_y = round(*std::max_element(oy.begin(), oy.end()));
    
        printf("min_x: %f", this->min_x);
        printf("min_y: %f", this->min_y);
        printf("max_x: %f", this->max_x);
        printf("max_y: %f", this->max_y);

        this->x_width = round((this->max_x - this->min_x) / this->resolution);
        this->y_width = round((this->max_y - this->min_y) / this->resolution);
        printf("x_width: %f", this->x_width);
        printf("y_width: %f", this->y_width);       
        
        vector<vector<int>> temp_obmap(this->y_width, vector<int>(this->x_width, 0));
        for(int  i =0; i<this->x_width; i++){
            int x = this->calc_grid_pos(i, this->min_x);
            for(int j = 0; j<this->y_width; j++){
                int y = this->calc_grid_pos(j, this->min_y);
                for(unsigned int k = 0; k<ox.size(); k++){
                    float d = sqrt(pow((ox[k]-x), 2)+pow((oy[k]-y), 2));
                    if(d<=this->rr){
                        temp_obmap[i][j] = 1; 
                        break;
                    }
                }
            }
        }
        obmap = temp_obmap;
        
        return obmap;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    vector<vector<float>> calc_final_path(Node* goal, map<int, Node*> closed_set){
        vector<float> rx;
        vector<float> ry;
        rx.push_back(this->calc_grid_pos(goal->x, this->min_x));
        ry.push_back(this->calc_grid_pos(goal->y, this->min_y));
        int parent_index = goal->parent_index;
        while(parent_index != -1){
            Node* n = closed_set.at(parent_index);
            rx.push_back(this->calc_grid_pos(n->x, this->min_x));
            ry.push_back(this->calc_grid_pos(n->y, this->min_y));
            parent_index = n->parent_index;
        }        

  return {rx, ry};
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
   inline  float calc_grid_pos(int index,float min_pos){
       return  index * this->resolution+min_pos;
   }
      ///////////////////////////////////////////////////////////////////////////////////////////////////// 
    inline int calc_xy_index( float position, float min_pos){
        return round((position - min_pos) / this->resolution);
    }
   /////////////////////////////////////////////////////////////////////////////////////////////////////
    inline int calc_grid_index(Node* node){ 
        return ((node->y-this->min_y)*this->x_width)+(node->x-this->min_x);
    }//calculates delX+delY between min pos of grid and current pos of node
   ///////////////////////////////////////////////////////////////////////////////////////////////////// 

    float calc_heuristic(Node* n1, Node* n2){
        float w = 1.0;
        float dist = w*sqrt(pow((n1->x-n2->x),2) + pow( (n1->y-n2->y),2));
        return dist;    
   }
   ///////////////////////////////////////////////////////////////////////////////////////////////////// 
    bool verify_node(Node* node){
        float px  = this->calc_grid_pos(node->x, this->min_x);
        float py = this->calc_grid_pos(node->y, this->min_y);
         if (px < this->min_x || py < this->min_y || 
                px >= this->max_x || py >= max_y){
                    return false;
                } //Out of bounds grid check
       
       if (this->obmap[node->x][node->y ] == true){
           return false;
       } //if node resides on obstacle map--> collision==TRUE

        return true;
    }
///////////////////////////////////////////////////////////////////////////////////////////
    vector<vector<float>> get_motion_model(){
                                                               
         vector<vector<float>> motion = {//{dx   dy    cost}
                                                                {1.0, 0.0, 1.0},
                                                                {0.0, 1.0, 1.0},
                                                                {-1.0, 0.0, 1.0},
                                                                {0, -1.0, 1.0},
                                                                {-1.0, -1.0, float(sqrt(2))},
                                                                {-1.0, 1.0, float(sqrt(2))},
                                                                {1.0, -1.0, float(sqrt(2))},
                                                                {1.0, 1.0,  float(sqrt(2))}
                                                                };

    return motion;
}
    
/////////////DATA MEMBERS OF PARENT CLASS///////////////////////////////////////ap
    vector<vector<float>> grid;
    vector<vector<int>> obmap;
    float resolution, rr, min_x, min_y, max_x, max_y, x_width, y_width;
    vector<vector<float>> motion;
    vector<float> ox, oy;
//////////////////END OF A_STAR_PLANNER CLASS/////////////////
};


////////////////////////////////MAIN//////////////////////////////////////////////////////////////
int main(int argc, char* argv[]){
   printf("Name = %s \n\t\t______START__________\n\n", argv[0]);

    //////////////////VARIABLE DECLARATION//////////////////////////
    
    //Start and goal pos declaration
    float sx = 10.0;               // [m]
    float sy = 10.0;                 // [m]
    float gx = 50.0;                 // [m]
    float gy = 50.0;                 // [m]
    //Grid properties
    int grid_rows = 10;
    int grid_cols  = 10;
    float grid_resolution = 2.0;        //[m]
    //Robot Properties
    float robot_radius     = 1.0;   //[m]
    
    //Obstacle list Intialization
    vector<float> ox_;
    vector<float> oy_;
    // add edges
    for(float i=-10.0; i<60.0; i++){
        ox_.push_back(i);
        oy_.push_back(-10.0);
    }
    for(float i=-10.0; i<60.0; i++){
        ox_.push_back(60.0);
        oy_.push_back(i);
    }
    for(float i=-10.0; i<61; i++){
        ox_.push_back(i);
        oy_.push_back(60.0);
    }
    for(float i=-10.0; i<61; i++){
        ox_.push_back(-10.0);
        oy_.push_back(i);
    }
    for(float i=-10.0; i<40; i++){
        ox_.push_back(20.0);
        oy_.push_back(i);
    }
    for(float i=0; i<40; i++){
        ox_.push_back(40.0);
        oy_.push_back(60.0 - i);
    }

    //Calling constructor for A_Star_planner Class 
    A_Star_planner planner_obj(grid_rows, grid_cols, grid_resolution, ox_, oy_, robot_radius);
    //Calling planner function
    vector<vector<float>> rxry = planner_obj.planner(sx, sy, gx,gy);
    cout<<"CODE FINISHED"<<endl;

}
