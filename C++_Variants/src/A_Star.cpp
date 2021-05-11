//A* Grid Planning C++ Variant 
//Uses Sciplot plotting as a substitute for matplotlib
//Eigen used for Linear algebra 

// STL Vectors used for grid formation and update
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
using namespace std;
#include <eigen3/Eigen/Dense>
#include <sciplot/sciplot.hpp>

class A_Star_planner{
public:
   A_Star_planner(int rows_, int cols_, float resolution_, vector<float> ox_, vector<float> oy_ , float rr_){
        vector<vector<float>> grid_(rows_,vector<float>(cols_,0));
        grid = grid_;
        resolution = resolution_;
        rr = rr_;
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
 

    void planner(float sx, float sy, float gx, float gy){
      Node startNode =  Node(1,2,3,4);
      Node goalNode = Node(1,2,3,4);
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
    }


   inline  float calc_grid_pos(int index,float min_pos){
       return  index * this->resolution+min_pos;
   }
    inline int calc_xy_index( float position, float min_pos){
        return round((position - min_pos) / this->resolution);
    }
    inline int calc_grid_index(Node* node){
        return ((node->y-this->min_y)*this->x_width)+(node->x-this->min_x);
    }
    
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

    vector<vector<int>> calc_obmap(vector<int> ox, vector<int> oy){
        
        this->min_x = round(*min_element(ox.begin(), ox.end()));
        this->max_x = round(*min_element(ox.begin(), ox.end()));
        this->min_y = round(*min_element(oy.begin(), oy.end()));
        this->max_y = round(*min_element(oy.begin(), oy.end()));
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
                for(int k = 0; k<ox.size(); k++){
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

    vector<vector<float>> get_motion_model(){
         vector<vector<float>> motion = {{1.0, 0.0, 1.0},
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

    

    float calc_heuristic(Node* n1, Node* n2){
        float w = 1.0;
        float dist = w*sqrt(pow((n1->x-n2->x),2) + pow( (n1->y-n2->y),2));
        return dist;    
   }

    
/////////////DATA MEMBERS OF PARENT CLASS///////////////////////////////////////ap
    vector<vector<float>> grid;
    vector<vector<int>> obmap;
    struct finalpose_List {
        vector<float> rx, ry;
    };     finalpose_List structObj;
    float resolution, rr, min_x, min_y, max_x, max_y, x_width, y_width;

//////////////////END OF A_STAR_PLANNER CLASS/////////////////
};



int main(int argc, char* argv[]){
   printf("Name = %s \n\t\t______START__________\n\n", argv[0]);

    //////////////////VARIABLE DECLARATION//////////////////////////
    //Start and goal pos declaration
    float sx = 10.0;               // [m]
    float sy = 10.0;                 // [m]
    float gx = 50.0;                 // [m]
    float gy = 50.0;                 // [m]
    float grid_size = 2.0;        //[m]
    float robot_radius = 1.0;   //[m]

    


}
