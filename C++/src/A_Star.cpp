//A* Grid Planning C++ Variant 
//Uses Sciplot plotting as a substitute for matplotlib
//Eigen used for Linear algebra 
//vector<float/int> used for grid formation and update
//map<int, Node*> used for maintaining traversed nodes
/*Major Chunk working now------------VISUALIZATION PENDING*/

#define watch(x) cout << "Node formed: " << (#x) << ":\t"

#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <map>
#include <string>
#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

class A_Star_planner {
public:
  A_Star_planner(float resolution_, vector<float> ox_, vector<float> oy_, float rr_){

    resolution = resolution_;
    rr = rr_;
    ox = std::move(ox_);
    oy = std::move(oy_);
    this->min_x = 0.0, this->min_y = 0.0;
    this->max_x = 0.0, this->max_y = 0.0;
    this->x_width = 0.0;
    this->y_width = 0.0;
    this->motion = get_motion_model();
    this->obmap = this->calc_obmap(ox, oy);
  } // CONSTRUCTOR FOR CLASS A*_Planner

  ////////////////////////CLASS NODE//////////////////////////////////////////////////
  class Node {
  public:
    Node(float x, float y, float cost, int parent_index,
         Node *p_node_ = nullptr) {
      this->x = x;
      this->y = y;
      this->cost = cost;
      this->parent_index = parent_index;
      parent_node = p_node_;
    }

    [[nodiscard]] string str_maker() const {
      return "X: "+to_string(this->x) + "," +"Y: "+ to_string(this->y) + "," +"COST: "+
             to_string(this->cost) + "," + "PARENT_INDEX: "+to_string(this->parent_index);
    }
    // Data members of subclass Node
    float x, y, cost;
    float parent_index;
    [[maybe_unused]] Node *parent_node;
  };
  //////////////////////////////////////////END CLASS NODE//////////////////////////////////

  vector<vector<float>> planner(float sx, float sy, float gx, float gy) {
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

    Node *startNode = new Node(this->calc_xy_index(sx, this->min_x),
                               this->calc_xy_index(sy, this->min_y), 0.0, -1);
    watch(startNode);
    cout<<startNode->str_maker()<<endl;
    Node *goalNode = new Node(this->calc_xy_index(gx, this->min_x),
                              this->calc_xy_index(gy, this->min_y), 0.0, -1);
    watch(goalNode);
    cout<<goalNode->str_maker()<<endl;

    map<int, Node *> open_set;
    map<int, Node *> closed_set;
    map<int, Node *>::iterator it;

    // {  TAKES A pointer to NODE object}     //calc_grid_index(Node* node)
    open_set.insert(pair<int, Node *>(this->calc_grid_index(startNode), startNode));
    int count = 0;
    int c_id = 0; // grid index of current node used as key in open_set

    while (true) {
      count += 1;
      printf("%d\n", count);
      if (open_set.empty()) {
        printf("OPEN SET EMPTY");
        break;
      }

      auto temp_min_cost = INFINITY;
      for (it = open_set.begin(); it != open_set.end(); it++) {
        if ((it->second->cost) + A_Star_planner::calc_heuristic(goalNode, it->second)<=temp_min_cost) {
          temp_min_cost = (it->second->cost) + A_Star_planner::calc_heuristic(goalNode, it->second);
          c_id = it->first;
        }
      }

      cv::circle(bg, cv::Point(this->calc_grid_pos(startNode->x, this->min_x),
                               this->calc_grid_pos(startNode->y, this->min_y)),
                 this->rr,cv::Scalar(255,0,0), cv::FILLED);      //  START NODE

      cv::circle(bg, cv::Point(this->calc_grid_pos(goalNode->x, this->min_x),
                               this->calc_grid_pos(goalNode->y, this->min_y)),
                 this->rr,cv::Scalar(0,0,255), cv::FILLED);      //  GOAL NODE

      imshow("astar", bg);

      // find key corresponding to a temp_min_cost
      Node *current = open_set.at(c_id);
      if (current->x == goalNode->x && current->y == goalNode->y) {
        printf("-------------Goal Found-------------");
        goalNode->parent_index = current->parent_index;
        goalNode->cost = current->cost;
        break;
      }
      open_set.erase(c_id); // Deletes pair at key value c_id
      closed_set.insert(pair<int, Node*>(c_id, current)); // add traversed node to closed set

      /////////continue here from line 110
      for (auto &i : motion) {
        Node *node = new Node(current->x + i[0], current->y + i[1],current->cost + i[2], c_id);
        int n_id = this->calc_grid_index(node);

       //NODE VERIFICATION
        if (!this->verify_node(node))    {continue;}
        if (closed_set.count(n_id) > 0)  {continue;}
        if (open_set.count(n_id) == 0)   {open_set.insert(pair<int, Node *>(n_id, node));}
        else {if (open_set.at(n_id)->cost > node->cost) {open_set.insert(pair<int, Node *>(n_id, node));}}

        cv::circle(bg,
                   cv::Point(this->calc_grid_pos(node->x, this->min_x),this->calc_grid_pos(node->y, this->min_y)),
                   this->rr,cv::Scalar(255,0,0), cv::FILLED);      //  START NODE
        cv::imshow("astar", bg);
        cv::waitKey(1);
      }
    }
    return this->calc_final_path(goalNode, closed_set);
  }

  vector<vector<int>> calc_obmap(vector<float> ox, vector<float> oy) {

    this->min_x = round(*std::min_element(ox.begin(), ox.end()));
    this->max_x = round(*std::max_element(ox.begin(), ox.end()));
    this->min_y = round(*std::min_element(oy.begin(), oy.end()));
    this->max_y = round(*std::max_element(oy.begin(), oy.end()));
    this->x_width = round((this->max_x - this->min_x) / this->resolution);
    this->y_width = round((this->max_y - this->min_y) / this->resolution);

    printf("min_x: %f\t", this->min_x);
    printf("min_y: %f\t", this->min_y);
    printf("max_x: %f\t", this->max_x);
    printf("max_y: %f\t", this->max_y);
    printf("x_width: %f\t", this->x_width);
    printf("y_width: %f\t\n", this->y_width);

    this->bg = cv::Mat(this->resolution * (this->y_width+1),this->resolution * (this->x_width+1), CV_8UC3, cv::Scalar(255, 255, 255));
    vector<vector<int>> temp_obmap((int)this->y_width, vector<int>((int)this->x_width, 0));

    for (int ix = 0; ix < this->x_width; ix++) {
      int x = this->calc_grid_pos(ix, this->min_x);
      for (int iy = 0; iy < this->y_width; iy++) {
        int y = this->calc_grid_pos(iy, this->min_y);
        if(this->x_width <= this-> y_width) {
          for (unsigned int k = 0; k < ox.size(); k++) {
            float d = sqrt(pow((ox[k] - x), 2) + pow((oy[k] - y), 2));
            if (d < this->rr) {
              temp_obmap[ix][iy] = 1;
              cv::circle(bg, cv::Point(x, y), 5, cv::Scalar(0, 0, 0),cv::FILLED); //  START NODE
              cv::imshow("astar", this->bg);
              cv::waitKey(1);
              break;
            }
          }
        }
        else{
          for (unsigned int k = 0; k < oy.size(); k++) {
            float d = sqrt(pow((ox[k] - x), 2) + pow((oy[k] - y), 2));
            if (d < this->rr) {
              temp_obmap[ix][iy] = 1;
              cv::circle(bg, cv::Point(x, y), 5, cv::Scalar(0, 0, 0),cv::FILLED); //  START NODE
              cv::imshow("astar", this->bg);
              cv::waitKey(1);
              break;
            }
          }
        }
      }
    }
    obmap = temp_obmap;

    return obmap;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  vector<vector<float>> calc_final_path(Node *goal, map<int, Node *> closed_set) const {
    vector<float> rx;
    vector<float> ry;
    rx.push_back(this->calc_grid_pos(goal->x, this->min_x));
    ry.push_back(this->calc_grid_pos(goal->y, this->min_y));
    int parent_index = goal->parent_index;

    while (parent_index != -1) {
      Node *n = closed_set.at(parent_index);
      rx.push_back(this->calc_grid_pos(n->x, this->min_x));
      ry.push_back(this->calc_grid_pos(n->y, this->min_y));
      parent_index = n->parent_index;
    }
    return {rx, ry};
  }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
   [[nodiscard]] inline  float calc_grid_pos(int index,float min_pos) const{
       return  index * this->resolution+min_pos;
   }
      ///////////////////////////////////////////////////////////////////////////////////////////////////// 
    [[nodiscard]] inline int calc_xy_index( float position, float min_pos) const{
        return round((position - min_pos) / this->resolution);
    }
   /////////////////////////////////////////////////////////////////////////////////////////////////////
    inline int calc_grid_index(Node* node) const{
        return ((node->y-this->min_y)*this->x_width)+(node->x-this->min_x); //number of rows*columns+ number of extra columns to that row from left
        /*  000000000000000000
         *  111100000000000000        Row -> 2:  4 columns
         *  111111111111111111        Row->  1: 18 columns
         *  111111111111111111        Row->  0: 18 columns
         *  min-y = 0 min-x = 0 x width=18 y width=4
         *  Returns: 2-0*18 + 4-0 = 40 -> Grid index
         */
    }//calculates delX+delY between min pos of grid and current pos of node
   ///////////////////////////////////////////////////////////////////////////////////////////////////// 

    static float calc_heuristic(Node* n1, Node* n2){
        float w = 1.0;
        float dist = w*sqrt(pow((n1->x-n2->x),2) + pow( (n1->y-n2->y),2));
        return dist;    
   }
   ///////////////////////////////////////////////////////////////////////////////////////////////////// 
    bool verify_node(Node* node){
        float px  = this->calc_grid_pos(node->x, this->min_x);
        float py = this->calc_grid_pos(node->y, this->min_y);
         if (px < this->min_x || py < this->min_y || px >= this->max_x || py >= max_y){
            return false;
        } //Out of bounds grid check
       
       if (this->obmap[node->x][node->y] == true){
           return false;
        } //if node resides on obstacle map--> collision==TRUE

        return true;
    }
///////////////////////////////////////////////////////////////////////////////////////////
    static vector<vector<float>> get_motion_model(){
                                                               
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
    
/////////////DATA MEMBERS OF PARENT CLASS///////////////////////////////////////
    vector<vector<int>> obmap;
    float resolution, rr, min_x, min_y, max_x, max_y, x_width, y_width;
    vector<vector<float>> motion;
    vector<float> ox, oy;
    cv::Mat bg;
//////////////////END OF A_STAR_PLANNER CLASS/////////////////
};


////////////////////////////////MAIN//////////////////////////////////////////////////////////////
int main(int argc, char* argv[]){
  cv::namedWindow("astar", cv::WINDOW_AUTOSIZE);
  printf("Name = %s \n\t\t______START__________\n\n", argv[0]);

    ////////////////VARIABLE DECLARATION//////////////////////////
//     // Start and goal pos declaration
//     float sx = 50.0;  // [m]
//     float sy = 50.0;  // [m]
//     float gx = 500.0; // [m]
//     float gy = 500.0; // [m]
//     // Grid properties
//     float grid_resolution = 10.0; //[m]
//     // Robot Properties
//     float robot_radius = 5.0; //[m]
//
//     // Obstacle list Initialization
//     vector<float> ox_;
//     vector<float> oy_;
//     // add edges
//     for (float i = -100.0; i < 600.0; i++) {
//       ox_.push_back(i);
//       oy_.push_back(-100.0);
//     }
//     for (float i = -100.0; i < 500.0; i++) {
//       ox_.push_back(600.0);
//       oy_.push_back(i);
//     }
//     for (float i = -100.0; i < 610.0; i++) {
//       ox_.push_back(i);
//       oy_.push_back(600.0);
//     }
//     for (float i = -100.0; i < 510.0; i++) {
//       ox_.push_back(-100.0);
//       oy_.push_back(i);
//     }
//     for (float i = -100.0; i < 600; i++) {
//       ox_.push_back(0.0);
//       oy_.push_back(i);
//     }
//     for (float i = 0; i < 400; i++) {
//       ox_.push_back(400.0);
//       oy_.push_back(600.0 - i);
//     }
//   PARAMS VERSION 1

//  //Start and goal pos declaration
//  float sx = 10.0;               // [m]
//  float sy = 10.0;                 // [m]
//  float gx = 500.0;                 // [m]
//  float gy = 500.0;                 // [m]
//  //Grid properties
//  float grid_resolution = 10.0;        //[m]
//  //Robot Properties
//  float robot_radius     = 5.0;   //[m]
//  //Obstacle list Initialization
//  vector<float> ox_;
//  vector<float> oy_;
//  // add edges
//  for(float i=-100.0; i<600.0; i++){
//    ox_.push_back(i);
//    oy_.push_back(-100.0);
//  }
//  for(float i=-100.0; i<600.0; i++){
//    ox_.push_back(600.0);
//    oy_.push_back(i);
//  }
//  for(float i=-100.0; i<610.0; i++){
//    ox_.push_back(i);
//    oy_.push_back(600.0);
//  }
//  for(float i=-100.0; i<610.0; i++){
//    ox_.push_back(-100.0);
//    oy_.push_back(i);
//  }
//  for(float i=-100.0; i<400; i++){
//    ox_.push_back(200.0);
//    oy_.push_back(i);
//  }
//  for(float i=0; i<400; i++){
//    ox_.push_back(400.0);
//    oy_.push_back(600.0 - i);
//  }
//


    // Start and goal pos declaration
//     float sx = 10.0; // [m]
//     float sy = 10.0; // [m]
//     float gx = 50.0; // [m]
//     float gy = 50.0; // [m]
//     // Grid properties
//     float grid_resolution = 2.0; //[m]
//     // Robot Properties
//     float robot_radius = 1.0; //[m]
//
//     // Obstacle list Initialization
//     vector<float> ox_;
//     vector<float> oy_;
//     // add edges
//     for (float i = -10.0; i < 60.0; i++) {
//       ox_.push_back(i);
//       oy_.push_back(-10.0);
//     }
//     for (float i = -10.0; i < 60.0; i++) {
//       ox_.push_back(60.0);
//       oy_.push_back(i);
//     }
//     for (float i = -10.0; i < 61.0; i++) {
//       ox_.push_back(i);
//       oy_.push_back(60.0);
//     }
//     for (float i = -10.0; i < 61.0; i++) {
//       ox_.push_back(-10.0);
//       oy_.push_back(i);
//     }
//     for (float i = -10.0; i < 40; i++) {
//       ox_.push_back(20.0);
//       oy_.push_back(i);
//     }
//     for (float i = 0; i < 40; i++) {
//       ox_.push_back(40.0);
//       oy_.push_back(60.0 - i);
//     }
//   //PARAMS VERSION 2 ACTIVE

//
  // Start and goal pos declaration
  float sx = 20.0;  // [m]
  float sy = 20.0;  // [m]
  float gx = 500.0; // [m]
  float gy = 500.0; // [m]
  // Grid properties

  float grid_resolution = 5.0;
  float robot_radius = 2.50;

  // Obstacle list Initialization
  vector<float> ox_;
  vector<float> oy_;
  // add edges
  for (float i = 10.0; i < 550.0; i++) {
    oy_.push_back(10.0);
    ox_.push_back(i);
  } //Top border
  for (float i = 10.0; i < 551.0; i++) {
    oy_.push_back(550.0);
    ox_.push_back(i);
  } //Bottom border
  for (float i = 10.0; i < 600.0; i++) {
    oy_.push_back(i);
    ox_.push_back(10.0);
  } //Left border
  for (float i = 10.0; i < 601.0; i++) {
    oy_.push_back(i);
    ox_.push_back(601);
  } //Right border


  for (float i = 10.0; i < 150.0; i++) {
    oy_.push_back(50.0);
    ox_.push_back(i);
  } //Wall 1
  for (float i = 300.0; i < 500.0; i++) {
    oy_.push_back(i);
    ox_.push_back(150.0);
  } //Wall 2
  for (float i = 10.0; i < 400.0; i++) {
    oy_.push_back(i);
    ox_.push_back(200.0);
  } //Wall 3
  for (float i = 10.0; i < 150.0; i++) {
    oy_.push_back(i);
    ox_.push_back(400.0);
  } //Wall 4

    printf("____________PARAMS___________\n");
    printf("------Robot_Radius:%f\t-----Grid_Resolution:%f\n", robot_radius, grid_resolution);
    printf("---S_x: %f\t---S_y:%f\t---G_y:%f\t---G_y:%f\n", sx,sy,gx,gy);

    A_Star_planner planner_obj( grid_resolution, ox_, oy_, robot_radius);
    vector<vector<float>> rxry = planner_obj.planner(sx, sy, gx,gy);

    for(int i = rxry[0].size()-1; i>= 0; i--){
      cv::circle(planner_obj.bg, cv::Point(rxry[0][i], rxry[1][i]),planner_obj.rr, cv::Scalar(100,100,100), cv::FILLED);
      cv::imshow("astar", planner_obj.bg);
      cv::waitKey(1);
    }
  cv::waitKey(0);

//    if ((key & 0xFF) == 27) {
//      cv::destroyAllWindows();
//    }

  cout<<"CODE FINISHED"<<endl;

}
