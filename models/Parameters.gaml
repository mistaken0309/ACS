/**
* Name: Parameters
* Author: Annalisa Congiu
* Description: parameters and variables for the simulations and common agents
* Tags: Tag1, Tag2, TagN
*/

model Parameters
import "./../models/cenAV.gaml"
import "./../models/simulationAVmain.gaml"

/* List of parameters and global variables used in the simulations and common agents */

global{
	bool centralised_ON<-true; 
	file shape_file_roads  <- file("../includes/roads.shp") ;
	file shape_file_nodes  <- file("../includes/nodes.shp");
	file shape_file_buildings  <- file("../includes/buildings.shp");
	geometry shape <- envelope(shape_file_roads);
	
	graph the_graph;  
	graph road_network;  
	map road_weights;
	map graph_weights;
	list<cenAV> not_full_cars;
	bool not_reset<-true;
//	bool centralised_ON<-true;
	
	float step <- 5 #s;	
	
	int nb_people <- 500 ;
	int modstart <- ((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5);
	int nb_car <- 10 ;
	int nb_cars <-  10;
//	bool centralised_ON <- true parameter: "Centralised or Decentralised: " category: "Initial"; //0= centralised, 1 = decentralised
	
//	//represent the day time for the agent to inform them to go work or home
//	float current_hour update: ((time/#hour)-(g)*24);
//	int h update: (time/#hour) mod 24; 
//	int g update: int(time/#hour/24); 
	
	//Variables to manage the minimal and maximal time to start working/go home
	float min_work_start <- 0.5;
	float max_work_start <- 1.0;
	float min_work_end <- 17.0;
	float max_work_end <- 18.0;
	
	float before_work_search<-0.5; //(30 minuti)
	float before_work_start	<-0.25; // (15 minuti)
	float after_work_start<-0.5;
	
	//Manage the speed allowed in the model for the people agents
	float min_speed <- 5.0  #km / #h;
	float max_speed <- 20.0 #km / #h; 
	
	float cost_km <- 1.50;
	
	float v_length <- 5.0#m;
	
	float stats_grouping_time <-0.0;
	float stats_path_time <-0.0;
	int n_grouping;
	
	float start_simulation;
}