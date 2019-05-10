/**
* Name: simulationAVmain
* Author: Annalisa Congiu
* Description: simulation of a AV Carpooling System
* Tags: 
*/

model simulationAVmain

import "./../models/common_agents.gaml"
import "./../models/Parameters.gaml"
import "./../models/decAV.gaml"
import "./../models/cenAV.gaml"

/* *
 * Experiments to create the map and the agents for the Centralised or 
 * Decentralised  simulation of a Autonomous Vehicles Carpooling system
 * */

global {   
//	file shape_file_roads  <- file("../includes/roads.shp") ;
//	file shape_file_nodes  <- file("../includes/nodes.shp");
//	file shape_file_buildings  <- file("../includes/buildings.shp");
//	geometry shape <- envelope(shape_file_roads);
//	
//	graph the_graph;  
//	graph road_network;  
//	map road_weights;
//	map graph_weights;
//	list<cenAV> not_full_cars;
//	
//	float step <- 5 #s;	
//	
//	int nb_people <- 500 parameter: "People in the system" category: "Initial";
//	int nb_car <- ((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5)  parameter: "AV in the system" category: "Initial";
//	int nb_cars <- ((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5)  parameter: "Cars in the system" category: "Initial";
//	bool centralised_ON <- true parameter: "Centralised or Decentralised: " category: "Initial"; //0= centralised, 1 = decentralised
//	bool not_reset<-true;
//
//	//represent the day time for the agent to inform them to go work or home
//	float current_hour update: ((time/#hour)-(g)*24);
//	int h update: (time/#hour) mod 24; 
//	int g update: int(time/#hour/24); 
//	
//	//Variables to manage the minimal and maximal time to start working/go home
//	float min_work_start <- 0.5 parameter: "Min. start hour: " category: "Hours";
//	float max_work_start <- 1.0 parameter: "Max. start hour: " category: "Hours";
//	float min_work_end <- 17.0 parameter: "Min. end hour: " category: "Hours";
//	float max_work_end <- 18.0 parameter: "Max. end hour: " category: "Hours";
//	
//	float before_work_search<-0.5  parameter: "Minutes before work to search for lift: " category: "Hours"; //(30 minuti)
//	float before_work_start	<-0.25  parameter: "Minutes before work to go alone: " category: "Hours"; // (15 minuti)
//	float after_work_start<-0.5  parameter: "Minutes after work to go alone: " category: "Hours";
//	
//	//Manage the speed allowed in the model for the people agents
//	float min_speed <- 5.0  #km / #h;
//	float max_speed <- 20.0 #km / #h; 
//	
//	float cost_km <- 1.50;
//	
//	float v_length <- 5.0#m;
//	
//	float stats_grouping_time <-0.0;
//	float stats_path_time <-0.0;
//	int n_grouping;
//	
//	float start_simulation;
	
	init {
		write string((nb_people mod 5) >0) + " " + string(nb_car);
		//create the intersection and check if there are traffic lights or not by looking the values inside the type column of the shapefile and linking
		// this column to the attribute is_traffic_signal. 
		create intersection from: shape_file_nodes with:[is_traffic_signal::(read("type") = "traffic_signals")];
		
		//create road agents using the shapefile and using the oneway column to check the orientation of the roads if there are directed
		create road from: shape_file_roads with:[lanes::int(read("lanes")), oneway::string(read("oneway")), junction::string(read("junction"))] {
			geom_display <- shape + (2.5 * lanes);
			maxspeed <- (lanes = 1 ? 30.0 : (lanes = 2 ? 50.0 : 70.0)) °km/°h;
			switch oneway {
				match "no" {
					if junction!='roundabout'{
						create road {
							lanes <- max([1, int (myself.lanes / 2.0)]);
							shape <- polyline(reverse(myself.shape.points));
							maxspeed <- myself.maxspeed;
							geom_display  <- myself.geom_display;
							linked_road <- myself;
							myself.linked_road <- self;
						}
						lanes <- int(lanes /2.0 + 0.5);
					}
				}
				match "-1" {
					shape <- polyline(reverse(shape.points));
				}	
			}
		}
		
//		create a new linked road for all oneway roads that are isolated and are not roudabouts
		ask road where ((each.linked_road)=nil and each.junction!='roundabout'){
			create road {
				lanes <- myself.lanes;
				shape <- polyline(reverse(myself.shape.points));
				maxspeed <- myself.maxspeed;
				geom_display  <- myself.geom_display;
				linked_road <- myself;
				myself.linked_road <- self;
			}
		}
		
//		create road weight for the road network 
		road_weights <- road as_map (each::(each.shape.perimeter));
		graph_weights <- road as_map (each::each.shape.perimeter);
		//creation of the road network using the road and intersection agents
		road_network <-  (as_driving_graph(road, intersection))  with_weights road_weights;
		the_graph<- as_edge_graph(road);
		
		//initialize the traffic light
		ask intersection {
			do initialize;
		}

//		creation of the builgings from the shape files and division between residential and 
		create building from: shape_file_buildings with:[type::string(read("type")), group::string(read("group"))]; 
		ask building{
			if group='residential'{
				color <- #lightblue;
			} else{
				color<-#blue;
			}
		}
		write "created buildings";
		list<building> living_buildings<- building where (each.group='residential');
		list<building> work_buildings <-building where (each.group='industrial');

//		creation of the people with random parameters
		create people number: nb_people { 
			speed <- min_speed + rnd (max_speed - min_speed) ;
			start_work <- rnd (min_work_start,max_work_start, 0.5);
			end_work <- rnd (min_work_end,max_work_end, 0.5);
			living_place <- one_of(living_buildings) ;
			working_place <- one_of(work_buildings) ;
			location <- living_place.location;
		}
		
		write "created people";

//		creation of AV with random parameters
		if centralised_ON = true{
			create cenAV number: nb_car { 
				max_speed <- 160 °km/°h;
				vehicle_length <- v_length;
				right_side_driving <- true;
				proba_lane_change_up <- 0.1 + (rnd(500) / 500);
				proba_lane_change_down <- 0.5+ (rnd(500) / 500);
				location <- one_of(road).location;
				security_distance_coeff <- 5/9 * 3.6 * (1.5 - rnd(1000) / 1000);  
				proba_respect_priorities <- 1.0 - rnd(200/1000);
				proba_respect_stops <- [1.0];
				proba_block_node <- 0.0;
				proba_use_linked_road <- 0.0;
				max_acceleration <- 5/3.6;
				speed_coeff <- 1.2 - (rnd(400) / 1000);
				threshold_stucked <-int ( (1 + rnd(5))°mn);
				proba_breakdown <- 0.00001;
				max_passengers<-5;
			}
		} else{
			create decAV number: nb_car { 
				max_speed <- 160 °km/°h;
				vehicle_length <- v_length;
				right_side_driving <- true;
				proba_lane_change_up <- 0.1 + (rnd(500) / 500);
				proba_lane_change_down <- 0.5+ (rnd(500) / 500);
				location <- one_of(road).location;
				security_distance_coeff <- 5/9 * 3.6 * (1.5 - rnd(1000) / 1000);  
				proba_respect_priorities <- 1.0 - rnd(200/1000);
				proba_respect_stops <- [1.0];
				proba_block_node <- 0.0;
				proba_use_linked_road <- 0.0;
				max_acceleration <- 5/3.6;
				speed_coeff <- 1.2 - (rnd(400) / 1000);
				threshold_stucked <-int ( (1 + rnd(5))°mn);
				proba_breakdown <- 0.00001;
				max_passengers<-5;
			}
		}

//		creation of the other cars with random parameters
		create cars number: nb_cars { 
			max_speed <- 160 °km/°h;
			vehicle_length <- v_length;
			right_side_driving <- true;
			proba_lane_change_up <- 0.1 + (rnd(500) / 500);
			proba_lane_change_down <- 0.5+ (rnd(500) / 500);
			location <- one_of(intersection where empty(each.stop)).location;
			security_distance_coeff <- 5/9 * 3.6 * (1.5 - rnd(1000) / 1000);  
			proba_respect_priorities <- 1.0 - rnd(200/1000);
			proba_respect_stops <- [1.0];
			proba_block_node <- 0.0;
			proba_use_linked_road <- 0.0;
			max_acceleration <- 5/3.6;
			speed_coeff <- 1.2 - (rnd(400) / 1000);
			threshold_stucked <-int ( (1 + rnd(5))°mn);
			proba_breakdown <- 0.00001;
		}	
	}
	reflex stop_simulation when: remove_duplicates(people collect each.state)=['working'] and (length(list(people))=nb_people){
		write "passengers taken: " +length(people where (each.got_lift=true));
      do pause ;
   }
	reflex update_road_speed {
		road_weights <- road as_map (each::(each.shape.perimeter * (each.speed_coeff)));
		road_network <- road_network with_weights road_weights;
	}
	
	reflex update_not_full_cars when: (centralised_ON and (( ( ( current_hour >(min_work_start-before_work_search) ) and ( current_hour < (max_work_start-before_work_start) ) ) or 
					( (current_hour >min_work_end) and (current_hour <(max_work_end +after_work_start)) ) ))){
		not_full_cars <- cenAV where (((each.state='wander') or (each.state='still_space')) and (each.passengers_waiting<=each.max_passengers));
		not_reset <- true;
	}
	reflex clear_not_full_cars when: (centralised_ON and not_reset and (( ( ( current_hour <=(min_work_start-before_work_search) ) and ( current_hour >= (max_work_start-before_work_start) ) ) or 
					( (current_hour <= min_work_end) and (current_hour >=(max_work_end +after_work_start)) ) ))){
						not_full_cars <- [];
						not_reset<- false;
	}
	reflex create_groups  when:(centralised_ON and ( ( ( current_hour >(min_work_start-before_work_search) ) and ( current_hour < (max_work_start-before_work_start) ) ) or ( (current_hour >min_work_end) and (current_hour <(max_work_end +after_work_start)) ) )
								and !empty(not_full_cars)){
		
		list<list<people>> people_in_range <- (people where ((each.state contains 'search_lift' )and each.the_target!=nil) simple_clustering_by_distance 1 )  where (/*( (length (each)) <=5) and*/ ( (length (each)) >0) ) ;
		if(people_in_range!=[]){
			stats_grouping_time <- machine_time;
			
			
			write "___________________GROUPS @ " + h+" ("+ current_hour+") DAY "+(g+1)+"___________________";
			loop bigger over: people_in_range where (length(each)>5){
//				write "group longer than 5 " +bigger;
				loop while: length(bigger)>5{
					list<people> five<-copy_between(bigger,0,5);
//					write five;
					remove all:five from: bigger;
//					write bigger;
					add five to: people_in_range;
					five<-nil;
				}
			}
			
			int people_considered <- length(people where ((each.state contains 'search_lift' )and each.the_target!=nil));
			int cars_considered <- length(not_full_cars);
			int groups_considered <- length(people_in_range);
			
			loop one_group over: people_in_range{
				list<string> names<-one_group collect each.name; 
//				write string(names) + " - "+string(one_group);
				intersection pass_location<-intersection closest_to (one_of(one_group).location);
//				write "\n"+"pass_location: "+pass_location;
				map<intersection, list<string>> destinations<-nil;
				loop p over: one_group{
					if pass_location=nil{
						pass_location<- intersection closest_to (p.location);
					}
 					if destinations.keys contains (intersection closest_to p.the_target){
 						add p.name to: destinations[(intersection closest_to p.the_target)];
 					}else{
 						add (intersection closest_to p.the_target)::[p.name] to: destinations;
 					}
				}
//				write " one_group: " + one_group+ " destinations:"+ destinations;
//				write "destinations: "+ destinations;
				
//				write string(names) + " " + string(destinations) + " "+ string(pass_location);
//				assign group to the nearest car
				list<cenAV> available_cars <- (not_full_cars where  ( (each.passengers_waiting < each.max_passengers) and ((road_network path_between (each.location, pass_location)).edges!=[]))) 
								sort_by ((road_network path_between (each.location, pass_location)).shape.perimeter / each.speed);
								
				loop c over: available_cars{
//					write "CAR: "+c.name+ " GROUP: "+ one_group;
					path tmp <- nil;
					int ref_angle <-0;
					intersection origin<-nil;
					float time_tmp <-0.0;
					if !empty(c.give_lift_to.keys){
						origin<-  last(c.give_lift_to.keys);
						ref_angle <- angle_between(first(c.give_lift_to.keys).location, c.location, pass_location.location);
						tmp <- road_network path_between(last(c.give_lift_to.keys), pass_location);
						time_tmp <- tmp.edges=[] ? 500 : tmp.shape.perimeter/8.3;
					} else{
						tmp <- road_network path_between(c.location, pass_location);
						time_tmp <- tmp.edges=[] ? 500.0 : 0.0;
					}
					
					if tmp.edges!=[]{
						if ref_angle<30 and time_tmp <180{
//							write c.name + " angle: " + ref_angle + " source of path:" + ((origin=nil) ? c.location : origin) +" time @ slowest "+ tmp.shape.perimeter/8.3; 
							list<people> copy1;
							list<people> to_remove;
							add all:one_group to: copy1;							
							loop d over: destinations.keys{
								if d = pass_location{
									 to_remove <- copy1 where (!empty([each.name] inter destinations[d]));
									remove all:to_remove from: copy1;
								}else{
									if !empty(c.people_destinations.keys){
										point centre <- mean(c.people_destinations.keys collect each.location);
										if d distance_to centre < 500{
											if !(c.people_destinations.keys contains d){
												add d::destinations[d] to: c.people_destinations;
											} else{
												add all:destinations[d] to:c.people_destinations[d]; 
											}
											
										}else{
											to_remove <- copy1 where (!empty([each.name] inter destinations[d]));
											remove all:to_remove from: copy1;
	//										write c.name + " (removed) d:"+ d+" people: "+ destinations[d];
										}
									}else{
										add d::destinations[d] to: c.people_destinations;
									}
								}
							}
							if copy1!=[]{
								if c.give_lift_to.keys contains pass_location{
									add all:copy1 to: c.give_lift_to[pass_location];
								} else{
									add pass_location::copy1 to: c.give_lift_to;
								}
								remove all:copy1 from:one_group;
								loop p over: copy1{
									p.state<- 'wait_for_lift';
									add p to: c.all_passengers;
								}
	//							write c.name + " " + c.give_lift_to + " "+ c.people_destinations;
								c.passengers_waiting <- length(copy1)+ c.passengers_waiting;
								
								c.state<-'still_place';
								write c.name + " was assigned passengers (TOT: "+c.passengers_waiting+"): " + copy1 collect each.name + " that are at: "+ pass_location + " and current_location is: " + c.location ;
								write c.name + " there is a path between us with distance : "+ tmp.shape.perimeter;
								write c.name + " waiting : "+ c.passengers_waiting + " give_lift_to : "+ c.give_lift_to;
								break;
							}
						} 
					}
				}
			}
			stats_grouping_time<- machine_time - stats_grouping_time;

			n_grouping<- n_grouping+1;
			write "grouping "+(n_grouping<0? "0":"") +n_grouping+ ", " + stats_grouping_time + ", "+ people_considered + ", "+cars_considered + ", "+groups_considered;
		}
	}
	
	reflex stop_simulation when: remove_duplicates(people collect each.state)=['working'] and (length(list(people))=nb_people){
		list<people> were_passengers <- (people where (each.got_lift=true));
		list<float> costs_proposed <- were_passengers collect each.cost_proposed;
		list<float> w_times<- were_passengers collect each.waiting_time;
		write "passengers taken: " +length(were_passengers);
		write "cars used: " +length(cenAV where (each.gave_lift=true));
		write 'cost proposed ('+(length(were_passengers)=length(costs_proposed))+' ' +length(costs_proposed)+'): ' + mean(costs_proposed);
		write costs_proposed;
		write 'waiting times ('+(length(were_passengers)=length(w_times))+' ' +length(w_times)+'): ' + mean(w_times);
		write w_times;
		write 'total_duration '+ total_duration + ' current_hour '+ current_hour;
		do pause ;
   }
}


experiment Centralised type: gui {
	float minimum_cycle_duration <- 0.01;
	output {
		monitor "Current hour" value: current_hour;
		
		display city_display {
			graphics "world" {
				draw world.shape.contour color:#black;
			}
			
			species building aspect: base refresh:false;
			species road aspect: base ;
			species intersection aspect: base;
			species cenAV aspect: base;
			species cars aspect: base;
			species people aspect: base transparency: 0.2;
			
		}
	}	
}

experiment DecentralisedSimulation type: gui {
	float minimum_cycle_duration <- 0.01;
	
	output {
		monitor "Current hour" value: current_hour;
		
		display city_display {
			species building aspect: base refresh:false;
			species road aspect: base ;
			species intersection aspect: base;
			species decAV aspect: base;
			species cars aspect: base;
			species people aspect: base transparency: 0.2;
			
		}
	}
}
