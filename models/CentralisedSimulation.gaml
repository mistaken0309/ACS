model CentralisedSimulation
 
global {   
	
	file shape_file_roads  <- file("../includes/roads.shp") ;
	file shape_file_nodes  <- file("../includes/nodes.shp");
	file shape_file_buildings  <- file("../includes/buildings.shp");
	geometry shape <- envelope(shape_file_roads);
	
	list<geometry> zones<- to_rectangles(shape, {shape.width/4,shape.height/4});
	
	graph the_graph;  
	graph road_network;  
	map road_weights;
	map graph_weights;
	
	int nb_people <- 500;
//	int nb_cars <- ((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5);
	int nb_cars <- 100;
	int nb_car <- ((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5);
	
	list<car> not_full_cars <- list(car) 
				update: ( ( ( current_hour >(min_work_start-before_work_search) ) and ( current_hour < (max_work_start-before_work_start) ) ) or 
					( (current_hour >min_work_end) and (current_hour <(max_work_end +after_work_start)) ) ) ? 
					(car where (((each.state='wander') or (each.state='still_space')) and (each.passengers_waiting<=each.max_passengers))) : []; 
	float step <- 5 #s;	

	//represent the day time for the agent to inform them to go work or home
	float current_hour update: ((time/#hour)-(g)*24);
	int h update: (time/#hour) mod 24; 
	int g update: int(time/#hour/24); 
	
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
	
	// distances covered by people and cars in the system, when ensembles are created
//	float cars_tot_distance_cover<-0.0#m  ;
//	float people_tot_distance_cover<-0.0#m  ;
//	float sys_tot_distance_cover<-0.0#m ;
//		
//	float cars_tot_distance_covered<-0.0#m  update:  sum (car collect each.dist_covered_cars) #m;
//	float people_tot_distance_covered<-0.0#m update: sum(people collect each.dist_covered_alone)#m;
//	float sys_tot_distance_covered<-0.0#m update: people_tot_distance_covered#m+cars_tot_distance_covered#m;
	
	//Stock the number of times agents reached their goal (their house or work place)
	//int nbGoalsAchived <- 0 update count (sum people collect each.n_goal);
	
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
		
		road_weights <- road as_map (each::each.shape.perimeter * each.speed_coeff);
		graph_weights <- road as_map (each::each.shape.perimeter);
//		road_weights <- road as_map (each::((each.shape.perimeter / each.maxspeed)));
		//creation of the road network using the road and intersection agents
		road_network <-  (as_driving_graph(road, intersection))  with_weights road_weights;
		the_graph<- as_edge_graph(road);
		
		//initialize the traffic light
		ask intersection {
			do initialize;
		}
		
		create building from: shape_file_buildings with:[type::string(read("type")), group::string(read("group"))]; 
		ask building{
			if group='residential'{
				color <- #cyan;
			} else{
				color<-#blue;
			}
		}
		write "created buildings";
		list<building> living_buildings<- building where (each.group='residential');
		list<building> work_buildings <-building where (each.group='industrial');
//		list<building> l_buildings<- copy_between(living_buildings,120,124);
		create people number: nb_people { 
			speed <- min_speed + rnd (max_speed - min_speed) ;
			start_work <- rnd (min_work_start,max_work_start, 0.5);
			end_work <- rnd (min_work_end,max_work_end, 0.5);
			living_place <- one_of(living_buildings) ;
			working_place <- one_of(work_buildings) ;
			location <- living_place.location;
//			road_knowledge<-road_weights;
		}
		
		write "created people";
		
		create car number: nb_car { 
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
//		road_weights <- road as_map (each::each.shape.perimeter / each.speed_coeff);
		road_weights <- road as_map (each::(each.shape.perimeter * (each.speed_coeff)));
		road_network <- road_network with_weights road_weights;
	}
	reflex create_groups  when:( ( ( current_hour >(min_work_start-before_work_search) ) and ( current_hour < (max_work_start-before_work_start) ) ) or ( (current_hour >min_work_end) and (current_hour <(max_work_end +after_work_start)) ) )
								and !empty(not_full_cars){
		
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
//				list<car> available_cars <- (car where (((each.on_board + each.passengers_waiting + length(one_group))<= each.max_passengers) 
//					and ((road_network path_between (each.location, pass_location)).edges!=[]))) 
//					sort_by ((road_network path_between (each.location, pass_location)).shape.perimeter / each.speed);
				list<car> available_cars <- (not_full_cars where  ( (each.passengers_waiting < each.max_passengers) and ((road_network path_between (each.location, pass_location)).edges!=[]))) 
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
		write "cars used: " +length(car where (each.gave_lift=true));
		write 'cost proposed ('+(length(were_passengers)=length(costs_proposed))+' ' +length(costs_proposed)+'): ' + mean(costs_proposed);
		write costs_proposed;
		write 'waiting times ('+(length(were_passengers)=length(w_times))+' ' +length(w_times)+'): ' + mean(w_times);
		write w_times;
		write 'total_duration '+ total_duration + ' current_hour '+ current_hour;
		do pause ;
   }
} 

//species that will represent the intersection node, it can be traffic lights or not, using the skill_road_node skill
species intersection skills: [skill_road_node] {
	bool is_traffic_signal;
	list<list> stop;
	int time_to_change <- rnd(6,10);
	int counter <- rnd (time_to_change) ;
	list<road> ways1;
	list<road> ways2;
	bool is_green;
	rgb color_fire;
	
	action initialize {
		if (is_traffic_signal) {
			do compute_crossing;
			stop<< [];
			if (flip(0.5)) {
				do to_green;
			} else {
				do to_red;
			}	
		}
	}
	
	action compute_crossing{
		if  (length(roads_in) >= 2) {
			road rd0 <- road(roads_in[0]);
			list<point> pts <- rd0.shape.points;						
			float ref_angle <-  float( last(pts) direction_to rd0.location);
			loop rd over: roads_in {
				list<point> pts2 <- road(rd).shape.points;						
				float angle_dest <-  float( last(pts2) direction_to rd.location);
				float ang <- abs(angle_dest - ref_angle);
				if (ang > 45 and ang < 135) or  (ang > 225 and ang < 315) {
					ways2<< road(rd);
				}
			}
		}
		loop rd over: roads_in {
			if not(rd in ways2) {
				ways1 << road(rd);
			}
		}
	}
	
	action to_green {
		stop[0] <- ways2 ;
		color_fire <- #green;
		is_green <- true;
	}
	
	action to_red {
		stop[0] <- ways1;
		color_fire <- #red;
		is_green <- false;
	}
	reflex dynamic_node when: is_traffic_signal  {
		counter <- counter + 1;
		if (counter >= time_to_change) { 
			counter <- 0;
			if is_green {do to_red;}
			else {do to_green;}
		} 
	}
	
	aspect base {
		if (is_traffic_signal) {	
			draw circle(5) color: color_fire;
		}
	}
	
	aspect base3D {
		if (is_traffic_signal) {	
			draw box(1,1,10) color:#black;
			draw sphere(5) at: {location.x,location.y,12} color: color_fire;
		}
	}
}
//species that will represent the roads, it can be directed or not and uses the skill skill_road
species road skills: [skill_road] { 
	geometry geom_display;
	string oneway;
	string junction;
	float perim<-shape.perimeter;	
	int nb_agents<-length(all_agents) update: length(all_agents);
	float capacity <- 1+(shape.perimeter*lanes)/v_length;
	float speed_coeff<- 0.1 update: (length(all_agents)/capacity) min:0.1 max:((shape.perimeter < 1.0) ? 1.0 : 3.0);
	
	int ov_rgb<-150 update: 150-(150*int(speed_coeff-0.1)) min: 0 max:255; //0.1 ->150 // 1 e oltre -> 0
	int ov_rgbR<-150 update: 255*int(speed_coeff-0.1)  min: 150 max: 255; // 1 e oltre -> 255 // 0.1 -> 0
	rgb color<-rgb(127,127,127) update: rgb(ov_rgbR, ov_rgb, ov_rgb);
	
	aspect base {    
		draw shape color: color end_arrow: 6;
	} 
	aspect base3D {    
		draw geom_display color: #gray ;
	} 
}
//species that will represent the buildings
species building{
	string type;
	string group;
	rgb color<- rgb(200,200,200);

	aspect base{
		draw shape color: color border: #gray;
	}
}

species people skills:[moving] control: fsm {
	rgb color <- #lightblue ;
	building living_place <- nil ;
	building working_place <- nil ;
	float start_work ;
	float end_work ;
	point the_target <- nil ;
	float dist<-0.0 ;
	float dist_covered_alone ;
	intersection origin;
	
	bool starting<-true;
	bool late<-false;
	float actual_time_in;
	
	map road_knowledge<-graph_weights update: graph_weights;
	path path_to_follow<-nil;
	float look_up<-50.0;
	string next_state<-nil;
	bool got_lift<-false; 
	list<intersection> close_intersections <- nil 
		update: ((state contains 'search_lift') ) ? intersection overlapping (self.shape+look_up) : nil;
	
	float distance_to_cover;
	float time_to_cover<-0.0;
	float time_needed<-0.0;
	float cost_to_cover<-0.0;
	float cost_proposed<-0.0;
	float departure_time<-0.0;
	float arrival_time<-0.0;
	float waiting_time<-0.0 update: ((state contains "search_lift") and got_lift) ? (current_hour - waiting_time) : waiting_time;
	bool set_waiting_time<-true;
	float time_trip <- 0.0;
	float total_time_needed  <- 0.0;
	
	state resting initial:true{
		enter{
			color <- #lightblue;
			next_state<-'search_lift_work';
		}
		transition to: search_lift_work when: current_hour = start_work-before_work_search;
	}
	state search_lift_work{
		enter{
			the_target <- working_place.location;
			color<- #yellow;
			late<-false;
			next_state<-'go_work';
		}
		transition to: go_work when: current_hour = start_work -before_work_start;
	}
	state wait_for_lift{
		if current_hour>start_work and current_hour<end_work{
//			write string(self.name)+" getting late " + string(current_hour) + " " + string(start_work);
			late<-true;
		}else{
			late<-false;
		}
		color<-#darkseagreen;
	}
	state go_work{
		enter{
			the_target<-working_place.location;
			if current_hour>start_work{
//				write string(self.name)+" getting late " + string(current_hour) + " " + string(start_work);
				late<-true;
			} else{
				late<-false;
			}
			color<-#thistle;
			next_state<-'working';
		}
		transition to: working when: current_hour >= start_work and self.location = working_place.location;
	}
	state working{
		enter{
			color <- #blue;
			actual_time_in<-time/3600;
			if late{write string(self.name) + " ATI "+actual_time_in + " SUPPOSED "+start_work ;}
			next_state<-'search_lift_home';
		}
		transition to: search_lift_home when: current_hour = end_work;
	}
	state search_lift_home{
		enter{
			the_target <- living_place.location;
			color<- #yellow;	
			late<-false;
			color<-#thistle;
			next_state<-'go_home';
		}
		transition to: go_home when: current_hour = end_work+after_work_start;
	}
	state go_home{
		enter{
			the_target<-living_place.location;
			next_state<-'resting';
		}
		transition to: resting when: self.location = living_place.location;		
	}
	
	reflex start_waiting_time when: (state contains "search_lift") and got_lift=false and set_waiting_time{
		if set_waiting_time{
			waiting_time <- current_hour;
			set_waiting_time<-false;
		}
	}
//	reflex stop_waiting_time when: (state contains "search_lift") and got_lift{
//		waiting_time <- current_hour - waiting_time;
//	}
	reflex search_path when: the_target!=nil and path_to_follow=nil and ((state contains "search_lift") or (state contains 'go' and location!=the_target)){
		if (path_to_follow = nil) {
			//Find the shortest path using the agent's own weights to compute the shortest path
			path_to_follow <- path_between(the_graph with_weights road_knowledge, location,the_target);
			if path_to_follow!=nil{
				list<geometry> segments <- path_to_follow.segments;
				loop seg over:segments{
					dist <- (dist + seg.perimeter );
					time_needed<- (time_needed + (seg.perimeter/(speed)));
					time_needed<- (time_needed + (seg.perimeter/(speed)));
					
				}
			}
		}
	}
	reflex move when: path_to_follow!=nil and (state contains 'go_'){
		//the agent follows the path it computed but with the real weights of the graph
		if starting{
			departure_time<- current_hour;
			starting<-false;
		}
		do follow path:path_to_follow speed: 30.0#m/#s move_weights: graph_weights;
		if the_target = location {
			arrival_time<- current_hour;
			time_to_cover<- arrival_time-departure_time;
			dist_covered_alone<-dist_covered_alone + dist;
			dist<-0.0;
			the_target <- nil ;
			time_needed<-0.0;
			path_to_follow<-nil;
			starting<-true;
		}
	}
	aspect base {
//		draw triangle(50) color: color rotate: 90 + heading;
		draw square(30) color: color;
	}
}
//Car species (ensemble) that will move on the graph of roads to a target and using the skill advanced_driving
species car skills: [advanced_driving] control:fsm{ 
	rgb color <- rgb(rnd(200), rnd(200), rnd(200)) ;
	int counter_stucked <- 0;
	int threshold_stucked;
	bool breakdown <- false;
	float proba_breakdown ;
	intersection the_target;
	float time_needed;
	float dist <-0.0;
	float dist_covered_cars<-0.0;
	bool gave_lift <- false;
	int n_travels <- 0;
	
	map<intersection, list<people>> give_lift_to;
	map<intersection, list<string>> people_destinations;
	map<list<intersection>, list<float>> cost_legs;
	list<people> all_passengers;
	map<string, list<float>> cost_passengers<-nil; // list of costs per each passenger
	
	int max_passengers<-5;
	int passengers_waiting<-0;
	int on_board<- 0 update: length(passenger) min:0;
	int time_to_go<-rnd(12,24);
	int counter<-0;
	
	map<list<string>, float> waiting_time<-nil;
	float mean_of_costs<-0.0;
	float max_of_costs<-0.0;
	float min_of_costs<-0.0;
	
	float arrived_time<-0.0;
	float starting_time<-0.0;	
	
	state wander initial: true{}
	state still_place{}
	state moving{} 
	state stop{}
	state computing{}
	
	reflex breakdown when: flip(proba_breakdown){
		breakdown <- true;
		max_speed <- 1 °km/°h;
		
	}
//	reflex keep_in_check_real_speed{
//		float rs_ma<-(real_speed + max_acceleration);
//		float road_max;
//		float confront_car_road ;
//		if current_road!=nil{
//			road_max <- (road(current_road).maxspeed * speed_coeff);
//			confront_car_road <- (rs_ma < road_max ? rs_ma: road_max);
//		}else{
//			confront_car_road <- rs_ma;
//		}
//		real_speed <- (max_speed<confront_car_road) ? max_speed : confront_car_road;
//	}
	reflex get_path_wandering when: final_target = nil and state='wander'{
		the_target <- one_of(intersection where not each.is_traffic_signal);
		current_path <- compute_path(graph: road_network, target: the_target );
		if (current_path = nil) {
			final_target <- nil;
		}else{starting_time<- time;}
	}
	
	action reset_vars_for_wander{
		state<- 'wander';
		the_target<-nil;
		
		passengers_waiting<-0;
		on_board<-0;
		all_passengers<-nil;
		
		
		give_lift_to<-nil;
		people_destinations<-nil;
		cost_legs<-nil;
		cost_passengers<-nil;
		waiting_time<-nil;
		
		mean_of_costs<-0.0;
		max_of_costs<-0.0;
		min_of_costs<-0.0;
		
		counter<-0;
	}	
	
	action chose_new_target{
//		write self.name + " lift: " + give_lift_to.keys + " check " + (!empty(give_lift_to.keys))+ " " + people_destinations.keys;
		if !empty(give_lift_to.keys){
			the_target<- first(give_lift_to.keys);
		}else{
			the_target<- first(people_destinations.keys);
		}
		write self.name + " chosen "+ the_target+" as new target";
	}
	
	reflex get_path_moving when: final_target = nil and state='moving'{
		loop while: the_target=nil{
			do chose_new_target;
//			write '\n'+self.name+ ' chosed new_target: '+ the_target;
		}
		
		current_path <- compute_path(graph: road_network, target: the_target );
		
		if current_path!=nil{
			list<road> roads_in_path <- list<road>(current_path.edges);
			loop r over:roads_in_path{
				dist <- dist+r.shape.perimeter;
				time_needed<- (time_needed + (r.shape.perimeter/r.maxspeed));
			}
//			write string(self.name)+" h"+current_hour + " from "+ intersection overlapping self.location +" to "+ the_target+" Estimated time to cover " + dist+" is " +time_needed + " seconds";
			starting_time<- time;
		}
	}
	
	reflex move when: current_path != nil and final_target != nil and (state='wander' or state='moving'){
		do drive;
		if real_speed < 5°km/°h {
			counter_stucked<- counter_stucked + 1;
			if (counter_stucked mod threshold_stucked = 0) {
				proba_use_linked_road <- min([1.0,proba_use_linked_road + 0.1]);
			}
		} else {
			counter_stucked<- 0;
			proba_use_linked_road <- 0.0;
		}
		
		if (self.location = the_target.location){
			arrived_time<-time;
			final_target<-nil;
//			write string(self.name)+ " h"+current_hour+ " got to destination in "+(arrived_time-starting_time) + " | " +arrived_time+ " --- " + starting_time ;
			dist_covered_cars<-dist_covered_cars + dist;
			dist<-0.0;
			time_needed<-0.0;
			if state='moving'{state<-'stop';}
		}		
	}
	
	reflex waiting_other_passengers when: state='still_place'  {
		counter <- counter + 1;
		if (counter >= time_to_go or passengers_waiting=max_passengers) { 
			counter <- 0;
			the_target<-nil;
			final_target<-nil;
			current_path<-nil;
			state<-'computing';
			write self.name + " h"+current_hour+" setting off. (Passengers waiting: "+passengers_waiting + ". Time_left: "+(time_to_go-counter)*60;
		} 
	}

	
	reflex create_path_costs when:!empty(give_lift_to) and state='computing' {
		stats_path_time<- machine_time;
		write self.name + " give_lift_to: "+ give_lift_to ;
		list<intersection> sorted_origins <- give_lift_to.keys;
		list<intersection> sorted_destinations <- (people_destinations.keys) sort_by (road_network path_between(last(give_lift_to.keys), each));
		
		map<intersection, list<string>> tmp <- people_destinations;
		people_destinations<-nil;
		
		
		int people_on_leg <- 0;
		float time_to_reach<- 0.0;
		path leg<-nil;
		float cost<-0.0;
		list<intersection> new_key;
		float time_leg<- 0.0;
		
		write self.name + " h"+current_hour+" COMPUTING COSTS";
		write self.name + " all passengers: " + all_passengers;
		write self.name + " origins: " + sorted_origins + " destinations: " +sorted_destinations;
		
		int i<-0;
		write self.name + " computing costs for leg <"+ location + ", "+ sorted_origins[0] +">"; 
		leg <- road_network path_between(location, sorted_origins[0]);
		loop e over: list<road>(leg.edges){
			time_to_reach <- time_to_reach + (e.shape.perimeter/e.maxspeed);
			time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
		}
//		write self.name + " time to wait for "+ give_lift_to[sorted_origins[0]]+ " is "+ time_to_reach;

		add (give_lift_to[sorted_origins[0]] collect each.name)::time_to_reach to: waiting_time;
		
		loop times: length(give_lift_to.keys)-1{
			path leg<-nil;
			float cost<-0.0;
			list<intersection> new_key;
			float time_leg<- 0.0;
			
			people_on_leg<- people_on_leg + length(give_lift_to[sorted_origins[i]]);
			write self.name + " computing costs for leg <"+ sorted_origins[i] + ", "+ sorted_origins[i+1] +"> "+ people_on_leg + " passengers on";
			leg <- road_network path_between(sorted_origins[i], sorted_origins[i+1]);
			loop e over: list<road>(leg.edges){
				time_to_reach <- time_to_reach + (e.shape.perimeter/e.maxspeed);
				time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
			}
//			write self.name + " time to wait for "+ give_lift_to[sorted_origins[i+1]]+ " is "+ time_to_reach;
			add (give_lift_to[sorted_origins[i+1]] collect each.name)::time_to_reach to: waiting_time;
			cost <- (leg.shape.perimeter/1000)*cost_km;
			new_key <- [sorted_origins[i], sorted_origins[i+1]];
			add new_key::[cost, people_on_leg, time_leg] to:cost_legs;	
			
			i<- i+1;
		}
		
		i<-0;
		time_leg<-0.0;
		cost<-0.0;
		
		
		if last(sorted_origins)!=sorted_destinations[0]{
			people_on_leg<- people_on_leg + length(give_lift_to[last(sorted_origins)]);
			write self.name + " computing costs for leg <"+ last(sorted_origins) + ", "+ sorted_destinations[0] +"> with "+ people_on_leg + " passengers on"; 
			leg <- road_network path_between(last(sorted_origins), sorted_destinations[0]);
			loop e over: list<road>(leg.edges){
				time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
			}
			cost <- (leg.shape.perimeter/1000)*cost_km;
			new_key <- [last(sorted_origins), sorted_destinations[0]];
			add new_key::[cost, people_on_leg, time_leg] to:cost_legs;
			add sorted_destinations[0]::tmp[sorted_destinations[0]] to:people_destinations;
		} else{
			people_on_leg<- people_on_leg + length(give_lift_to[last(sorted_origins)])-length(tmp[sorted_destinations[i]]);
			i<-1;
			write self.name + " computing costs for leg <"+ last(sorted_origins) + ", "+ sorted_destinations[i] +"> with "+ people_on_leg + " passengers on";
			leg <- road_network path_between(last(sorted_origins), sorted_destinations[i]);
			loop e over: list<road>(leg.edges){
				time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
			}
			cost <- (leg.shape.perimeter/1000)*cost_km;
			new_key <- [last(sorted_origins), sorted_destinations[i]];
			add new_key::[cost, people_on_leg, time_leg] to:cost_legs;
			add sorted_destinations[0]::tmp[sorted_destinations[i]] to:people_destinations;
		}
		loop times: ((last(sorted_origins)!=sorted_destinations[0]) ? (length(sorted_destinations)-1) : (length(sorted_destinations)-2)){
			time_leg<-0.0;
			cost<-0.0;
			
			people_on_leg<- people_on_leg - length(tmp[sorted_destinations[i]]);
			write self.name + " computing costs for leg <"+ sorted_destinations[i] + ", "+ sorted_destinations[i+1] +"> with "+ people_on_leg + " passengers on";
			leg <- road_network path_between(sorted_destinations[i], sorted_destinations[i+1]);
			loop e over: list<road>(leg.edges){
				time_leg<- time_leg + (e.shape.perimeter/e.maxspeed);
			}
//			write self.name + "from: "+ sorted_destinations[i] + " to: "+ sorted_destinations[i+1] + " "+ leg;
			
			cost <- (leg.shape.perimeter/1000)*cost_km;
			new_key <- [sorted_destinations[i], sorted_destinations[i+1]];
			add new_key::[cost, people_on_leg, time_leg] to:cost_legs;
			add sorted_destinations[i+1]::tmp[sorted_destinations[i+1]] to:people_destinations;
			i<- i+1;
			
		}
		
//		write self.name + " waiting times: " + waiting_time + " cost_legs:"+cost_legs;
		
		int total_stops <- 1+ length(sorted_origins) + length(sorted_destinations);
		
		loop p over: all_passengers{
			
			intersection origin <- intersection closest_to (p.location);
			intersection dest <- intersection closest_to (p.the_target);
			write self.name + " computing costs for passenger "+ p.name + " from "+ origin+ "to "+ dest;
			
			cost<-0.0;
			time_leg<-0.0;
			
			list<string> key_waiting <- (waiting_time.keys where (each contains p.name))[0];
			
			write self.name + " waiting_time["+key_waiting+"]="+ waiting_time[key_waiting];
			
			time_to_reach <- waiting_time[key_waiting];
			
			bool next_too<-false;
			
			loop key over: cost_legs.keys{
				if key[0]=origin{
					next_too<-true;
					cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
					time_leg <- time_leg + cost_legs[key][2];
				}
				if next_too{
					cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
					time_leg <- time_leg + cost_legs[key][2];
					if key[1]=dest{break;}
				}
			}
			add (p.name)::[cost, time_to_reach, time_leg] to: cost_passengers;
		}	
		write self.name + " waiting times: " + waiting_time + " cost_legs:"+cost_legs + " cost_passengers: "+cost_passengers;
		stats_path_time<- machine_time - stats_path_time;
		n_travels <- n_travels+1;
		int total_passengers<- length(cost_passengers.keys);
		list<float> costs_p <- cost_passengers.values collect each[0];
		list<float> waiting_times <- cost_passengers.values collect each[1];
		gave_lift<-true;
			
		write ((car index_of self<10)? "0": "") + string(car index_of self) +", " +n_travels +", " +stats_path_time+", " +total_stops+", " +total_passengers+", " +waiting_times+ ", "+ costs_p; 
		state<-'moving';
	}
	reflex capture_people when: !empty(give_lift_to) and state='stop'{
		write self.name + " the_target: "+ the_target + " | "+ give_lift_to[the_target]; 
		list<string>names <- give_lift_to[the_target] collect each.name;
		list<people> to_capture <- give_lift_to[the_target]; 
		remove key:the_target from: give_lift_to;
		write self.name + " removed the key " + the_target + " from " + give_lift_to;
		ask to_capture{
			got_lift<-true;
			cost_proposed <- myself.cost_passengers[name][0];
			time_trip <- myself.cost_passengers[name][2];
			total_time_needed <- waiting_time + time_trip;
		}
		capture to_capture as: passenger{} 
		write self.name + " h"+current_hour+" at: " + the_target + " has captured: " + names;
		
		the_target<-nil;
		state<-'moving';
		write self.name + " h"+current_hour+" changed state back to moving";
	}
	reflex drop_people when:!empty(passenger) and state='stop'{
		list<string>names;
		list<string> states_dropped;
		list<people>dropped;
		string substitute_state;
		
//		write self.name + " Passengers on before droppping someone: " + list(passenger);
//		write self.name + " People destinations: " + people_destinations + " current_target "+ the_target ;
		loop p over: (passenger){
			if people_destinations[the_target] contains p.name and self.location=the_target.location{
//				write self.name + " People destinations[target]: " + people_destinations[the_target] + " - p "+ p + "="+ p.name;
				add p to: dropped;
				add p.name to: names;
				point t<-p.the_target;
				if p.the_target!=location{
					if (p.next_state='working' or p.next_state='go_work') { //next_state
						substitute_state<-'go_work';
					}
					if (p.next_state='resting' or p.next_state='go_home') {
						substitute_state<-'go_home';
					}
				}else{
					substitute_state<-p.next_state;
				}
				add substitute_state to: states_dropped;
				ask p{
					state<- substitute_state;
					location<-myself.location;
					path_to_follow<-nil;
				}
				release p in:world as:people{}
			}
		}
		if !empty(dropped){
			write string(self.name) +' ('+current_hour+') dropped '+(names) + ' with states: '+ states_dropped; //+' @ '+ location +' with: '+p_targets+ ' - ' +state;
		}
		
		remove key:the_target from: people_destinations;
//		write self.name + ' removed the_target from int_targets and from people_destinations and then the_target is set to nil';
		write self.name + ' - '+ people_destinations;
		the_target<-nil;
		if empty(passenger){
			do reset_vars_for_wander;
			write self.name + " back to wandering";
		} else{state<-'moving';}
	}
	
	species passenger parent: people{
		aspect default{
		}
	}

	aspect base {
		draw image_file("../includes/car-png-top-white-top-car-png-image-34867-587.png") size:{5#m,2.5#m} rotate: heading+180;
		draw breakdown ? square(15) : triangle(30) color: color rotate:heading + 90;
	} 
	aspect base3D {
		point loc <- calcul_loc();
		draw box(vehicle_length, 1,1) at: loc rotate:  heading color: color;
		
		draw triangle(0.5) depth: 1.5 at: loc rotate:  heading + 90 color: color;
		
		if (breakdown) {
			draw circle(2) at: loc color: #red;
		}
	} 
	
	point calcul_loc {
		if (current_road = nil) {
			return location;
		} else {
			float val <- (road(current_road).lanes - current_lane) + 0.5;
			val <- on_linked_road ? val * - 1 : val;
			if (val = 0) {
				return location; 
			} else {
				return (location + {cos(heading + 90) * val, sin(heading + 90) * val});
			}
		}
	}
	
} 

//Car species (ensemble) that will move on the graph of roads to a target and using the skill advanced_driving
species cars skills: [advanced_driving]{ 
	rgb color <- rgb(rnd(255), rnd(255), rnd(255)) ;
	int counter_stucked <- 0;
	int threshold_stucked;
	bool breakdown <- false;
	float proba_breakdown ;
	intersection target;

	reflex breakdown when: flip(proba_breakdown){
		breakdown <- true;
		max_speed <- 1 °km/°h;
	}
	reflex keep_in_check_real_speed{
//		real_speed <- road(current_road).maxspeed;
		float rs_ma<-(real_speed + max_acceleration);
		float road_max;
		float confront_car_road ;
		if current_road!=nil{
			road_max <- (road(current_road).maxspeed * speed_coeff);
			confront_car_road <- (rs_ma < road_max ? rs_ma: road_max);
		}else{
			confront_car_road <- rs_ma;
		}
		real_speed <- (max_speed<confront_car_road) ? max_speed : confront_car_road;
	}
	
	reflex time_to_go when: final_target = nil {
		target <- one_of(intersection where not each.is_traffic_signal);
		current_path <- compute_path(graph: road_network, target: target);
		if (current_path = nil ) {
			final_target <- nil;
		}
	}
	reflex move when: current_path != nil and final_target != nil {
		do drive;
		if real_speed < 5°km/°h {
			counter_stucked<- counter_stucked + 1;
			if (counter_stucked mod threshold_stucked = 0) {
				proba_use_linked_road <- min([1.0,proba_use_linked_road + 0.1]);
			}
		} else {
			counter_stucked<- 0;
			proba_use_linked_road <- 0.0;
		}
	}

	aspect base { 
		draw image_file("../includes/car-png-top-white-top-car-png-image-34867-587.png") size:{5#m,2.5#m} rotate: heading+180;
		draw breakdown ? square(15) : triangle(15) color: color rotate:heading + 90;
	} 
	aspect base3D {
		point loc <- calcul_loc();
		draw box(vehicle_length, 1,1) at: loc rotate:  heading color: color;
		
		draw triangle(0.5) depth: 1.5 at: loc rotate:  heading + 90 color: color;
		
		if (breakdown) {
			draw circle(2) at: loc color: #red;
		}
	} 
	
	point calcul_loc {
		if (current_road = nil) {
			return location;
		} else {
			float val <- (road(current_road).lanes - current_lane) + 0.5;
			val <- on_linked_road ? val * - 1 : val;
			if (val = 0) {
				return location; 
			} else {
				return (location + {cos(heading + 90) * val, sin(heading + 90) * val});
			}
		}
	}
	
} 

experiment Centralised type: gui {
	float minimum_cycle_duration <- 0.01;
	parameter "people" var: nb_people <- 100 ;
    parameter "cars" var: nb_car <- ((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5) ;

	output {
		monitor "Current hour" value: current_hour;
//		monitor "Tot distance cars" value: cars_tot_distance_covered/1000;
//		monitor "Tot distance people" value: people_tot_distance_covered/100;
//		monitor "Tot distance covered" value: sys_tot_distance_covered/1000;
		
		display city_display {
			graphics "world" {
				draw world.shape.contour color:#black;
			}
			
			species building aspect: base refresh:false;
			species road aspect: base ;
			species intersection aspect: base;
			species car aspect: base;
			species people aspect: base transparency: 0.2;
			
		}
//		display CostsPeople refresh: every (10 #cycle){
//				    	
//	    	chart "People moving alone" type: histogram size: {1, 0.5} position: {0, 0} 
//	    	title_font: 'Arial'  		title_font_size: 15.0 
//	    	tick_font: 'Arial'		tick_font_size: 10
//	    	label_font: 'Arial'   		label_font_size: 10
//	    	series_label_position: legend{
//					datalist["lift", "alone"] value:[((people where (each.got_lift=true)) collect each.cost_to_cover),
//						((people where (each.got_lift=false)) collect each.cost_to_cover)] color:[#blue, #crimson] line_visible: false;	
//	    	}
//	    	
//	    	chart "Mean of cost for passengers of each car" type: histogram size: {1, 0.5} position: {0, 0.5} 
//			series_label_position: legend{
//				data "max" value: car collect each.max_of_costs color: #blue;	
//				data "mean" value: car collect each.mean_of_costs color: #cyan;
//				data "min" value: car collect each.min_of_costs color: #lightblue;			
//	    	}
//		}
	}
	
}
