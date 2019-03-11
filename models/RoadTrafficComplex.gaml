model RoadTrafficComplex
 
global {   
	file shape_file_roads  <- file("../includes/roads.shp") ;
	file shape_file_nodes  <- file("../includes/nodes.shp");
	file shape_file_buildings  <- file("../includes/buildings.shp");
	geometry shape <- envelope(shape_file_roads);
	
	graph the_graph;  
	graph road_network;  
	map road_weights;
	map graph_weights;
	
	int nb_people <- 100;
	int nb_cars <- 100;//((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5);
	int nb_car <- ((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5);
	 
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
		

		road_weights <- road as_map (each::(each.shape.perimeter));
		graph_weights <- road as_map (each::each.shape.perimeter);
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
				color <- #lightblue;
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
	
	reflex update_road_speed {
		road_weights <- road as_map (each::(each.shape.perimeter * (each.speed_coeff)));
//		road_weights <- road as_map (each::((each.shape.perimeter / each.maxspeed) * (each.speed_coeff)));
		road_network <- road_network with_weights road_weights;
	}

	reflex stop_simulation when: remove_duplicates(people collect each.state)=['working'] and (length(list(people))=nb_people){
		write "passengers taken: " +length(people where (each.got_lift=true));
      do pause ;
   }
} 

//species that will represent the intersection node, it can be traffic lights or not, using the skill_road_node skill
species intersection skills: [skill_road_node] {
	bool is_traffic_signal;
	list<list> stop;
	int time_to_change <- rnd(12,20);
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
	
//	float speed_coeff <- 0.1 update: (1.0- exp(-nb_agents/capacity)) min: 0.1;
//	float speed_coeff<- 0.0 update: (length(all_agents)/capacity) min:0.0; // = 1 max capience reached // <0 still space in // >1 should avoid it
//	float speed_coeff <- 1.0 update: 1 + (float(length((cars+car) at_distance 1.0)) / shape.perimeter * 200 / lanes);
//	int ov_rgb<-150 update: 300*int(speed_coeff)-150 min: 0 max:255; //0 ->150 // 1 e oltre -> 0
//	int ov_rgbR<-150 update: 150*int(speed_coeff)  min: 150 max: 255; // 1 e oltre -> 255 // 0 -> 0
	
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
		}
	}
//	reflex stop_waiting_time when: (state contains "search_lift") and got_lift{
//		waiting_time <- current_hour - waiting_time;
//	}
	reflex search_path when: the_target!=nil and path_to_follow=nil and(state contains "search_lift"){
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
		draw square(50) color: color;
	}
}
//Car species (ensemble) that will move on the graph of roads to a target and using the skill advanced_driving
species car skills: [advanced_driving] control:fsm{ 
	list<float> waiting_times;
	list<float> final_costs;
	
	int n_travel<-0;
	
	int total_passengers_travel<-0;
	int total_stops_travel<-0;
	int n_addition<-0;
	
	float stats_first_path_time<-0.0;
	
	rgb color <- rgb(rnd(200), rnd(200), rnd(200)) ;
	int counter_stucked <- 0;
	int threshold_stucked;
	bool breakdown <- false;
	float proba_breakdown ;
	intersection the_target;
	float time_needed;
	float dist <-0.0;
	float dist_covered_cars<-0.0;
	
	path eventual_path;
	map<intersection, list<people>> people_destinations;
	list<people> first_group;
	map<intersection, list<people>> give_lift_to;
	map<intersection, list<string>> people_destinations_for_dropping;
//	map<intersection, list<float>> people_costs;
	map<list<intersection>, list<float>> cost_legs;
	list<intersection> ordered_or_dest;
	list<intersection> int_targets <- nil;
	bool added_last;
	int added_people;
	int origin_index;
	int index_dest;
	bool origin_exists<-false;
	bool update_costs_for_passengers<-false;
	
	list<float> costs_passengers<-nil; // list of passengers
	float mean_of_costs<-0.0;
	float max_of_costs<-0.0;
	float min_of_costs<-0.0;
	
	int max_passengers<-5;
	list<intersection> current_road_nodes;
	bool update<-true;
	float arrived_time<-0.0;
	float starting_time<-0.0;
	
	people first_p;
	list<people> possible_pass;
	list<people> people_near <- 
				(length(passenger) < max_passengers and current_road_nodes!=nil and state!='first_stop' and update) 
				? (people where ((each.state contains 'search_lift') and each.close_intersections contains_any self.current_road_nodes)) : nil
				update: (length(passenger) < max_passengers and current_road_nodes!=nil and state!='first_stop' and update) 
				? (people where ((each.state contains 'search_lift') and each.close_intersections contains_any self.current_road_nodes)) : nil; 
		
	state wander initial: true{}
	state moving{} 
	state stop{}
	state first_stop{}
	
	reflex update_road_nodes when: current_road!=nil and state!='first_stop'{
		if current_road_nodes contains intersection(road(current_road).target_node){
			update<-false;
			
		}else{
			current_road_nodes<-nil;
			add intersection(road(current_road).target_node) to:current_road_nodes;
			add intersection(road(current_road).source_node) to:current_road_nodes;	
			update<-true;
		}
		
	}
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
	reflex get_path_wandering when: final_target = nil and state='wander'{
		the_target <- one_of(intersection where not each.is_traffic_signal);
		current_path <- compute_path(graph: road_network, target: the_target );
		if (current_path = nil) {
			final_target <- nil;
		}else{starting_time<- time;}
	}
	
	reflex choose_first_passenger when: !empty(people_near) and state='wander'{
		write "\n"+self.name + ' changing state to first_stop';
		stats_first_path_time<- machine_time;
		state<-'first_stop';
		
		eventual_path<-nil;
		intersection origin<- current_road_nodes[1];
		
		possible_pass<- people_near;
		people_near<-nil;
		possible_pass <- possible_pass sort_by (each.the_target distance_to origin);
		intersection first_p_target_inter;
		
		if eventual_path=nil{
			loop while: eventual_path=nil and !empty(possible_pass){
				first_p<- last(possible_pass);
				first_p_target_inter <-(intersection closest_to first_p.the_target);
				eventual_path<- road_network path_between (current_road_nodes[1],first_p_target_inter); // path chosen for first passenger -> chose general path
				if eventual_path.edges=[]{
					first_p<-nil;
				}
				remove first_p from: possible_pass; //add first_p to: checked;
			}
			if eventual_path.edges=[]{
				write self.name + "Couldn't get anybody, going back to wandering";
				self.location<- current_road_nodes[1].location;
				possible_pass<-nil;
				people_near<-nil;
				do reset_vars_for_wander;
			}
		}	
	}
	reflex check_other_cars_first_p when: first_p!=nil and state='first_stop'{
		ask (car-self) where (each.state='first_stop' and each.first_p=first_p){
			if self.possible_pass=myself.possible_pass{
				remove self.first_p from: self.possible_pass;
				if empty(self.possible_pass){
					write self.name + " Somebody else is already taking care of all my passengers";
					write self.name + " Going back to wandering";
					people_destinations_for_dropping<-nil;
					costs_passengers<-nil;
					mean_of_costs<-0.0;
					min_of_costs<-0.0;
					max_of_costs<-0.0;
					self.state<-'wander';
				}else{
					write self.name + " Somebody else ("+myself.name+") is already taking care of "+ first_p.name;
				}
			}
		}
	}	
	reflex change_first_passenger when: first_p=nil and state='first_stop'{
		eventual_path<-nil;
		intersection first_p_target_inter;
		
		if eventual_path=nil{
			loop while: eventual_path=nil and !empty(possible_pass){
				first_p<- last(possible_pass);
				first_p_target_inter <-(intersection closest_to first_p.the_target);
				eventual_path<- road_network path_between (current_road_nodes[1],first_p_target_inter); // path chosen for first passenger -> chose general path
				if eventual_path.edges=[]{
					first_p<-nil;
				}
				remove first_p from: possible_pass;
			}
			if eventual_path=nil{
				state<-'wander';
				write "Couldn't get anybody, going back to wandering";
				possible_pass<-nil;
				people_near<-nil;
				self.location<- current_road_nodes[1].location;
				do reset_vars_for_wander;
			}
		}
	}
	
	action reset_vars_for_wander{
		self.location<- current_road_nodes[1].location;
		state<- 'wander';
		first_p<-nil;
		first_group<-nil;
		possible_pass<-nil;
		people_near<-nil;
		people_destinations<-nil;
		people_destinations_for_dropping<-nil;
		costs_passengers<-nil;
		cost_legs<-nil;
		int_targets<-nil;
		ordered_or_dest<-nil;
		mean_of_costs<-0.0;
		min_of_costs<-0.0;
		max_of_costs<-0.0;
		total_passengers_travel <- 0;
		total_stops_travel <- 0;
		waiting_times <- nil;
		final_costs <- nil;
	}
	action remove_due_to_direction{
		list removed_p;
		loop p over: (possible_pass){ // remove passengers that want to go towards a place in a complete different direction
			if !dead(p){
				if angle_between(last(int_targets).location, first(int_targets).location, p.the_target)>20.0{ // 1st passenger destination, 1st passenger origin, p destination
					add p to: removed_p;
				} else {
					intersection cl_int <- (intersection closest_to p.the_target);
					if (people_destinations.keys contains cl_int){
						add p to:people_destinations[cl_int];
					} else{
						add cl_int::[p] to: people_destinations;
					}
				}
			} else{
				add p to: removed_p;
			}
			
		}
		if !empty(removed_p){
			remove all: removed_p from: possible_pass;
		}
		write self.name + " destinations (inside angle removal): "+ people_destinations;	
		// check in order to avoid to compute the current path again for other passengers that go to the same location;		
		if people_destinations.keys contains last(int_targets){
			first_group<-nil;
			add first_p to:first_group;
			add all:people_destinations[last(int_targets)] to: first_group;
			remove all:people_destinations[last(int_targets)] from: possible_pass; // are already going to be captured;
			remove key: last(int_targets) from: people_destinations;
		}
	}
	
	reflex create_path when: !empty(possible_pass+first_p) and state='first_stop' and first_p!=nil{ 
		int total_available_people <- length(possible_pass);
		intersection origin<- current_road_nodes[1];
		first_group <- [first_p];
		
		intersection first_p_target_inter <- (intersection closest_to first_p.the_target);
		
		eventual_path <- road_network path_between (origin,first_p_target_inter);
		float time_for_eventual_path<-0.0;
		loop e over: (eventual_path.edges){
			time_for_eventual_path <- time_for_eventual_path + (road(e).shape.perimeter / road(e).maxspeed);
		}
		
		write self.name + " First passenger "+ first_p.name + " ("+ first_p +") " +" has objective "+ first_p.the_target +" and the nearest "+ first_p_target_inter.name;
		write  self.name + " other possible pass : "+ possible_pass;
		
		add origin to: int_targets; // formally add starting point
		add first_p_target_inter to: int_targets; // formally add ending point
		write self.name + " - " +int_targets;
		
		do remove_due_to_direction;
		write self.name + " destinations create_pass after angle removal: "+ people_destinations;
		write  self.name + " other possible pass (after direction removal): "+ possible_pass;
		int dest_length<- length(people_destinations.keys);
		if !empty(possible_pass){
			int i<-0;
			float length_ev_path;
			write self.name + " \t" + people_destinations +" - "+ int_targets;
			
			if length(people_destinations.keys)>0{
				length_ev_path <- eventual_path.shape.perimeter;
				
				loop dest over: (people_destinations.keys){
					path deviation1 <- nil;
					path deviation2 <- nil;
					float length_dev;
					
//					write self.name+ " from: "+ int_targets[i]+" to: "+ dest+ " to: "+ last(int_targets);
					deviation1 <- road_network path_between (int_targets[i], dest) ;
					deviation2 <- road_network path_between (dest, last(int_targets));
					
					if (deviation1.edges!=[]) and (deviation2.edges!=[]){
						length_dev <- deviation1.shape.perimeter + deviation2.shape.perimeter ;
					}else{
						length_dev <- length_ev_path + (length_ev_path/3) ;
					}
					
					if length_dev < (length_ev_path) {
						int at_i<- length(int_targets)-1;
						add dest at:at_i to: int_targets;
						write self.name+ " adding "+ dest + " to int_targets: " + int_targets;
						length_ev_path <-0.0;
						int j<-0;
						write self.name + " - " + int_targets;
						loop inters over:(int_targets-first(int_targets)){
							path current_leg<-road_network path_between(int_targets[j], inters);						 
							length_ev_path <- length_ev_path+((current_leg).shape.perimeter) ;							
							write self.name + ' Path from int_targets['+j+'] '+int_targets[j]+' to ' + inters + ' is ' + ((current_leg).shape.perimeter) + /*' with time '+ time_leg + */" TOT dist: " +length_ev_path /* +" TOT time: "+ time_path*/;
							j<-j+1;
						}
						i<-i+1;
					} else{
						remove all:people_destinations[dest] from: possible_pass;
						write self.name+ " remaining possible passengers: "+ possible_pass +"\n";
						remove key:dest from: people_destinations;
					}
					
				}
				
			}
		}
		
		add all:first_group to: possible_pass;
		
		write self.name+ " final possible passengers: "+ possible_pass;
		
		add origin::possible_pass to: give_lift_to;
		add first_p_target_inter::first_group to:people_destinations;
		
		if length(possible_pass)>5{
			write self.name+ " got more passenger than should have";
		}
//		write self.name + " int_targets(final): "+int_targets+ " | people_destinations(final): "+people_destinations;
		int total_added_people ;
		if !empty(possible_pass){
			people_destinations_for_dropping<-nil;			
			costs_passengers<-nil;
			int i <- 0;
			
			float sum_time_leg <- 0.0;
			float sum_distance_leg <- 0.0;
			float sum_cost_leg <- 0.0;
			int leg_passengers <- length(possible_pass);
			
			loop dest over: (int_targets-first(int_targets)){
				float time_leg <- 0.0;
				float cost_leg <- 0.0;
				
				path leg_path <- road_network path_between (int_targets[i], dest);
//				write self.name + " path between "+ int_targets[i] + " to "+ dest +" "+leg_path;
					
				loop e over: list<road>(leg_path.edges){
					time_leg <- time_leg + (e.shape.perimeter / e.maxspeed);
				}
	
				cost_leg <- ((leg_path.shape.perimeter/1000) * cost_km);
//				if i>0{
				sum_cost_leg <- sum_cost_leg + (cost_leg/leg_passengers); 
				sum_time_leg <- sum_cost_leg + time_leg;
//				}
					
//				list<float> costs <- [sum_cost_leg, sum_time_leg];
//				add dest::costs to: people_costs;
				
				add [(int_targets[i]), dest]::[cost_leg,leg_passengers, time_leg]  to: cost_legs;
				
				list<string> names<- nil;	
				loop p over: people_destinations[dest]{
					ask p{
						origin<- myself.current_road_nodes[1];
						got_lift<-true;
						cost_proposed<-sum_cost_leg;
					}
//					write self.name + " LIFT FOR "+ p.name + " "+p.got_lift+" "+p.cost_to_cover + " "+cost_leg;
					add p.name to: names;
				}
				
				add dest::names to:people_destinations_for_dropping;
				
				loop times:length(names){
					add sum_cost_leg to: costs_passengers;
				}
				
				leg_passengers<- leg_passengers - length(people_destinations[dest]);
				i<-i+1;
			}
			add all:int_targets to: ordered_or_dest;
			
			mean_of_costs <- mean(costs_passengers);
			min_of_costs<- min(costs_passengers);
			max_of_costs<- max(costs_passengers); 
			write self.name + " notifying other cars that I've taken: " + possible_pass collect each.name;
			ask car-self{
				remove all: possible_pass from: self.possible_pass;
			}
			total_added_people <- length(possible_pass);
			add all: (possible_pass collect each.waiting_time) to: waiting_times;
			capture possible_pass as:passenger{}
			
			write self.name + ' final destinations: '+people_destinations.keys+ " | final ordered_or_dest: " + ordered_or_dest;
			write self.name + ' initial costs per leg: '+ cost_legs + ' initial costs per passengers: '+ costs_passengers; 
			
			self.location<- origin.location;
//			do search_other_passengers;
			state<-'moving';
			first_p <- nil;
			possible_pass<-nil;
			final_target <- nil;
			the_target <- nil;
			current_path <- nil;
			people_near<-nil;
		} else{
			write self.name + " Couldn't get anybody, going back to wandering";
			possible_pass<-nil;
			people_near<-nil;
			do reset_vars_for_wander;
		}
		stats_first_path_time <- machine_time - stats_first_path_time;
		write "addition "+((car index_of self)<10 ? "0": "") + string(car index_of self) + ", "+ n_travel+ ", "+n_addition+", "+ stats_first_path_time+", "+ total_added_people+", "+ total_available_people + ", "+ 0 + ", "+ dest_length + ", "+ length(ordered_or_dest);
		total_passengers_travel <- total_passengers_travel + total_added_people;
	}
	
	action chose_new_target{
		the_target<- first(int_targets-first(int_targets));
	}
	
	reflex get_path_moving when: final_target = nil and state='moving'{
		loop while: the_target=nil{
			do chose_new_target;
			write '\n'+self.name+ ' chosed new_target: '+ the_target;
		}
		
		current_path <- compute_path(graph: road_network, target: the_target );
		
		if current_path!=nil{
			list<road> roads_in_path <- list<road>(current_path.edges);
			loop r over:roads_in_path{
				dist <- dist+r.shape.perimeter;
				time_needed<- (time_needed + (r.shape.perimeter/r.maxspeed));
			}
			write string(self.name)+" h"+current_hour + " from "+ intersection overlapping self.location +" to "+ the_target+" Estimated time to cover " + dist+" is " +time_needed + " seconds";
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
			write string(self.name)+ " h"+current_hour+ " got to destination in "+(arrived_time-starting_time) + " | " +arrived_time+ " --- " + starting_time ;
			dist_covered_cars<-dist_covered_cars + dist;
			dist<-0.0;
			time_needed<-0.0;
			if state='moving'{state<-'stop';}
		}		
	}
	
	
	reflex update_costs_passengers when: update_costs_for_passengers=true {
		list<float> costs_for_on <- list(passenger) collect each.cost_proposed;
		list<intersection> origins_p <- list(passenger) collect each.origin;
//		write self.name + " current_cost_legs: " + cost_legs;
//		write self.name + " passengers on: " + (list(passenger) collect each.name) + "with origins: "+ origins_p + "with costs "+costs_for_on;
		
		float prev_cost<-0.0;
		loop p over: list(passenger){
			bool next_too<-false;
			float cost<-0.0;
			loop key over: cost_legs.keys{
				if key[0]=p.origin{
					next_too<-true;
					cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
				}
				if next_too{
					cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
					if key[1]=(intersection closest_to p.the_target){break;}
				}
			}
			prev_cost <- p.cost_proposed;
			p.cost_proposed <- cost;
//			write self.name + " updated costs for " + p.name + " is "+ cost +" "+ p.cost_proposed + " (was "+ prev_cost+")"; 
		}
		
		costs_for_on <- list(passenger) collect each.cost_proposed;
		write self.name + " AFTER cost update | passengers on: " + (list(passenger) collect each.name)+" with costs "+costs_for_on;
		update_costs_for_passengers<-false;
		
	}
	
	action update_costs{
		int difference_in_dest_or <- (index_dest - origin_index)-1;
//		write self.name + " origin_index: " + origin_index + " index_dest: " + index_dest + " targets in between: " + difference_in_dest_or;
		int stop_at;
		
		int i<-0;
		bool take_existing<-false;
		map<list<intersection>, list<float>> tmp <- cost_legs;
		cost_legs<-nil;
//		write self.name + " " + tmp.keys + " length: "+ length(tmp.keys);
		loop key over: tmp.keys{
//			write self.name + " VALUE OF I: " + i;
			if i<origin_index-1 or (origin_exists and i <origin_index){
				write self.name + " i:"+ i+ " before origin_index | copying " + key +"::" + tmp[key] +" to: cost_legs";
				add key::tmp[key] to: cost_legs;
			}
			if  ((origin_exists and i=origin_index) or (origin_index=0 and i=0)){
				if difference_in_dest_or = 0 and key[1] != ordered_or_dest[index_dest]{
					write self.name + " the destination is right after the origin " + origin_index + " " + index_dest + " " + key[0] + " " + key[1];
					write self.name + " leg2 - from: " + ordered_or_dest[origin_index] + " to: " +ordered_or_dest[index_dest] + " " + tmp[key][1] + " "+added_people;
					write self.name + " leg3 - from: " + ordered_or_dest[index_dest] + " to: " + key[1] + " "+ tmp[key][1];
					path leg2 <- road_network path_between (ordered_or_dest[origin_index], ordered_or_dest[index_dest]);
					float time2<-0.0;
					loop e over: list<road>(leg2.edges){
						time2<- time2 + (e.shape.perimeter/e.maxspeed);
					}
					float cost2 <- ((leg2.shape.perimeter/1000)*cost_km);
					add [ordered_or_dest[origin_index], ordered_or_dest[index_dest]]::[cost2, int(tmp[key][1] + added_people), time2] to: cost_legs;
					
					path leg3 <- road_network path_between (ordered_or_dest[index_dest], key[1]);
					float time3<-0.0;
					loop e over: list<road>(leg3.edges){
						time3<- time3 + (e.shape.perimeter/e.maxspeed);
					}
					float cost3 <- ((leg3.shape.perimeter/1000)*cost_km) ;
					add [ordered_or_dest[index_dest], key[1]]::[cost3, int(tmp[key][1]), time3] to: cost_legs;
				} else if difference_in_dest_or > 0{
					tmp[key][1]<- int(tmp[key][1] + added_people); 
					add key::tmp[key] to: cost_legs;
					write self.name + " i:"+ i+ " the destination NOT right after the origin | copying (+added) " + key +"::" + tmp[key] +" to: cost_legs";
				}
			}
			if ( !origin_exists and origin_index>0 and i=origin_index-1) {
					write self.name + " " + i +" origin_index!=0 / i=origin_index-1 | leg1 - from: " + key[0] + " to: " + ordered_or_dest[origin_index] + " " + key[0] + " " + key[1] + " " + tmp[key][1] + " " + added_people;
					path leg1 <- road_network path_between (key[0], ordered_or_dest[origin_index]);
					float time1<-0.0;
					loop e over: list<road>(leg1.edges){
						time1<- time1 + (e.shape.perimeter/e.maxspeed);
					}
					float cost1 <- ((leg1.shape.perimeter/1000)*cost_km);
					add [key[0], ordered_or_dest[origin_index]]::[cost1, int(tmp[key][1]), time1] to: cost_legs;
				if difference_in_dest_or = 0 and key[1] != ordered_or_dest[index_dest]{
					write self.name + " the destination is right after the origin " + origin_index + " " + index_dest + " " + key[0] + " " + key[1];
					write self.name + " leg2 - from: " + ordered_or_dest[origin_index] + " to: " +ordered_or_dest[index_dest] + " " + tmp[key][1] + " "+added_people;
					write self.name + " leg3 - from: " + ordered_or_dest[index_dest] + " to: " + key[1] + " "+ tmp[key][1];
					path leg2 <- road_network path_between (ordered_or_dest[origin_index], ordered_or_dest[index_dest]);
					float time2<-0.0;
					loop e over: list<road>(leg2.edges){
						time2<- time2 + (e.shape.perimeter/e.maxspeed);
					}
					float cost2 <- ((leg2.shape.perimeter/1000)*cost_km);
					add [ordered_or_dest[origin_index], ordered_or_dest[index_dest]]::[cost2, int(tmp[key][1] + added_people), time2] to: cost_legs;
					
					path leg3 <- road_network path_between (ordered_or_dest[index_dest], key[1]);
					float time3<-0.0;
					loop e over: list<road>(leg3.edges){
						time3<- time3 + (e.shape.perimeter/e.maxspeed);
					}
					float cost3 <- ((leg3.shape.perimeter/1000)*cost_km) ;
					add [ordered_or_dest[index_dest], key[1]]::[cost3, int(tmp[key][1]), time3] to: cost_legs;
				}else if difference_in_dest_or > 0 or (difference_in_dest_or = 0 and key[1]= ordered_or_dest[index_dest] ){ 
					write self.name + " the destination is NOT right after the origin or it already existed "+ origin_index + " " + index_dest + " "  + key[0] + " " + key[1] + " " + tmp[key][1] + " " + added_people;
					write self.name + " leg2 - from: " + ordered_or_dest[origin_index] + " to: " + key[1]+ " "+ tmp[key][1] + "+" + added_people;
					path leg2 <- road_network path_between (ordered_or_dest[origin_index], key[1]);
					float time2<-0.0;
					loop e over: list<road>(leg2.edges){
						time2<- time2 + (e.shape.perimeter/e.maxspeed);
					}
					float cost2 <- ((leg2.shape.perimeter/1000)*cost_km);
					add [ordered_or_dest[origin_index], key[1]]::[cost2, int(tmp[key][1]+added_people), time2] to: cost_legs;
				}
			}
			if !added_last and ((origin_exists and i>origin_index and i<index_dest-1  and difference_in_dest_or>0)
				or (!origin_exists and i>=origin_index and i<index_dest-2 and origin_index>0 )){
				write self.name + " origin_exist: " + origin_exists + " index_dest-1: " + (index_dest-1) + " index_dest-2: " + (index_dest-2) + " "  + difference_in_dest_or; 
				write self.name + " i:"+ i+ " after origin and before dest | copying " + key +"::" + tmp[key][0] + " " + tmp[key][1]  + "+" + added_people+" with added_people to: cost_legs";
				add key::[tmp[key][0], int(tmp[key][1]+ added_people), tmp[key][2]] to: cost_legs;
			}
			if !added_last and ((origin_exists and i = index_dest-1 and i>=origin_index and difference_in_dest_or>0)
				or (!origin_exists and i = index_dest-2 and i>=origin_index and origin_index>0)){
					if key[1]= ordered_or_dest[index_dest]{
						write self.name + " i:"+ i+ " destination already interted | copying " + key +"::" + tmp[key][0] + " " + tmp[key][1] +"+" +added_people +" to: cost_legs";
						add key::[tmp[key][0], int(tmp[key][1]+ added_people), tmp[key][2]] to: cost_legs;
					} else{
						write self.name + " i:"+ i; 
						write self.name + " leg1 - from: " + key[0] + " to: " +ordered_or_dest[index_dest] + " " + tmp[key][1] + " "+added_people;
						write self.name + " leg2 - from: " + ordered_or_dest[index_dest] + " to: " + key[1] + " "+ tmp[key][1];
						path leg1 <- road_network path_between (key[0], ordered_or_dest[index_dest]);
						float time1<-0.0;
						loop e over: list<road>(leg1.edges){
							time1<- time1 + (e.shape.perimeter/e.maxspeed);
						}
						float cost1 <- ((leg1.shape.perimeter/1000)*cost_km);
						add [key[0], ordered_or_dest[index_dest]]::[cost1, int(tmp[key][1]+added_people), time1] to: cost_legs;
						
						path leg2 <- road_network path_between (ordered_or_dest[index_dest], key[1]);
						float time2<-0.0;
						loop e over: list<road>(leg2.edges){
							time2<- time2 + (e.shape.perimeter/e.maxspeed);
						}
						float cost2 <- ((leg2.shape.perimeter/1000)*cost_km);
						add [ordered_or_dest[index_dest], key[1]]::[cost2, int(tmp[key][1]), time2] to: cost_legs;
					}
			}
//			write self.name + " added_last: "+ added_last + " origin_exists "+ origin_exists + " index_dest-1 " + (index_dest-1) + " index_dest-2 " + (index_dest-2) + " origin_index " + origin_index; 
			if !added_last and (
				(origin_exists and i >= index_dest and difference_in_dest_or = 0)
//				or (origin_exists and i >= index_dest-1 and difference_in_dest_or =0)
				or (!origin_exists and i >= index_dest-1) ){
				write self.name + " i:"+ i+ " after dest | copying " + key +"::" + tmp[key] +" to: cost_legs";
				add key::tmp[key] to: cost_legs;
			}
			if added_last and ((i>=origin_index and !origin_exists) or i>origin_index){
				write self.name + " i:"+ i+ " after origin and added_last | copying " + key +"::" + tmp[key][0] + " " + tmp[key][1]  + "+" + added_people+" with added_people to: cost_legs";
				add key::[tmp[key][0], int(tmp[key][1]+ added_people), tmp[key][2]] to: cost_legs;
			}
			
			i<-i+1;
			
		}
		if added_last{
			intersection key0 <- ordered_or_dest[length(ordered_or_dest)-2];
			intersection key1 <- last(ordered_or_dest);
			write self.name + ' adding as last. From '+ key0 +' to ' + key1;
			path leg1 <- road_network path_between (key0, key1);
			float time1<-0.0;
			loop e over: list<road>(leg1.edges){
				time1<- time1 + (e.shape.perimeter/e.maxspeed);
			}
			float cost1 <- ((leg1.shape.perimeter/1000)*cost_km);
			add [key0, key1]::[cost1, int(added_people), time1] to: cost_legs;
			added_last<-false;
			write self.name + " " + cost_legs;
		}
		
		
	}

		
	reflex add_on_road when: !empty(people_near) and (state='moving') and length(passenger)< max_passengers{ //state!='wander' or state!='first_stop' or state!='stop'
		stats_first_path_time <- machine_time;
		state<-'first_stop';
//		write self.name + " changing to first_stop on the road to add passengers";
		possible_pass<-people_near;
		do remove_due_to_direction;
//		write self.name + " remove passengers due to direction";
		int total_added_people<-0;
		int total_available_people<-length(possible_pass);
		int dest_length <- 0;
		int initial_stops <-0;
		if !empty(possible_pass){
			write self.name + " h"+current_hour+" there are passengers on the " + current_road_nodes[1];
			map<intersection, list<people>> people_on_road <- possible_pass group_by (intersection closest_to each.the_target);
			write self.name + " initial people_on_road: "+ people_on_road;
			
			int inted <- ((ordered_or_dest index_of the_target));
			list<intersection> going_back<- copy_between(ordered_or_dest, 0, inted);
			list<intersection> common_back <- people_on_road.keys inter going_back;
			add current_road_nodes[1] to: common_back;
			
			if !empty(common_back){
				loop k over: common_back{
					remove key:k from: people_on_road;
					write self.name + " removed key: " + k;
				}
			}
			
			common_back<-nil;
			
			if !empty(people_on_road){
				initial_stops <- length(ordered_or_dest);
				
				origin_exists<-false;
				origin_index<- inted-1;
				write self.name + " people_on_road after - common back "+ people_on_road;
				write self.name + " ordered_or_dest[inted-1]: " + ordered_or_dest[inted-1] + " ordered_or_dest[origin_index]: " + ordered_or_dest[origin_index] + " current_road_nodes[1]: " + current_road_nodes[1];	
				
				if ordered_or_dest[inted-1]!=current_road_nodes[1]{
					add current_road_nodes[1] at:inted to: ordered_or_dest;
					write self.name + ' added '+ current_road_nodes[1] + '(new origin) at: ' + inted + ' to:'+ ordered_or_dest;
					origin_index <- inted;
				} else{
					origin_exists<-true;
					write self.name + ' index: '+ origin_index +' origin already existed';
				}
				
				passenger first_still_on <-  first(list(passenger));
				float first_remaining_time <-  (first_still_on.start_work - current_hour)* 3600;
				intersection first_remaining_target;
				float start_to_first <- 0.0;
				
				loop dest over: people_destinations_for_dropping.keys{
					if people_destinations_for_dropping[dest] contains first_still_on.name{
						first_remaining_target<-dest;
						break;
					}
				}
				
				int j <- 0;
				int n <- (int_targets index_of first_remaining_target);
				write self.name + " n:" + n;
				float normal_time<-0.0;
				loop times: n{
					path leg;
					if j = 0{
						leg <- (road_network path_between (location, int_targets[j+1]));
					} else{
						leg<- (road_network path_between (int_targets[j], int_targets[j+1]));
					}
					start_to_first <- start_to_first + (leg.shape.perimeter / 8.3) ;
					loop e over: leg.edges{
						normal_time<- normal_time + (road(e).shape.perimeter / road(e).maxspeed);
					}
					j <- j+1;
				}
				
				write self.name + " first passenger that got on from remaining " +first_still_on.name + " has target "+ first_remaining_target + " and start_to_finish: " +start_to_first + " and in normal time: "+ normal_time;
				
				int remaining_places <- max_passengers - length(list(passenger));
				dest_length<- length(people_on_road.keys);
				loop dest over: people_on_road.keys{
					path target_to_dest ;
					float time_t2dest<-0.0;
					path target_to_next;
					float time_t2next<-0.0;
					float time_dest2next<-0.0;
					bool added<- false;
					int i<-1;
					int index_max <- length(int_targets)-1;
					added_people<-0;
					
					if remaining_places=0{
						write self.name + " there are are no more available seats (break loop) ";
						break;
					}
					if remaining_places>0{
						list<people> toadd <- (people_on_road[dest]);
						if length(toadd) > remaining_places{
							toadd <- people_on_road[dest] copy_between(0, remaining_places-1);
							write self.name + " dropping some passengers else full (" +toadd + " instead of "+ people_on_road[dest]+ ")";
						}
						if length(toadd) = 0{
							break;
						} 
						
						if int_targets contains dest{
//							origin_exists<-true; 
							index_dest<- ordered_or_dest index_of dest;
							
								
							if people_destinations_for_dropping.keys contains dest{ 
								add all:list<string>(toadd collect each.name) to: people_destinations_for_dropping[dest];
								write self.name +" dest: "+ dest+ " was already in for_dropping "+ people_destinations_for_dropping[dest];
							} 
							added_people<- length(toadd)+ added_people; 
							remaining_places <- remaining_places - length(toadd);
							total_added_people  <- total_added_people + added_people;
														  
							write self.name + " capturing " + toadd collect each.name + " that have as target already in int_targets: " + dest;
							write self.name + " " + int_targets + " " + ordered_or_dest + " " + people_destinations_for_dropping;
							write self.name + " updating costs ";
							
	//						do update_costs_existing;
							do update_costs;
							origin_exists<-true;
							bool next_too<-false;
		
//							float cost<-0.0;
//							loop key over: cost_legs.keys{
//								if key[0]=current_road_nodes[1]{
//									next_too<-true;
//									cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
//								}
//								if next_too{
//									cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
//									if key[1]=dest{break;}
//								}
//							}
							ask toadd{
//								cost_proposed <- cost;
								got_lift<-true;
								self.origin <- myself.current_road_nodes[1];
	//							write myself.name + " " + self.name + " from:"+ self.origin + " " + myself.current_road_nodes[1];
							}
		
							add all: (toadd collect each.waiting_time) to: waiting_times;
							capture toadd as: passenger{
							}
							added<-true;
							update_costs_for_passengers<-true;					
						} 
						else{				
							loop while: added=false{
								if i=1{
									target_to_dest <- road_network path_between(location, dest);
									target_to_next <- road_network path_between(location, int_targets[i]);
									time_t2dest <- (target_to_dest.edges=[])? 0 : target_to_dest.shape.perimeter /8.3;
									time_t2next <- target_to_next.shape.perimeter /8.3;
									write self.name + " index: "+ i + " index_max: "+ index_max;
									write self.name + " origin: "+ location + " destination: "+ dest + " time (@ slowest): " + time_t2dest;
									write self.name + " origin: "+ location + " next target: "+ int_targets[i] + " time (@ slowest): " + time_t2next; 
									
								}else if i< index_max{
									target_to_dest <- road_network path_between(int_targets[i-1], dest);
									target_to_next <- road_network path_between(int_targets[i-1], int_targets[i]);
									time_t2dest <- (target_to_dest.edges=[])? 0 : target_to_dest.shape.perimeter /8.3;
									time_t2next <- target_to_next.shape.perimeter /8.3;
									write self.name + " index: "+ i + " index_max: "+ index_max;
									write self.name + " origin: "+ int_targets[i-1] + " destination: "+ dest + " time (@ slowest): " + time_t2dest;
									write self.name + " origin: "+ int_targets[i-1] + " next target: "+ int_targets[i] + " time (@ slowest): " + time_t2next; 
								}
	
								if target_to_dest.edges!=[] and i<=index_max{
									if target_to_dest.shape.perimeter < target_to_next.shape.perimeter{
										write self.name + " dest: "+ dest + " next_target: "+ int_targets[i];
										path dest_to_target <- road_network path_between (dest, int_targets[i]);
										if dest_to_target.edges!=[]{
											time_dest2next <- dest_to_target.shape.perimeter /8.3;
											float with_change <- (start_to_first - time_t2next) + (time_t2dest + time_dest2next);
											write self.name + " start2first: "+start_to_first +" time_t2next: "+ time_t2next+" time_t2dest: "+ time_t2dest+ " time_dest2next: "+ time_dest2next + " first_remaining_time: " + first_remaining_time;
											if with_change < (start_to_first + (start_to_first/2)) and first_remaining_time > (with_change*8.3/13.8){
												write self.name + " deviation path takes less than 3/2 the time to cover eventual path "+ with_change + " "+ start_to_first + " "+ first_remaining_time;
												write self.name + " and the time remaining for 1st passenger ("+ first_remaining_time +")is monre than time to cover road with change "+ with_change + "="+ (with_change*8.3/13.8);
												
												if !((int_targets-int_targets[0]) contains dest){
													index_dest<- ordered_or_dest index_of int_targets[i];
													write self.name + " index_dest "+ index_dest + "= index of "+ int_targets[i] + " " + int_targets + " in ordered_or_dest: " + ordered_or_dest; 
													add dest at:(index_dest) to: ordered_or_dest; 
													add dest at:i to: int_targets;
												}
												if people_destinations_for_dropping contains dest{
													add all:list<string>(toadd collect each.name) to: people_destinations_for_dropping[dest];
													write self.name +" dest: "+ dest+ " was already in for_dropping "+ people_destinations_for_dropping[dest];
												}else{
													add dest::list<string>(toadd collect each.name) to:people_destinations_for_dropping;
												} 
				
												write self.name + " " + int_targets + " " + ordered_or_dest + " " + people_destinations_for_dropping;
												write self.name + " capturing " + toadd collect each.name + " that have as target: " + dest;
												write self.name + " recomputing costs";
												added_people<- length(toadd)+ added_people;
												total_added_people  <- total_added_people + added_people;
												remaining_places <- remaining_places - length(toadd);
												
												do update_costs;
												origin_exists<-true;
	//											bool next_too<-false;
	//											
	//											float cost<-0.0;
	//											loop key over: cost_legs.keys{
	//												if key[0]=current_road_nodes[1]{
	//													next_too<-true;
	//													cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
	//												}
	//												if next_too{
	//													cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
	//													if key[1]=dest{
	//														break;
	//													}
	//												}
	//											}
												ask toadd{
	//												cost_proposed<- cost;
													got_lift<-true;
													self.origin <- myself.current_road_nodes[1];
		//											write myself.name + " " + self.name + " from:"+ self.origin + " " + myself.current_road_nodes[1];
												}
												add all: (toadd collect each.waiting_time) to: waiting_times;
												capture toadd as: passenger{}						
												
												if i=1{
													the_target<-nil;
													final_target<-nil;
													current_target<-nil;	
												}
												added<-true;
												update_costs_for_passengers<-true;
											} else {
												write self.name + " deviation path takes too much " + with_change + " "+ start_to_first;
												added<-true;
												
											}
										}
									}
								}
								if i=index_max and !added{
									write self.name + " index: "+ i + " index_max: "+ index_max;
									write self.name + " origin (last): "+ last(int_targets) + " destination: "+ dest;
									target_to_dest <- road_network path_between(last(int_targets), dest);
									if target_to_dest.edges!=[]{
										if !((int_targets-int_targets[0]) contains dest){
											index_dest<- length(ordered_or_dest);
											write self.name + " index_dest "+ index_dest + "= new index of "+ ordered_or_dest;
											add dest to: int_targets;
											add dest to: ordered_or_dest;
										}
										
//										list<people> toadd <- (people_on_road[dest]);
//										if length(toadd) > remaining_places{
//											toadd <- people_on_road[dest] copy_between(0, remaining_places-1);
//											write self.name + " dropping some passengers else full (" +toadd + " instead of "+ people_on_road[dest]+ ")";
//										}
										
										if people_destinations_for_dropping contains dest{
											add all:list<string>(toadd collect each.name) to: people_destinations_for_dropping[dest];
											write self.name +" dest: "+ dest+ " was already in for_dropping "+ people_destinations_for_dropping[dest];
										}else{
											add dest::list<string>(toadd collect each.name) to:people_destinations_for_dropping;
										} 
										added_people<- length(toadd)+ added_people;
										total_added_people  <- total_added_people + added_people;
										remaining_places <- remaining_places - length(toadd);
										added_last<-true;
										
																	  
										write self.name + " capturing " + toadd collect each.name + " that have as target: " + dest;
										write self.name + " " + int_targets + " " + ordered_or_dest + " " + people_destinations_for_dropping;
										write self.name + " updating costs (added_last)";
										
										do update_costs;
										origin_exists<-true;
	//									bool next_too<-false;
	
											
	//									float cost<-0.0;
	//									loop key over: cost_legs.keys{
	//										if key[0]=current_road_nodes[1]{
	//											next_too<-true;
	//											cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
	//										}
	//										if next_too{
	//											cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
	//											if key[1]=dest{break;}
	//										}
	//									}
										
										
										ask toadd{
	//										cost_proposed <- cost;
											got_lift<-true;
											self.origin <- myself.current_road_nodes[1];
	//										write myself.name + " " + self.name + " from:"+ self.origin + " " + myself.current_road_nodes[1];
										}
										add all: (toadd collect each.waiting_time) to: waiting_times;
										
										capture toadd as: passenger{}
										added<-true;
										update_costs_for_passengers<-true;
									}
									break;
								}
								i<-i+1;	
							}
							remove all:people_on_road[dest] from: possible_pass;
						}
					} 
//					origin_exists <-true;

				}
				if total_added_people=0{ 
					if (ordered_or_dest index_of current_road_nodes[1]) !=0{
						remove current_road_nodes[1] from: ordered_or_dest;
						write self.name + ' removed '+ current_road_nodes[1] + '(new origin) from: '+ ordered_or_dest;
						
					}
				} else{
					write self.name + " ("+ total_added_people+ ") updated costs "+ cost_legs;
				}
				stats_first_path_time <- machine_time - stats_first_path_time;
				n_addition <- n_addition + 1;
				write "addition "+((car index_of self)<10 ? "0": "") + string(car index_of self) + ", "+ n_travel+ ", "+n_addition+", "+ stats_first_path_time+", "+ total_added_people+", "+ total_available_people + ", "+ initial_stops + ", "+ dest_length + ", "+ length(ordered_or_dest);
			}
		}
		state<-'moving';
		people_near<-nil;
		possible_pass<-nil;
		self.location<- current_road_nodes[1].location;
		total_passengers_travel <- total_passengers_travel + total_added_people; 
	}
	
	reflex drop_people when:!empty(passenger) and state='stop'{
		list<string>names;
		list<string> states_dropped;
		list<people>dropped;
		string substitute_state;
		
//		write self.name + " Passengers on before droppping someone: " + list(passenger);
//		write self.name + " People destinations: " + people_destinations_for_dropping + " current_target "+ the_target ;
		loop p over: (passenger){
			if people_destinations_for_dropping[the_target] contains p.name and self.location=the_target.location{
				add p.cost_proposed to: final_costs; 
//				write self.name + " People destinations[target]: " + people_destinations_for_dropping[the_target] + " - p "+ p + "="+ p.name;
				add p to: dropped;
				add p.name to: names;
				point t<-p.the_target;
				if p.the_target!=location{
					if (p.next_state contains 'work') { //next_state
						substitute_state<-'go_work';
					}
					if (p.next_state='resting' or p.next_state='go_home') {
						substitute_state<-'go_home';
					}
				}else{
					substitute_state<-p.next_state;
				}
				add substitute_state to: states_dropped;
				release p in:world as:people{
					state<- substitute_state;
				}
			}
		}
		if !empty(dropped){
			write string(self.name) +' ('+current_hour+') dropped '+(names) + ' with states: '+ states_dropped; //+' @ '+ location +' with: '+p_targets+ ' - ' +state;
		}
		
		remove the_target from: int_targets;
		remove key:the_target from: people_destinations_for_dropping;
		write self.name + ' removed the_target from int_targets and from people_destinations and then the_target is set to nil';
		write self.name + ' - '+ people_destinations_for_dropping;
		the_target<-nil;
		if empty(passenger){
			write "Travel_TOT " +((car index_of self)<10 ? "0": "") + string(car index_of self) + ", "+ n_travel+ ", "+total_passengers_travel+", "+ length(ordered_or_dest)+", "+ waiting_times+", "+ final_costs;
			n_travel<-n_travel+1;
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

experiment Decentralised type: gui {
	float minimum_cycle_duration <- 0.01;
	
	parameter "people" var: nb_people <- 100 ;
    parameter "cars" var: nb_car <- ((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5) ;
//	parameter "if true, simple data (simple track), if false complex one (Manhattan):" var: simple_data category: "GIS" ;
	output {
		monitor "Current hour" value: current_hour;
//		monitor "Tot distance cars" value: cars_tot_distance_covered/1000;
//		monitor "Tot distance people" value: people_tot_distance_covered/100;
//		monitor "Tot distance covered" value: sys_tot_distance_covered/1000;
		
		display city_display {
			graphics "world" {
				draw world.shape.contour;
			}
			species building aspect: base refresh:false;
			species road aspect: base ;
			species intersection aspect: base;
			species car aspect: base;
			species people aspect: base transparency: 0.8;
			
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