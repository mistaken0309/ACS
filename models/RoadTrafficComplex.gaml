model RoadTrafficComplex
 
global {   
	bool simple_data <- false;
	
	//Check if we use simple data or more complex roads
//	file shape_file_buildings  <- file("../includes/buildings.shp");
	
	file shape_file_roads  <- file("../includes/roads.shp") ;
	file shape_file_nodes  <- file("../includes/nodes.shp");
	file shape_file_buildings  <- file("../includes/buildings.shp");
	geometry shape <- envelope(shape_file_roads) + 50.0;
	
	list<geometry> zones<- to_rectangles(shape, {shape.width/4,shape.height/4});
	
	graph the_graph;  
	graph road_network;  
	map road_weights;
	
	int nb_people <- 300;
	int nb_cars <- ((nb_people mod 5) >0) ? int(nb_people/5) + 1 : int(nb_people/5);
	int nb_car <- 5; //((nb_people mod 5) >0) ? int(nb_people/10) + 1 : int(nb_people/10);
	 
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
		create road from: shape_file_roads with:[lanes::int(read("lanes")), oneway::string(read("oneway"))] {
			geom_display <- shape + (2.5 * lanes);
			maxspeed <- (lanes = 1 ? 30.0 : (lanes = 2 ? 50.0 : 70.0)) °km/°h;
			switch oneway {
				match "no" {
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
				match "-1" {
					shape <- polyline(reverse(shape.points));
				}	
			}
		}		
		ask road where ((each.linked_road)=nil){
			create road {
				lanes <- myself.lanes;
				shape <- polyline(reverse(myself.shape.points));
				maxspeed <- myself.maxspeed;
				geom_display  <- myself.geom_display;
				linked_road <- myself;
				myself.linked_road <- self;
			}
		}
		

		road_weights <- road as_map (each::((each.shape.perimeter / each.maxspeed)));
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
		road_weights <- road as_map (each::((each.shape.perimeter / each.maxspeed) * (each.speed_coeff)));
		road_network <- road_network with_weights road_weights;
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
	float perim<-shape.perimeter;
	int nb_agents<-length(all_agents) update: length(all_agents);
	float capacity <- 1+(shape.perimeter*lanes)/v_length;
//	float speed_coeff <- 0.1 update: (1.0- exp(-nb_agents/capacity)) min: 0.1;
	float speed_coeff<- 0.0 update: (length(all_agents)/capacity) min:0.0; // = 1 max capience reached // <0 still space in // >1 should avoid it
	
	int ov_rgb<-150 update: 150-(150*int(speed_coeff)) min: 0 max:255; //0 ->150 // 1 e oltre -> 0
	int ov_rgbR<-150 update: 255*int(speed_coeff)  min: 150 max: 255;
//	int ov_rgb<-150 update: int((1.1-speed_coeff)*255)-105 min:0; //0
//	int ov_rgbR<-150 update: int(speed_coeff*255) min:150; //255
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
//	float look_up<- 100.0;
//	list<intersection> close_intersections <- intersection overlapping (self.shape+look_up) ;
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
	bool late<-false;
	float time_needed<-0.0;
	float actual_time_in;
	map road_knowledge<-road_weights update: road_weights;
	path path_to_follow<-nil;
	float look_up<-50.0;
	string next_state<-nil;
	bool got_lift<-false;
	list<intersection> close_intersections <- nil 
		update: ((state contains 'search_lift') ) ? intersection overlapping (self.shape+look_up) : nil;
	
	path path_to_cover;
	float distance_to_cover;
	float time_to_cover;
	float cost_to_cover<-0.0;
	float cost_proposed<-0.0;
	bool up_costs<-true;
	
	reflex update_costs when: (state contains 'search_lift') and the_target!=nil and up_costs=true{
//		write self.name + " " +the_target;
		path_to_cover <- path_between(the_graph, location,the_target);
		distance_to_cover <- path_to_cover.shape.perimeter;
		time_to_cover<-0.0;
		loop e over: path_to_cover.edges{
			time_to_cover <- time_to_cover + (road(e).shape.perimeter / 5.0 );
		}
		cost_to_cover<- (distance_to_cover/1000)*cost_km;
		up_costs<-false;
	}
	
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
			up_costs<-true;
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
			up_costs<-true;
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
	
	reflex movement when: the_target!=nil and path_to_follow=nil and(state="search_lift_home" or state="search_lift_work"){
		if (path_to_follow = nil) {
			//Find the shortest path using the agent's own weights to compute the shortest path
			path_to_follow <- path_between(the_graph with_weights road_knowledge, location,the_target);
			if path_to_follow!=nil{
				list<geometry> segments <- path_to_follow.segments;
				loop seg over:segments{
					dist <- (dist + seg.perimeter );
					time_needed<- (time_needed + (seg.perimeter/(speed)));
					
				}
			}
		}
	}
	reflex move when: path_to_follow!=nil and (state contains 'go_'){
		//the agent follows the path it computed but with the real weights of the graph
		do follow path:path_to_follow speed: 5.0#m/#s move_weights: road_weights;
		if the_target = location {
			dist_covered_alone<-dist_covered_alone + dist;
			dist<-0.0;
			the_target <- nil ;
			time_needed<-0.0;
		}
	}
	aspect base {
//		draw triangle(50) color: color rotate: 90 + heading;
		draw square(50) color: color;
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
	
	path eventual_path;
	map<intersection, list<people>> people_destinations;
	list<people> first_group;
	map<intersection, list<people>> give_lift_to;
	map<intersection, list<string>> people_destinations_for_dropping;
	map<intersection, list<float>> people_costs;
//	map<intersection, float> cost_legs;
	map<list<intersection>, list<float>> cost_legs;
	list<intersection> ordered_or_dest;
	bool added_last;
	int added_people;
	int origin_index;
	int index_dest;
	bool origin_exists<-false;
	
	list<float> costs_passengers<-nil; // list of passengers
	float mean_of_costs<-0.0;
	float max_of_costs<-0.0;
	float min_of_costs<-0.0;
	
	int max_passengers<-5;
	list<intersection> current_road_nodes;
	
	list<intersection> int_targets <- nil;
	list<intersection> int_origins <- nil;
	
	float arrived_time<-0.0;
	float starting_time<-0.0;
	
	people first_p;
	list<people> possible_pass;
	list<people> people_near <- 
				(length(passenger) < max_passengers and current_road_nodes=nil and state!='real_stop') ? nil : (people where ((each.state contains 'search_lift') and each.close_intersections contains_any self.current_road_nodes))
				update: (length(passenger) < max_passengers and current_road_nodes=nil and state!='real_stop') ? nil : (people where ((each.state contains 'search_lift') and each.close_intersections contains_any self.current_road_nodes)); 
	
	
	
	state wander initial: true{}
	state moving{} 
	state stop{}
	state real_stop{}
	
	reflex update_road_nodes when: current_road!=nil and state!='real_stop'{
		if current_road_nodes contains intersection(road(current_road).target_node){
			
		}else{
			current_road_nodes<-nil;
			add intersection(road(current_road).target_node) to:current_road_nodes;
			add intersection(road(current_road).source_node) to:current_road_nodes;	
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
		write "\n"+self.name + ' changing state to real_stop';
		state<-'real_stop';
		
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
				do reset_vars_for_wander;
			}
		}	
	}
	reflex check_other_cars_first_p when: first_p!=nil and state='real_stop'{
		ask (car-self) where (each.state='real_stop' and each.first_p=first_p){
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
	reflex change_first_passenger when: first_p=nil and state='real_stop'{
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
				
		// check in order to avoid to compute the current path again for other passengers that go to the same location;		
		if people_destinations.keys contains last(int_targets){
			first_group<- people_destinations[last(int_targets)] + first_p;
			remove people_destinations[last(int_targets)] from: possible_pass; // are already going to be captured;
			remove key: last(int_targets) from: people_destinations;
		}
	}
	
	
	reflex create_path when: !empty(possible_pass+first_p) and state='real_stop' and first_p!=nil{ 
		intersection origin<- current_road_nodes[1];
		first_group <- [first_p];
		
		intersection first_p_target_inter <- (intersection closest_to first_p.the_target);
		
		eventual_path <- road_network path_between (origin,first_p_target_inter);
		float time_for_eventual_path<-0.0;
		loop e over: (eventual_path.edges){
			time_for_eventual_path <- time_for_eventual_path + (road(e).shape.perimeter / road(e).maxspeed);
		}
		
		write self.name + " First passenger "+ first_p.name + " ("+ first_p +") " +" has objective "+ first_p.the_target +" and the nearest "+ first_p_target_inter.name;
		write  self.name + " other possible pass : "+ possible_pass + "\n";
		
		add origin to: int_targets; // formally add starting point
		add origin to: int_origins;
		add first_p_target_inter to: int_targets; // formally add ending point
		write self.name + " - " +int_targets;
		
		do remove_due_to_direction;
				
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
//						write self.name+ " remaining possible passengers: "+ possible_pass +"\n";
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
				float distance_leg <- 0.0;
				float cost_leg <- 0.0;
				
				
					
				path leg_path <- road_network path_between (int_targets[i], dest);
//				write self.name + " path between "+ int_targets[i] + " to "+ dest +" "+leg_path;
				distance_leg <- leg_path.shape.perimeter;
					
				loop e over: list<road>(leg_path.edges){
					time_leg <- time_leg + (e.shape.perimeter / e.maxspeed);
				}
	
				cost_leg <- ((distance_leg/1000) * cost_km)/leg_passengers;
				if i>0{
					sum_time_leg <- time_leg + people_costs[int_targets[i]][0];
					sum_distance_leg <- distance_leg + people_costs[int_targets[i]][1];
					sum_cost_leg <- ((distance_leg/1000 * cost_km)/leg_passengers) + people_costs[int_targets[i]][2];
				}
					
				list<float> costs <- [sum_time_leg, sum_distance_leg, sum_cost_leg];
				add dest::costs to: people_costs;
				
				add [(int_targets[i]), dest]::[((distance_leg/1000) * cost_km),leg_passengers]  to: cost_legs;
				
				list<string> names<- nil;	
				loop p over: people_destinations[dest]{
					ask p{
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
			capture possible_pass as:passenger{
			}
			
			write self.name + ' final destinations: '+people_destinations.keys+ " | final ordered_or_dest: " + ordered_or_dest;
			write self.name + ' initial costs per leg: '+ cost_legs; 
			
			self.location<- origin.location;
//			do search_other_passengers;
			state<-'moving';
			possible_pass<-nil;
			final_target <- nil;
			the_target <- nil;
			current_path <- nil;
			people_near<-nil;
		} else{
			write self.name + " Couldn't get anybody, going back to wandering";
			do reset_vars_for_wander;
		}
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
	
	action update_costs{
		int difference_in_dest_or <- (index_dest - origin_index)-1;
		write self.name + " origin_index: " + origin_index + " index_dest: " + index_dest + " targets in between: " + difference_in_dest_or;
		int stop_at;
		
		int i<-0;
		bool take_existing<-false;
		map<list<intersection>, list<float>> tmp <- cost_legs;
		cost_legs<-nil;
		write self.name + " " + tmp.keys + " length: "+ length(tmp.keys);
		loop key over: tmp.keys{
			write self.name + " VALUE OF I: " + i;
			if i<origin_index-1{
				write self.name + " i:"+ i+ " before origin_index | copying " + key +"::" + tmp[key] +" to: cost_legs";
				add key::tmp[key] to: cost_legs;
			}
			if (origin_index>0 and i=origin_index-1) or (origin_index=0 and i=0){
				if key[1] = ordered_or_dest[origin_index] and origin_index!=0{
					write self.name + " i:"+ i+ " origin already interted | copying " + key +"::" + tmp[key] +" to: cost_legs";
					add key::tmp[key] to: cost_legs;
					if difference_in_dest_or = 0{
						write self.name + " the destination is right after the origin " + origin_index + " " + index_dest + " " + key[0] + " " + key[1];
						write self.name + " leg2 - from: " + ordered_or_dest[origin_index] + " to: " +ordered_or_dest[index_dest] + " " + tmp[key][1] + " "+added_people;
						write self.name + " leg3 - from: " + ordered_or_dest[index_dest] + " to: " + key[1] + " "+ tmp[key][1];
						path leg2 <- road_network path_between (ordered_or_dest[origin_index], ordered_or_dest[index_dest]);
						int people_on_leg2 <- int(tmp[key][1]+added_people);
						float cost2 <- ((leg2.shape.perimeter/1000)*cost_km) / people_on_leg2;
						add [ordered_or_dest[origin_index], ordered_or_dest[index_dest]]::[cost2, people_on_leg2] to: cost_legs;
						
						path leg3 <- road_network path_between (ordered_or_dest[index_dest], key[1]);
						int people_on_leg3 <- int(tmp[key][1]);
						float cost3 <- ((leg3.shape.perimeter/1000)*cost_km) / people_on_leg3;
						add [ordered_or_dest[index_dest], key[1]]::[cost3, people_on_leg3] to: cost_legs;
					}
				} else{
					if origin_index!=0{
						write self.name + " " + i +" origin_index!=0 / i=origin_index-1 | leg1 - from: " + key[0] + " to: " + ordered_or_dest[origin_index] + " " + key[0] + " " + key[1] + " " + tmp[key][1] + " " + added_people;
						path leg1 <- road_network path_between (key[0], ordered_or_dest[origin_index]);
						int people_on_leg1 <- int(tmp[key][1]);
						float cost1 <- ((leg1.shape.perimeter/1000)*cost_km) / people_on_leg1;
						add [key[0], ordered_or_dest[origin_index]]::[cost1, people_on_leg1] to: cost_legs;
					}
					if difference_in_dest_or = 0{
						write self.name + " the destination is right after the origin " + origin_index + " " + index_dest + " " + key[0] + " " + key[1];
						write self.name + " leg2 - from: " + ordered_or_dest[origin_index] + " to: " +ordered_or_dest[index_dest] + " " + tmp[key][1] + " "+added_people;
						write self.name + " leg3 - from: " + ordered_or_dest[index_dest] + " to: " + key[1] + " "+ tmp[key][1];
						path leg2 <- road_network path_between (ordered_or_dest[origin_index], ordered_or_dest[index_dest]);
						int people_on_leg2 <- int(tmp[key][1]+added_people);
						float cost2 <- ((leg2.shape.perimeter/1000)*cost_km) / people_on_leg2;
						add [ordered_or_dest[origin_index], ordered_or_dest[index_dest]]::[cost2, people_on_leg2] to: cost_legs;
						
						path leg3 <- road_network path_between (ordered_or_dest[index_dest], key[1]);
						int people_on_leg3 <- int(tmp[key][1]);
						float cost3 <- ((leg3.shape.perimeter/1000)*cost_km) / people_on_leg3;
						add [ordered_or_dest[index_dest], key[1]]::[cost3, people_on_leg3] to: cost_legs;
						
					}else {
						write self.name + " the destination is NOT right after the origin "+ origin_index + " " + index_dest + " "  + key[0] + " " + key[1] + " " + tmp[key][1] + " " + added_people;
						write self.name + " leg2 - from: " + ordered_or_dest[origin_index] + " to: " + key[1]+ " "+ tmp[key][1] + "+" + added_people;
						path leg2 <- road_network path_between (ordered_or_dest[origin_index], key[1]);
						int people_on_leg2 <- int(tmp[key][1]+added_people);
						float cost2 <- ((leg2.shape.perimeter/1000)*cost_km) / people_on_leg2;
						add [ordered_or_dest[origin_index], key[1]]::[cost2, people_on_leg2] to: cost_legs;
					}
				}
			}
			if !added_last and ((origin_exists and i>=origin_index and i<index_dest-1  and difference_in_dest_or>0)
				or (!origin_exists and i>=origin_index and i<index_dest-2 and origin_index>0 )){
				write self.name + " origin_exist: " + origin_exists + " index_dest-1: " + (index_dest-1) + " index_dest-2: " + (index_dest-2) + " "  + difference_in_dest_or; 
				write self.name + " i:"+ i+ " after origin and before dest | copying " + key +"::" + tmp[key][0] + " " + tmp[key][1]  + "+" + added_people+" with added_people to: cost_legs";
				add key::[tmp[key][0], tmp[key][1]+ added_people] to: cost_legs;
			}
			if !added_last and ((origin_exists and i = index_dest-1 and i>=origin_index and difference_in_dest_or>0)
				or (!origin_exists and i = index_dest-2 and i>=origin_index and origin_index>0)){
					if key[1]= ordered_or_dest[index_dest]{
						write self.name + " i:"+ i+ " destination already interted | copying " + key +"::" + tmp[key][0] + " " + tmp[key][1] +"+" +added_people +" to: cost_legs";
						add key::[tmp[key][0], tmp[key][1]+ added_people] to: cost_legs;
					} else{
						write self.name + " leg1 - from: " + key[0] + " to: " +ordered_or_dest[index_dest] + " " + tmp[key][1] + " "+added_people;
						write self.name + " leg2 - from: " + ordered_or_dest[index_dest] + " to: " + key[1] + " "+ tmp[key][1];
						path leg1 <- road_network path_between (key[0], ordered_or_dest[index_dest]);
						int people_on_leg1 <- int(tmp[key][1]+added_people);
						float cost1 <- ((leg1.shape.perimeter/1000)*cost_km) / people_on_leg1;
						add [key[0], ordered_or_dest[index_dest]]::[cost1, people_on_leg1] to: cost_legs;
						
						path leg2 <- road_network path_between (ordered_or_dest[index_dest], key[1]);
						int people_on_leg2 <- int(tmp[key][1]);
						float cost2 <- ((leg2.shape.perimeter/1000)*cost_km) / people_on_leg2;
						add [ordered_or_dest[index_dest], key[1]]::[cost2, people_on_leg2] to: cost_legs;
					}
			}
			write self.name + " added_last: "+ added_last + " origin_exists "+ origin_exists + " index_dest-1 " + (index_dest-1) + " index_dest-2 " + (index_dest-2) + " origin_index " + origin_index; 
			if !added_last and (
				(origin_exists and i >= index_dest and difference_in_dest_or >0)
				or (origin_exists and i >= index_dest-1 and difference_in_dest_or =0)
				or (!origin_exists and i >= index_dest-1) ){
				write self.name + " i:"+ i+ " after dest | copying " + key +"::" + tmp[key] +" to: cost_legs";
				add key::tmp[key] to: cost_legs;
			}
			if added_last and i>=origin_index{
				write self.name + " i:"+ i+ " after origin and added_last | copying " + key +"::" + tmp[key][0] + " " + tmp[key][1]  + "+" + added_people+" with added_people to: cost_legs";
				add key::[tmp[key][0], tmp[key][1]+ added_people] to: cost_legs;
			}
			
			i<-i+1;
		}
		if added_last{
			intersection key0 <- ordered_or_dest[length(ordered_or_dest)-2];
			intersection key1 <- last(ordered_or_dest);
			write self.name + ' adding as last. From '+ key0 +' to ' + key1;
			path leg1 <- road_network path_between (key0, key1);
			int people_on_leg1 <- added_people;
			float cost1 <- ((leg1.shape.perimeter/1000)*cost_km) / people_on_leg1;
			add [key0, key1]::[cost1, people_on_leg1] to: cost_legs;
			added_last<-false;
			write self.name + " " + cost_legs;
		}
		
	}
	
//	action update_costs_existing{
//		int difference_in_dest_or <- (index_dest - origin_index)-1;
//		write self.name + " modifying when already exiting | origin_index: " + origin_index + " index_dest: " + index_dest + " targets in between: " + difference_in_dest_or;
//		int stop_at;
//		
//		int i<-0;
//		bool take_existing<-false;
//		loop key over: cost_legs.keys{
//			if i>=origin_index and i<index_dest{
//				write self.name + " " + key + " original:" + cost_legs[key] + " after change: "+ cost_legs[key][1]+added_people;
//				cost_legs[key][1]<- cost_legs[key][1]+added_people;
//			}
//			if i=index_dest{ break; }
//			i<-i+1;
//		}
//		write self.name + " updated (existing) cost_legs: " + cost_legs;
//	}
	
	reflex add_on_road when: !empty(people_near) and (state='moving') and length(passenger)< max_passengers{ //state!='wander' or state!='real_stop' or state!='stop'
		state<-'real_stop';
//		write self.name + " changing to real_stop on the road to add passengers";
		possible_pass<-people_near;
		do remove_due_to_direction;
//		write self.name + " remove passengers due to direction";
		if !empty(possible_pass){
			write self.name + " there are passengers on the " + current_road_nodes[1];
			map<intersection, list<people>> people_on_road <- possible_pass group_by (intersection closest_to each.the_target);
			write self.name + " initial people_on_road: "+ people_on_road;
			
			if people_on_road.keys contains int_targets[0]{
				remove key: int_targets[0] from: people_on_road;
				write self.name + " removed key: " + int_targets[0];
			}
			int total_added_people<-0;

			origin_exists<-false;
//			if !(ordered_or_dest contains current_road_nodes[1]){
				int inted <- ((ordered_or_dest index_of the_target));
				origin_index<- inted-1;
				if ordered_or_dest[inted-1]!=current_road_nodes[1]{
					add current_road_nodes[1] at:inted to: ordered_or_dest;
					write self.name + ' added '+ current_road_nodes[1] + '(new origin) at: ' + inted + ' to:'+ ordered_or_dest;
					origin_index <- inted;
				} else{
					origin_exists<-true;
				}
//			}
			
//			if length(passenger)>= max_passengers{
			loop dest over: people_on_road.keys{
				path target_to_dest ;
				path target_to_next ;
				bool added<- false;
				int i<-1;
				int index_max <- length(int_targets)-1;
				added_people<-0;
				if max_passengers > length(passenger){
					if int_targets contains dest{
						origin_exists<-true;
						index_dest<- ordered_or_dest index_of dest;
						if people_destinations_for_dropping.keys contains dest{ 
							add all:(people_on_road[dest] collect each.name) to: people_destinations_for_dropping[dest];
							write self.name +" dest: "+ dest+ " was already in for_dropping "+ people_destinations_for_dropping[dest];
						} 
						
						added_people<- length(people_on_road[dest])+ added_people;
						total_added_people  <- total_added_people + added_people;
													  
						write self.name + " capturing " + people_on_road[dest] collect each.name + " that have as target already in int_targets: " + dest;
						write self.name + " " + int_targets + " " + ordered_or_dest + " " + people_destinations_for_dropping;
						write self.name + " updating costs ";
						
//						do update_costs_existing;
						do update_costs;
						bool next_too<-false;
	
						float cost<-0.0;
						loop key over: cost_legs.keys{
							if key[0]=current_road_nodes[1]{
								next_too<-true;
								cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
							}
							if next_too{
								cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
								if key[1]=dest{break;}
							}
						}
						ask people_on_road[dest]{
							cost_proposed <- cost;
							got_lift<-true;
						}
	
						
						capture people_on_road[dest] as: passenger{}
						write self.name + " first passenger that got on from remaining " + first(list(passenger)) + " (" +first(list(passenger)).name + ")";
						added<-true;
						
					} else{				
						loop while: added=false{
							if i=1{
								write self.name + " index: "+ i + " index_max: "+ index_max;
								write self.name + " origin: "+ location + " destination: "+ dest;
								write self.name + " origin: "+ location + " next target: "+ int_targets[i]; 
								target_to_dest <- road_network path_between(location, dest);
								target_to_next <- road_network path_between(location, int_targets[1]);
							}else if i< index_max{
								write self.name + " index: "+ i + " index_max: "+ index_max;
								write self.name + " origin: "+ int_targets[i-1] + " destination: "+ dest;
								write self.name + " origin: "+ int_targets[i-1] + " next target: "+ int_targets[i]; 
								target_to_dest <- road_network path_between(int_targets[i-1], dest);
								target_to_next <- road_network path_between(int_targets[i-1], int_targets[i]);
							}
		
							if target_to_dest.edges!=[] and i<=index_max{
								if target_to_dest.shape.perimeter < target_to_next.shape.perimeter{
									write self.name + " dest: "+ dest + " next_target: "+ int_targets[i];
									path dest_to_target <- road_network path_between (dest, int_targets[i]);
									if dest_to_target.edges!=[]{
										origin_exists<-true;
										if !((int_targets-int_targets[0]) contains dest){
											index_dest<- ordered_or_dest index_of int_targets[i];
											write self.name + " index_dest "+ index_dest + "= index of "+ int_targets[i] + " " + int_targets + " in ordered_or_dest: " + ordered_or_dest; 
											add dest at:(index_dest) to: ordered_or_dest; 
											add dest at:i to: int_targets;
										}
										if people_destinations_for_dropping contains dest{
											add all:(people_on_road[dest] collect each.name) to: people_destinations_for_dropping[dest];
											write self.name +" dest: "+ dest+ " was already in for_dropping "+ people_destinations_for_dropping[dest];
										}else{
											add dest::(people_on_road[dest] collect each.name) to:people_destinations_for_dropping;
										} 
		
										write self.name + " " + int_targets + " " + ordered_or_dest + " " + people_destinations_for_dropping;
										write self.name + " capturing " + people_on_road[dest] collect each.name + " that have as target: " + dest;
										write self.name + " recomputing costs";
										added_people<- length(people_on_road[dest])+ added_people;
										total_added_people  <- total_added_people + added_people;
										
										do update_costs;
										bool next_too<-false;

										
										
										float cost<-0.0;
										loop key over: cost_legs.keys{
											if key[0]=current_road_nodes[1]{
												next_too<-true;
												cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
											}
											if next_too{
												cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
												if key[1]=dest{
													break;
												}
											}
										}
										ask people_on_road[dest]{
											cost_proposed<- cost;
											got_lift<-true;
										}
										capture people_on_road[dest] as: passenger{}						
										
										if i=1{
											the_target<-nil;
											final_target<-nil;
											current_target<-nil;	
										}
										added<-true;
										write self.name + " first passenger that got on from remaining " + first(list(passenger)) + " (" +first(list(passenger)).name + ")";
										
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
									if people_destinations_for_dropping contains dest{
										add all:(people_on_road[dest] collect each.name) to: people_destinations_for_dropping[dest];
										write self.name +" dest: "+ dest+ " was already in for_dropping "+ people_destinations_for_dropping[dest];
									}else{
										add dest::(people_on_road[dest] collect each.name) to:people_destinations_for_dropping;
									} 
									added_people<- length(people_on_road[dest])+ added_people;
									total_added_people  <- total_added_people + added_people;
									added_last<-true;
									
																  
									write self.name + " capturing " + people_on_road[dest] collect each.name + " that have as target: " + dest;
									write self.name + " " + int_targets + " " + ordered_or_dest + " " + people_destinations_for_dropping;
									write self.name + " updating costs (added_last)";
									
									do update_costs;
									bool next_too<-false;

										
									float cost<-0.0;
									loop key over: cost_legs.keys{
										if key[0]=current_road_nodes[1]{
											next_too<-true;
											cost <-  cost + (cost_legs[key][0]/cost_legs[key][1]);
										}
										if next_too{
											cost <- cost + (cost_legs[key][0]/cost_legs[key][1]);
											if key[1]=dest{break;}
										}
									}
									ask people_on_road[dest]{
										cost_proposed <- cost;
										got_lift<-true;
									}
		
									
									capture people_on_road[dest] as: passenger{}
									write self.name + " first passenger that got on from remaining " + first(list(passenger)) + " (" +first(list(passenger)).name + ")";
									added<-true;
								}
								break;
							}
							i<-i+1;
							
								
						}
						remove all:people_on_road[dest] from: possible_pass;
						if length(passenger) = max_passengers{
							write self.name + " breaking the loop";
							break;
						}
					}
			
				}
			}
			if total_added_people=0{
				write self.name + ' removed '+ current_road_nodes[1] + '(new origin) from: '+ ordered_or_dest;
				remove current_road_nodes[1] from: ordered_or_dest;
			} else{
				write self.name + " ("+ total_added_people+ ") updated costs "+ cost_legs;
			}
		}
		state<-'moving';
		self.location<- current_road_nodes[1].location;
	}
	
	reflex drop_people when:!empty(passenger) and state='stop'{
		list<string>names;
		list<string> states_dropped;
		list<people>dropped;
		string substitute_state;
		
		write self.name + " Passengers on before droppping someone: " + list(passenger);
		write self.name + " People destinations: " + people_destinations_for_dropping + " current_target "+ the_target ;
		loop p over: (passenger){
			if people_destinations_for_dropping[the_target] contains p.name and self.location=the_target.location{
				write self.name + " People destinations[target]: " + people_destinations_for_dropping[the_target] + " - p "+ p + "="+ p.name;
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
	float dist_covered_cars<-0.0;

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

experiment experiment_2D type: gui {
	float minimum_cycle_duration <- 0.01;
//	parameter "if true, simple data (simple track), if false complex one (Manhattan):" var: simple_data category: "GIS" ;
	output {
		monitor "Current hour" value: current_hour;
//		monitor "Tot distance cars" value: cars_tot_distance_covered/1000;
//		monitor "Tot distance people" value: people_tot_distance_covered/100;
//		monitor "Tot distance covered" value: sys_tot_distance_covered/1000;
		
//		display city_display {
//			graphics "world" {
//				draw world.shape.contour;
//			}
//			species building aspect: base refresh:false;
//			species road aspect: base ;
//			species intersection aspect: base;
//			species car aspect: base;
//			species people aspect: base;
//			
//		}
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
